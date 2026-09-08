/*
 * BLOCK 7 — VEHICLE INTERFACE FIRMWARE SKELETON (owner: Controls & Actuation)
 * Target: custom STM32 ECU (adapt HAL handles/pins to your board).
 *
 * This file is a SKELETON: the safety-critical logic (AS state machine,
 * command timeout, CRC/counter validation, clamping) is complete; the
 * hardware bindings (CAN peripheral init, PWM, GPIO, encoder) are stubs
 * marked TODO(board).
 *
 * CAN map (500 kbps, matches fsd_stack/can_bridge_node.py):
 *   RX 0x100  steering(i16 mrad) torque(i16 0.1Nm) brake(u8 0-200) flags(u8)
 *   RX 0x101  rolling counter(u8) + CRC8(poly 0x31, init 0xFF) of 0x100
 *   TX 0x200  actual steering(i16 mrad) rpm(i16) as_state(u8) faults(u16)
 *   TX 0x201  wheel speeds fl/fr/rl/rr (4x u16, 0.01 rad/s)
 *   TX 0x210  ebs pressure(u16 0.01bar) motor_t(i8) inv_t(i8) lv(u16 0.01V)
 *
 * NON-NEGOTIABLE INVARIANTS (bench-verify per spec section 9):
 *   1. No valid 0x100+0x101 pair within 100 ms  -> zero torque, hold steer;
 *      a further 100 ms -> EBS engage.
 *   2. Torque is forwarded to the Bamocar ONLY in AS_DRIVING.
 *   3. All commands are clamped HERE, never trusted from the Jetson.
 *   4. AS_EMERGENCY latches until physical power-cycle / manual reset.
 *   5. RES and SDC act on hardware lines; this firmware only OBSERVES them.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ------------------------------------------------------------- limits */
#define STEER_LIMIT_MRAD      350   /* 0.35 rad at tire                  */
#define STEER_SLEW_MRAD_10MS  16    /* ~90 deg/s at tire                 */
#define TORQUE_LIMIT_DNM      300   /* 30.0 Nm                           */
#define CMD_TIMEOUT_MS        100
#define EBS_GRACE_MS          100
#define SUPERVISOR_TIMEOUT_MS 500   /* ebs keepalive relay via flags     */

/* ------------------------------------------------------------- state */
typedef enum {
    AS_OFF = 0, AS_READY, AS_DRIVING, AS_FINISHED, AS_EMERGENCY
} as_state_t;

typedef struct {
    int16_t steer_mrad;
    int16_t torque_dnm;
    uint8_t brake;              /* 0-200 = 0.0-1.0 */
    uint8_t flags;              /* bit0 = e-stop request */
    bool    valid;              /* CRC + counter checked */
} cmd_t;

static volatile as_state_t as_state = AS_OFF;
static volatile cmd_t      cmd;
static volatile uint32_t   last_cmd_ms   = 0;
static volatile uint32_t   now_ms        = 0;   /* incremented by SysTick */
static volatile uint8_t    expected_ctr  = 0;
static volatile bool       ctr_synced    = false;
static volatile uint8_t    crc_fail_run  = 0;
static volatile uint16_t   fault_flags   = 0;
static int16_t             steer_target_mrad = 0;
static int16_t             steer_actual_mrad = 0;  /* from BLDC encoder */

/* fault bits */
#define FAULT_CMD_TIMEOUT  (1u << 0)
#define FAULT_CRC          (1u << 1)
#define FAULT_SDC_OPEN     (1u << 2)
#define FAULT_LV_UNDERVOLT (1u << 3)
#define FAULT_SUPERVISOR   (1u << 4)

/* ------------------------------------------------------------- CRC8 */
static uint8_t crc8(const uint8_t *d, uint8_t n)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ------------------------------------------------- hardware stubs */
/* TODO(board): implement against your HAL. */
static void hw_init(void);
static bool hw_sdc_closed(void);            /* shutdown circuit state    */
static bool hw_asms_on(void);
static bool hw_res_go(void);                /* RES 'go' received         */
static bool hw_mission_selected(void);
static bool hw_standstill(void);
static bool hw_mission_complete(void);
static uint16_t hw_lv_centivolt(void);
static void hw_ebs_engage(void);            /* vent solenoid -> brakes   */
static void hw_bamocar_torque(int16_t dnm); /* Bamocar reg 0x90 frame    */
static void hw_steer_setpoint(int16_t mrad);/* 1 kHz BLDC position loop  */
static void hw_assi(as_state_t s);          /* lights + buzzer per rules */
static void can_send(uint16_t id, const uint8_t *data, uint8_t len);

/* --------------------------------------------------- CAN reception */
static uint8_t pending_payload[6];
static bool    pending_100 = false;

void can_rx_isr(uint16_t id, const uint8_t *d, uint8_t len)
{
    if (id == 0x100 && len >= 6) {
        memcpy(pending_payload, d, 6);
        pending_100 = true;
    } else if (id == 0x101 && len >= 2 && pending_100) {
        pending_100 = false;
        bool crc_ok = (crc8(pending_payload, 6) == d[1]);
        bool ctr_ok = !ctr_synced || (d[0] == expected_ctr);
        expected_ctr = (uint8_t)(d[0] + 1);
        ctr_synced = true;

        if (!crc_ok || !ctr_ok) {
            if (++crc_fail_run >= 2) {          /* two strikes = timeout */
                fault_flags |= FAULT_CRC;
                last_cmd_ms = 0;
            }
            return;
        }
        crc_fail_run = 0;
        cmd.steer_mrad = (int16_t)(pending_payload[0] | (pending_payload[1] << 8));
        cmd.torque_dnm = (int16_t)(pending_payload[2] | (pending_payload[3] << 8));
        cmd.brake      = pending_payload[4];
        cmd.flags      = pending_payload[5];
        cmd.valid      = true;
        last_cmd_ms    = now_ms;
    }
}

/* ------------------------------------------------- state machine */
static void update_state_machine(void)
{
    if (!hw_sdc_closed()) { fault_flags |= FAULT_SDC_OPEN; goto emergency; }
    if (hw_lv_centivolt() < 1900) { fault_flags |= FAULT_LV_UNDERVOLT; goto emergency; }

    switch (as_state) {
    case AS_OFF:
        if (hw_asms_on() && hw_mission_selected())
            as_state = AS_READY;
        break;
    case AS_READY:
        if (hw_res_go())
            as_state = AS_DRIVING;
        break;
    case AS_DRIVING:
        if (cmd.valid && (cmd.flags & 0x01)) goto emergency;      /* e-stop  */
        if (now_ms - last_cmd_ms > CMD_TIMEOUT_MS + EBS_GRACE_MS) {
            fault_flags |= FAULT_CMD_TIMEOUT;
            goto emergency;
        }
        if (hw_mission_complete() && hw_standstill())
            as_state = AS_FINISHED;
        break;
    case AS_FINISHED:
    case AS_EMERGENCY:
        break;                        /* latched; manual reset only */
    }
    return;

emergency:
    as_state = AS_EMERGENCY;
    hw_ebs_engage();
    hw_bamocar_torque(0);
}

/* ------------------------------------------------- 100 Hz control */
static void control_10ms(void)
{
    update_state_machine();

    bool cmd_fresh = (now_ms - last_cmd_ms) <= CMD_TIMEOUT_MS;

    if (as_state == AS_DRIVING && cmd_fresh && cmd.valid) {
        /* clamp everything locally — invariant 3 */
        int16_t t = cmd.torque_dnm;
        if (t >  TORQUE_LIMIT_DNM) t =  TORQUE_LIMIT_DNM;
        if (t < 0)                 t = 0;
        int16_t s = cmd.steer_mrad;
        if (s >  STEER_LIMIT_MRAD) s =  STEER_LIMIT_MRAD;
        if (s < -STEER_LIMIT_MRAD) s = -STEER_LIMIT_MRAD;
        /* slew limit — invariant: no step steering */
        int16_t ds = (int16_t)(s - steer_target_mrad);
        if (ds >  STEER_SLEW_MRAD_10MS) ds =  STEER_SLEW_MRAD_10MS;
        if (ds < -STEER_SLEW_MRAD_10MS) ds = -STEER_SLEW_MRAD_10MS;
        steer_target_mrad = (int16_t)(steer_target_mrad + ds);

        hw_bamocar_torque(t);
        hw_steer_setpoint(steer_target_mrad);
        /* brake: proportional hydraulic/pneumatic demand */
        /* TODO(board): map cmd.brake 0-200 onto brake actuator */
    } else if (as_state == AS_DRIVING && !cmd_fresh) {
        /* first 100 ms of staleness: safe hold before EBS grace expires */
        hw_bamocar_torque(0);
        hw_steer_setpoint(steer_target_mrad);   /* hold, don't center */
    } else {
        hw_bamocar_torque(0);                   /* invariant 2 */
    }

    hw_assi(as_state);

    /* TX 0x200 feedback */
    uint8_t tx[7];
    tx[0] = (uint8_t)(steer_actual_mrad & 0xFF);
    tx[1] = (uint8_t)((steer_actual_mrad >> 8) & 0xFF);
    tx[2] = 0; tx[3] = 0;                       /* TODO(board): rpm */
    tx[4] = (uint8_t)as_state;
    tx[5] = (uint8_t)(fault_flags & 0xFF);
    tx[6] = (uint8_t)((fault_flags >> 8) & 0xFF);
    can_send(0x200, tx, 7);
    /* TODO(board): TX 0x201 wheel speeds from encoder capture,
                    TX 0x210 pressures/temps at 10 Hz */
}

int main(void)
{
    hw_init();
    /* TODO(board): SysTick 1 kHz -> now_ms++; scheduler calling
       control_10ms() every 10 ms; 1 kHz steering PID reading the BLDC
       encoder into steer_actual_mrad and driving toward steer_target_mrad. */
    for (;;) { /* main loop or RTOS tasks */ }
}
