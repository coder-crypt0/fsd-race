# STM32 -> Jetson feedback (Block 7 output, relayed by the CAN bridge)
uint8 AS_OFF=0
uint8 AS_READY=1
uint8 AS_DRIVING=2
uint8 AS_FINISHED=3
uint8 AS_EMERGENCY=4

std_msgs/Header header
float32 actual_steering_angle   # rad at tire, from BLDC encoder
float32 motor_rpm               # from Bamocar
float32 motor_temp_c
float32 inverter_temp_c
float32 ebs_pressure_bar        # pneumatic line pressure
uint8   as_state
uint16  fault_flags             # bitfield, see firmware/stm32_bridge
