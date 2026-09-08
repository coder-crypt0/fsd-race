# Track knowledge state (Block 4 output, /mapping/status, 10 Hz).
# Tells the rest of the stack whether the track is still being discovered
# or is fully mapped — the gate between exploration speed and race speed.
uint8 MODE_EXPLORE=0       # first lap: speed bounded by the sensor horizon
uint8 MODE_KNOWN=1         # map closed: the whole lap geometry is known

std_msgs/Header header
uint8   mode
uint16  laps_completed     # start-line crossings since AS_DRIVING
bool    loop_closed        # start line crossed at least once
float32 lap_distance_m     # distance travelled since the last crossing
float32 track_length_m     # measured lap length, 0 until loop_closed
uint16  confirmed_cones
uint16  localized_cones    # landmarks matched by the last localization solve
float32 localization_rms_m # map-relative residual, 0 = not localizing
