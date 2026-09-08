# Planner-imposed speed ceiling (Block 5 -> Block 6, /planning/speed_limit).
# Block 5 is the only node that knows whether the path it just published is a
# sensor-horizon-limited exploration path, a fully known racing line, or a
# corridor with something in the way — so it owns the speed regime. Block 6
# still applies its own absolute v_max_mps ceiling and dynamics limits on top.
uint8 REASON_EXPLORE=0     # discovering the track: bounded by camera range
uint8 REASON_RACE=1        # closed map + valid global racing line
uint8 REASON_OBSTACLE=2    # avoiding something inside the corridor
uint8 REASON_BLOCKED=3     # no feasible gap — v_max_mps is 0, stop

std_msgs/Header header
float32 v_max_mps          # hard cap for the controller; 0 = come to a stop
uint8   reason
string  detail             # human-readable, for the log and the dashboard
