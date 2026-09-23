# Opt-in diagnostics only; never consumed by autonomy.
std_msgs/Header header       # report time; frame_id is empty
string component             # node or callback label
builtin_interfaces/Time input_stamp
bool has_input_stamp
float64 input_age_ms          # ROS time at callback entry minus input stamp
float64 callback_ms           # steady-clock wall duration, excludes diagnostic publication
uint64 callback_index         # includes callbacks skipped by the sampling limiter
