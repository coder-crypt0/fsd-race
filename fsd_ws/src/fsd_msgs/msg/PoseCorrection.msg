# Map-relative pose correction (Block 4 -> Block 3, /localization/correction).
# The rigid 2D offset that best aligns the current cone observations with the
# confirmed map. Block 3 applies it SLEW-LIMITED on top of dead reckoning —
# never as a jump, so the supervisor's pose-continuity check stays meaningful.
std_msgs/Header header
float32 dx                 # meters, odom frame
float32 dy                 # meters, odom frame
float32 dyaw               # rad
uint16  n_inliers          # matched landmark pairs behind this solve
float32 rms_m              # residual after the fit
bool    valid              # false = solve rejected, ignore dx/dy/dyaw
