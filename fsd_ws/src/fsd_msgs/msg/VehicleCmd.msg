# Block 6 output → Block 7 input. Published at a fixed 50 Hz, never event-driven.
std_msgs/Header header
float32 steering_angle     # rad at the TIRE, +left / -right
float32 torque_request     # Nm at motor shaft, forwarded to Bamocar
float32 brake_cmd          # 0.0 - 1.0 normalized brake demand
bool    emergency_stop     # true = force immediate EBS, overrides all above
