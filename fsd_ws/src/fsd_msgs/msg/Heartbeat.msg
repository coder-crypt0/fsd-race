# Published by every node on /safety/heartbeat at >= 5 Hz.
uint8 STATUS_OK=0
uint8 STATUS_DEGRADED=1
uint8 STATUS_ERROR=2

std_msgs/Header header
string node_id
uint8  status
string message          # human-readable, empty when OK
