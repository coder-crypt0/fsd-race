# One confirmed landmark in the persistent track map (Block 4 output element)
uint8 SIDE_UNKNOWN=0
uint8 SIDE_LEFT=1
uint8 SIDE_RIGHT=2

uint32  id                 # persistent global id, never reused
uint8   color              # majority vote result, ConeDetection2D enum
float32 x                  # meters, odom frame
float32 y                  # meters, odom frame
uint8   side
uint16  observation_count
