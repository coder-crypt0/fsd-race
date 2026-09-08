# 3D cone in base_link frame (Block 2 output element)
uint32  id             # per-frame temporary id, NOT persistent
uint8   color          # enum values shared with ConeDetection2D
float32 confidence
float32 x              # meters, base_link, forward
float32 y              # meters, base_link, left
float32 z              # meters, base_link, up (~0 for ground cones)
float32 depth_sigma    # 1-sigma range uncertainty, meters
