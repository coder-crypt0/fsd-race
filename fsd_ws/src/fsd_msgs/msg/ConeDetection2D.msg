# 2D cone detection in image space (Block 1 output element)
uint8 COLOR_BLUE=0
uint8 COLOR_YELLOW=1
uint8 COLOR_ORANGE_SMALL=2
uint8 COLOR_ORANGE_BIG=3
uint8 COLOR_UNKNOWN=255

uint8 SOURCE_YOLO=0
uint8 SOURCE_HSV=1

uint8   color
float32 confidence     # 0.0 - 1.0
float32 cx             # bbox center x, pixels
float32 cy             # bbox center y, pixels
float32 width          # bbox width, pixels
float32 height         # bbox height, pixels
uint8   source
