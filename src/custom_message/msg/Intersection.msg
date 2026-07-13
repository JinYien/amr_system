uint8 UNKNOWN = 0
uint8 DEADEND = 1
uint8 CORRIDOR = 2
uint8 LEFT_CORNER = 3
uint8 RIGHT_CORNER = 4
uint8 LEFT_JUNCTION = 5
uint8 RIGHT_JUNCTION = 6
uint8 T_JUNCTION = 7
uint8 CROSS = 8
uint8 OPEN_SPACE = 9

std_msgs/Header header
uint8 junction
bool front_open
bool left_open
bool right_open
