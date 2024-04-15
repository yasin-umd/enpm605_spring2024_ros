# RobotStatus.msg

# Constants for the robot model
uint8 BURGER=0
uint8 WAFFLE=1
uint8 WAFFLE_PI=2

uint8 model # The model of the robot (BURGER, WAFFLE, WAFFLE_PI)
float64 distance # The distance the robot has traveled in meters