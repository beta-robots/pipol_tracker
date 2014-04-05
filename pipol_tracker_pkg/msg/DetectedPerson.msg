## Contains data relative to the detection of a person

Header header

# ID associated to a detected person by the personServer
int64 objectId

# colour associated to the detected person by the personServer
# values from the personServer are in [0-255]
std_msgs/ColorRGBA boxColour

# detection confidence
float64 confidence

# 3D position of the person projected on the ground plane, expressed in the world frame
geometry_msgs/Point  position3D

# rectangular zone of the image that contains the person detected
pal_vision_msgs/Rectangle box

# rectangular zone of the image that may contain the face of the person detected
# if no face has been detected, faceBox = Rectangle(0, 0, 0, 0)
pal_vision_msgs/Rectangle faceBox

