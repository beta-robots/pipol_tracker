#target id
int32 targetId

#target status is a bitwise OR of the following values
int32 TO_BE_REMOVED = 1
int32 OCCLUDDED = 2
int32 CANDIDATE = 4
int32 LEGGED_TARGET = 8
int32 VISUALLY_CONFIRMED = 10
int32 FRIEND_IN_SIGHT = 20
int32 BACK_LEARNT = 40
int32 FACE_LEARNT = 80

int32 targetStatus

#target 2D position
float64 x
float64 y

#target 2D linear velocity
float64 vx
float64 vy

#(x,y,vx,vy) covariance matrix
float64[16] covariances
