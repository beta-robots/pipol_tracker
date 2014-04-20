#target id
int32 targetId

#target status is a bitwise OR of the following values
#      TO_BE_REMOVED = 0x01,
#      OCCLUDDED = 0x02,
#      CANDIDATE = 0x04,
#      LEGGED_TARGET = 0x08,
#      VISUALLY_CONFIRMED = 0x10,
#      FRIEND_IN_SIGHT = 0x20,
#      BACK_LEARNT = 0x40,
#      FACE_LEARNT = 0x80
int32 targetStatus

#target 2D position
float64 x
float64 y

#target 2D linear velocity
float64 vx
float64 vy

#(x,y,vx,vy) covariance matrix
float64[16] covariances
