# Target id
int32 targetId

# Target status is a bitwise OR of the following values
int32 TO_BE_REMOVED         = 1     # 0x01
int32 OCCLUDDED             = 2     # 0x02
int32 CANDIDATE             = 4     # 0x04
int32 LEGGED_TARGET         = 8     # 0x08
int32 VISUALLY_CONFIRMED    = 16    # 0x10
int32 FRIEND_IN_SIGHT       = 32    # 0x20
int32 BACK_LEARNT           = 64    # 0x40
int32 FACE_LEARNT           = 128   # 0x80

int32 targetStatus

# Target 2D position
float64 x
float64 y

# Target 2D linear velocity
float64 vx
float64 vy

# (x,y,vx,vy) covariance matrix
float64[16] covariances
