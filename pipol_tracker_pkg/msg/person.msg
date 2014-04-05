#target id
int32 targetId

#target status 
#       TO_BE_REMOVED = 0
#       CANDIDATE = 1
#       TARGET = 2
#       VISUALLY_CONFIRMED = 3
#       FRIEND_IN_SIGHT = 4
#       FRIEND_OUT_OF_RANGE = 5
int32 status

#person 2D position
float64 x
float64 y

#person 2D velocity
float64 vx
float64 vy

#covariance matrix fo the four above defined parameters (x,y,vx,vy)
float64[16] covariances

#appearance model as three colored regions up, middle and bottom
coloredBox[3] appearance