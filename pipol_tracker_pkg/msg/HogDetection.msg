## Contains data relative to the detection of a person using the HOG descriptor

pal_vision_msgs/Rectangle  imageBoundingBox         # bounding box of image region in which the person has been detected
geometry_msgs/Vector3      direction                # unitary vector expressing in what direction wrt the robot base frame the person is
float32[]                  hog                      # HOG descriptor of the person region
std_msgs/ColorRGBA         principalEigenVectorRGB  # Eigen vector (remember this is an unitary vector) corresponding to the maximum eigen value of all the RGB values in the person region. 
std_msgs/ColorRGBA[]       rgbClusterCenters        # RGB cluster centers of the person region in the image obtained using k-means. The rgb components are expressed in [0..1]
uint32[]                   rgbDescriptor1           # Descriptor based on binarized RGB gradients between adjacent image blocks (version 1)
uint32[]                   rgbDescriptor2           # Descriptor based on binarized RGB gradients between adjacent image blocks (version 2)



