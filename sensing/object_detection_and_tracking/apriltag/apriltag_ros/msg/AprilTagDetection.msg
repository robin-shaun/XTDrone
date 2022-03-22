# Tag ID(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the IDs of each tag in the bundle.
int32[] id

# Tag size(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the sizes of each tag in the bundle, in the same
# order as the IDs above.
float64[] size

# Pose in the camera frame, obtained from homography transform. If a standalone
# tag, the homography is from the four tag corners. If a tag bundle, the
# homography is from at least the four corners of one member tag and at most the
# four corners of all member tags.
geometry_msgs/PoseWithCovarianceStamped pose