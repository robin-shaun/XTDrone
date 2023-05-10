# a header, used for interpreting the poses
Header header

# the name of the object. This should usually
# coincide with the /tf frame of the base/origin
# of the object, though this is no mandatory
# requirement.
string name

# If known: The object type in a database of known objects
object_recognition_msgs/ObjectType type

# If the model is made up of primitives:
# Solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# If the model is made up of meshes:
# The model meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# If known: bounding plane
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses

# Index into the primitive_poses to the
# pose which is considered to be the
# object's root / origin. Can also be
# set to constants ORIGIN_AVERAGE,
# ORIGIN_UNDEFINED or ORIGIN_CUSTOM.
int32 primitive_origin

# Index into the mesh_poses to the
# pose which is considered to be the
# object's root / origin. Can also be
# set to constants ORIGIN_AVERAGE,
# ORIGIN_UNDEFINED or ORIGIN_CUSTOM.
int32 mesh_origin

# If 'mesh_origin' or 'primitive_origin'
# are set to ORIGIN_CUSTOM, this field
# contains the origin of the overall object.
geometry_msgs/Pose origin

# Information in the message, either SHAPE or POSE
byte content

# For field 'content':
# Message contains the shape(s) of the object including poses
byte SHAPE=0

# For field 'content':
# Message contains only the pose of the primitives,
# i.e. only *_poses will be with values.
byte POSE=1

# For fields 'mesh_origin' and 'primitives_origin':
# the root/origin of the object is the
# average of all poses.
# Because the average of several quaternions
# can only be determined approximately when
# the quaternions are close together, this
# setting requires at least the orientation part
# of the field 'origin' to be set in addition.
int32 ORIGIN_AVERAGE=-1

# For fields 'mesh_origin' and 'primitives_origin':
# The origin of the object cannot be defined by this
# set of poses.
int32 ORIGIN_UNDEFINED=-2

# For fields 'mesh_origin' and 'primitives_origin':
# The origin is given in the custom field 'origin' 
int32 ORIGIN_CUSTOM=-3
