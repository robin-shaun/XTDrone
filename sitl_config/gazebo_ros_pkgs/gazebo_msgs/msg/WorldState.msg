# This is a message that holds data necessary to reconstruct a snapshot of the world
#
# = Approach to Message Passing =
# The state of the world is defined by either
#   1. Inertial Model pose, twist
#      * kinematic data - connectivity graph from Model to each Link
#      * joint angles
#      * joint velocities
#      * Applied forces - Body wrench
#        * relative transform from Body to each collision Geom
# Or
#   2. Inertial (absolute) Body pose, twist, wrench
#      * relative transform from Body to each collision Geom - constant, so not sent over wire
#      * back compute from canonical body info to get Model pose and twist.
#
# Chooing (2.) because it matches most physics engines out there
#   and is simpler.
#
# = Future =
# Consider impacts on using reduced coordinates / graph (parent/child links) approach
#   constraint and physics solvers.
#
# = Application =
# This message is used to do the following:
#   * reconstruct the world and objects for sensor generation
#   * stop / start simulation - need pose, twist, wrench of each body
#   * collision detection - need pose of each collision geometry.  velocity/acceleration if
#
# = Assumptions =
# Assuming that each (physics) processor node locally already has
#   * collision information - Trimesh for Geoms, etc
#   * relative transforms from Body to Geom - this is assumed to be fixed, do not send oved wire
#   * inertial information - does not vary in time
#   * visual information - does not vary in time
#

Header header

string[] name
geometry_msgs/Pose[] pose
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench
