- Include the local implementation of joint controller and joint trajectory execution (gazbo implementation) in these packages.

# Possible future contributions

gazebo:
    - the higher-level controller with position and velocity
    - a program which helps find PID values, either with neural network or Ziegler-Nichols method

# Objects

- move away from object_msgs and replace with object_recognition_msgs/RecognizedObject.msg. The bounding_mesh
can be used to replace primitives, which then have to be supported in object_moveit/ObjectMessageGenerator.

