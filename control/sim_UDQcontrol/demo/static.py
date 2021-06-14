import rospy
import sys
import tf2_ros as tf2
import tf_conversions as tfc
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage
import math
import numpy

def __genTf(frameT, frameS, posTs, q):
    ts = geometry_msgs.msg.TransformStamped()
    ts.header.stamp = rospy.Time.now()
    ts.header.frame_id = frameT
    ts.child_frame_id = frameS
    
    ts.transform.translation.x = posTs[0]
    ts.transform.translation.y = posTs[1]
    ts.transform.translation.z = posTs[2]

    ts.transform.rotation.x = q[0]
    ts.transform.rotation.y = q[1]
    ts.transform.rotation.z = q[2]
    ts.transform.rotation.w = q[3]
    return ts

def __main():
    rospy.init_node("AB2CD")
    rate = rospy.Rate(100)
    tbf = tf2.Buffer()
    
    t = TFMessage()
    sbc = tf2.StaticTransformBroadcaster()
    qA2B = tfc.transformations.quaternion_from_euler(math.pi, 0, math.pi / 2)
    ts1 = __genTf("A", "B", (-1, 0, 0.2), qA2B)
    t.transforms.append(ts1)

    qC2D = tfc.transformations.quaternion_from_euler(math.pi, 0, 0)
    ts2 = __genTf("C", "D", (0, 0, 1), qC2D)
    t.transforms.append(ts2)

    sbc.pub_tf.publish(t)
    rospy.spin()


if __name__ == "__main__":
    __main()
