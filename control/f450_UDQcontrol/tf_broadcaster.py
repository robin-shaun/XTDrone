import rospy
import sys
import tf2_ros as tf2
import tf_conversions as tfc
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import math
import numpy
from nav_msgs.msg import Odometry

def current_state_callback(data):
    global cur_state
    cur_state = data
    # print('upward vel %.2f', cur_state.pose.pose.orientation.x)

def qRot(p, q):
    qInvert = tfc.transformations.quaternion_inverse
    qMultiply = tfc.transformations.quaternion_multiply
    qInv = qInvert(q)
    pNew = qMultiply(qMultiply(q, p), qInv)
    return pNew

def qRotInv(p, q):
    qInvert = tfc.transformations.quaternion_inverse
    qMultiply = tfc.transformations.quaternion_multiply
    qInv = qInvert(q)
    pNew = qMultiply(qMultiply(qInv, p), q)
    return pNew


def __genTf(frameS, frameT, posTs, q):
    ts = TransformStamped()
    ts.header.stamp = rospy.Time.now()
    ts.header.frame_id = frameS
    ts.child_frame_id = frameT
    
    ts.transform.translation.x = posTs[0]
    ts.transform.translation.y = posTs[1]
    ts.transform.translation.z = posTs[2]

    ts.transform.rotation.x = q[0]
    ts.transform.rotation.y = q[1]
    ts.transform.rotation.z = q[2]
    ts.transform.rotation.w = q[3]
    return ts

def __main():
    rospy.init_node("tf_broadcaster")
    # rate = rospy.Rate(1)
    # tbf = tf2.Buffer()
    # tl = tf2.TransformListener(tbf)

    a = 0

    rospy.Subscriber("/vrpn_client_node/parallel_f450_3/pose", PoseStamped, current_state_callback)
    target_groundtruth_pub = rospy.Publisher(
        "/xtdrone/iris_0/ground_truth/target", Odometry, queue_size=10)
    global cur_state
    cur_state = PoseStamped()
    roundCount = 0
    rateFast = rospy.Rate(10)

    qInvert = tfc.transformations.quaternion_inverse
    qMultiply = tfc.transformations.quaternion_multiply
    while not rospy.is_shutdown():
        print("begin: " + str(roundCount))
        roundCount += 1
        # # lookup_transform(desFrame, srcFrame, time)
        # tfD2C = tbf.lookup_transform("C", "D", rospy.Time())
        # tfC2D = tbf.lookup_transform("D", "C", rospy.Time())
        # tfB2A = tbf.lookup_transform("A", "B", rospy.Time())
        # tfA2B = tbf.lookup_transform("B", "A", rospy.Time())

        bc = tf2.TransformBroadcaster()
        # # t20
        #####set-point
        # p02t = numpy.array([1, 2, 2, 0])
        # q02t = tfc.transformations.quaternion_from_euler(0, 0, math.pi/2)
        #####tracking
        # p02t = numpy.array([2 * math.cos(a), -1 + 2 * math.sin(a), 1, 0])
        # q02t = numpy.array(tfc.transformations.quaternion_from_euler(0, 0, math.pi / 2 + a)) #math.pi / 2 +
        p02t = numpy.array([2 * math.cos(a), -1 + 2 * math.sin(a), 2+math.sin(5*a), 0])
        q02t = numpy.array(tfc.transformations.quaternion_from_euler(0, 0, math.pi / 2 + a)) #math.pi / 2 +
        ts = __genTf("state_0", "target_state", p02t, q02t)
        # qt20 = qInvert(q02t)
        # pt20 = qRotInv(-p02t, q02t)
        # ts = __genTf("target_state", "state_0", pt20, qt20)
        bc.sendTransform(ts)

        if cur_state.pose.orientation.x != 0.0:
            # # 02c
            p02c = numpy.array([cur_state.pose.position.x, cur_state.pose.position.y, cur_state.pose.position.z, 0])
            q02c = (cur_state.pose.orientation.x, cur_state.pose.orientation.y, cur_state.pose.orientation.z, cur_state.pose.orientation.w)
            # print("currently:\t x %.2f\t y %.2f\t z %.2f" % (
            #     cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z))
            # ts = __genTf("current_state", "state_0", p02c, q02c)
            # bc.sendTransform(ts)
            # # c20
            pc20 = qRotInv(-p02c, q02c)
            qc20 = qInvert(q02c)
            ts = __genTf("current_state", "state_0", pc20, qc20)
            bc.sendTransform(ts)
            # # # c2t
            qc2t = qMultiply(qInvert(q02c), q02t)
            print("qc2t %.2f\t %.2f\t %.2f\t %.2f", qc2t[0], qc2t[1],qc2t[2],qc2t[3])
            pc2t = qRotInv((p02t - p02c), q02c)
            ts = __genTf("current_state", "target_state1", pc2t, qc2t)
            bc.sendTransform(ts)

        #####set-point
        # qdot02t = numpy.array([0, 0, 0, 0])
        # pdot02t = numpy.array([0, 0, 0, 0])
        #####tracking
        # qdot02t = numpy.array([0, 0, 0.05*numpy.cos(a/2), -0.05*numpy.sin(a/2)])
        # pdot02t = numpy.array([-0.2*numpy.sin(a), 0.2*numpy.cos(a), 0, 0])
        qdot02t = numpy.array([0, 0, 0.05*numpy.cos(a/2), -0.05*numpy.sin(a/2)])
        pdot02t = numpy.array([-0.2*numpy.sin(a), 0.2*numpy.cos(a), 0.25*numpy.cos(5*a), 0])

        omega02t = 2 * qMultiply(qdot02t, qInvert(q02t))

        tar = Odometry()
        tar.header.stamp = rospy.Time.now()
        tar.header.frame_id = "state_0"
        tar.pose.pose.position.x = p02t[0]
        tar.pose.pose.position.y = p02t[1]
        tar.pose.pose.position.z = p02t[2]
        tar.pose.pose.orientation.x = q02t[0]
        tar.pose.pose.orientation.y = q02t[1]
        tar.pose.pose.orientation.z = q02t[2]
        tar.pose.pose.orientation.w = q02t[3]
        tar.twist.twist.linear.x = pdot02t[0]
        tar.twist.twist.linear.y = pdot02t[1]
        tar.twist.twist.linear.z = pdot02t[2]
        tar.twist.twist.angular.x = omega02t[0]
        tar.twist.twist.angular.y = omega02t[1]
        tar.twist.twist.angular.z = omega02t[2]
        target_groundtruth_pub.publish(tar)


        if a < math.pi * 2:
            a += 0.01
        else:
            a = 0

        rateFast.sleep()


if __name__ == "__main__":
    __main() 
