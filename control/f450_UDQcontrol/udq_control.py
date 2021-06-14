import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
import sys, select, os
import tty, termios
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pyquaternion import Quaternion
from nav_msgs.msg import Odometry
import sys
import tf2_ros as tf2
import tf_conversions as tfc
import math
import numpy as np
from tf2_msgs.msg import TFMessage

class Controller:
    def __init__(self, vehicle_type, vehicle_id):
        self.MAX_LINEAR = 0.6
        self.MAX_ANG_VEL = 0.25  #0.15
        self.LINEAR_STEP_SIZE = 0.01
        self.ANG_VEL_STEP_SIZE = 0.01

        self.cmd_vel_mask = False
        self.ctrl_leader = False

        self.msg2all = """
        Control Your XTDrone!
        To all drones  (press g to control the leader)
        ---------------------------
           1   2   3   4   5   6   7   8   9   0
                w       r    t   y        i
           a    s    d       g       j    k    l
                x       v    b   n        ,

        w/x : increase/decrease forward velocity
        a/d : increase/decrease leftward velocity
        i/, : increase/decrease upward velocity
        j/l : increase/decrease orientation
        r   : return home
        t/y : arm/disarm
        v/n : takeoff/land
        b   : offboard
        s/k : hover and remove the mask of keyboard control
        0~9 : extendable mission(eg.different formation configuration)
              this will mask the keyboard control
        o   : init---offboard, arm, takeoff and hover
        p   : pose control
        CTRL-C to quit
        """

        self.e = """
        Communications Failed
        """

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def print_msg(self):
        print(msg2all)

    def qprod(self, q1, q2):
        return tfc.transformations.quaternion_multiply(q1, q2)

    # def qqprod(self, q1, q2):
    #     return np.hstack((q1[3]*q2[:3]+q2[3]*q1[:3]+np.cross(q1[:3], q2[:3]), q1[3]*q2[3] - np.dot(q1[:3], q2[:3])))

    def qconj(self, q):
        return np.hstack((-q[:3], q[3]))

    def qAd(self, q, p):
        return qprod(qprod(q, p), qconj(q))

    def qcrossProd(self, p1, p2):
        return np.hstack((np.cross(p1[:3], p2[:3]), [0]))

    def qdotProd(self, p1, p2):
        return np.array([p1[0] * p2[0], p1[1] * p2[1], p1[2] * p2[2], 0])

    def dqdotProd(self, xi1, xi2):
        return np.hstack((qdotProd(xi1[:4], xi2[:4]), qdotProd(xi1[4:8], xi2[4:8])))

    def dqprod(self, dq1, dq2):
        return np.hstack((qprod(dq1[:4], dq2[:4]), qprod(dq1[4:8], dq2[:4]) + qprod(dq1[:4], dq2[4:8])))

    def dqconj(self, dq):
        return np.hstack((qconj(dq[:4]), qconj(dq[4:8])))

    def dqAd(self, dq, xi):
        return dqprod(dqprod(dq, xi), dqconj(dq))

    def dqconstr(self, q, p_s):
        return np.hstack((q, qprod(p_s, q) / 2))

    def lambdadq(self, dq):
        if dq[3] >= 0:
            return dq
        else:
            return -dq

    def q2nvartheta(self, q):
        if q[3] > 1:
            q[3] = 1
        elif q[3] < -1:
            q[3] = -1
        vartheta = 2 * np.arccos(q[3])
        sin_varthataD2 = np.sin(vartheta / 2)
        if sin_varthataD2 == 0:
            sin_varthataD2 = sin_varthataD2 + 0.0000001
        n = np.array([q[0] / sin_varthataD2, q[1] / sin_varthataD2, q[2] / sin_varthataD2])
        return np.hstack((n, np.array(vartheta)))

    def dq2nvarthetaps(self, dq):
        p_s = qprod(2 * dq[4:8], qconj(dq[:4]))
        return np.hstack((q2nvartheta(dq[:4]), p_s))

    def dqln(dq):
        nvarthetaps = dq2nvarthetaps(self, dq)
        return np.hstack((nvarthetaps[:3] * nvarthetaps[3] * 0.5, [0], nvarthetaps[4:8] * 0.5))


def current_state_callback(data):
    global cur_state
    cur_state = data

def target_state_callback(data):
    global tar_state
    tar_state = data



if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    multirotor_type = 'iris' #sys.argv[1]
    multirotor_num = 1 #int(sys.argv[2])
    control_type = 'vel' #sys.argv[3]
    
    # if multirotor_num == 18:
    #     formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']
    # elif multirotor_num == 9:
    #     formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
    # elif multirotor_num == 6:
    #     formation_configs = ['waiting', 'T', 'diamond', 'triangle']
    
    cmd= String()
    twist = Twist()   
    
    rospy.init_node('multirotor_keyboard_UDQ_control')
    
    if control_type == 'vel':
        multi_cmd_vel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_vel_flu', Twist, queue_size=10)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=10)
        leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=10)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=10)
 
    else:
        multi_cmd_accel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_accel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_accel_flu', Twist, queue_size=10)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=10)
        leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=10)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=10)

    rospy.Subscriber("/xtdrone/iris_0/ground_truth/odom", Odometry, current_state_callback)
    rospy.Subscriber("/xtdrone/iris_0/ground_truth/target", Odometry, current_state_callback)
    global tar_state
    tar_state = Odometry()
    forward  = 0.0
    leftward  = 0.0
    upward  = 0.0
    angular = 0.0
    count = 0
    print_msg()
    while not rospy.is_shutdown(): #(1):
        key = getKey()
        if key == 'w' :
            forward = forward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'x' :
            forward = forward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'a' :
            leftward = leftward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'd' :
            leftward = leftward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'i' :
            upward = upward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == ',' :
            upward = upward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'j':
            angular = angular + ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'l':
            angular = angular - ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'r':
            cmd = 'AUTO.RTL'
            print_msg()
            print('Returning home')
        elif key == 't':
            cmd = 'ARM'
            print_msg()
            print('Arming')
        elif key == 'y':
            cmd = 'DISARM'
            print_msg()
            print('Disarming')
        elif key == 'v':
            cmd = 'AUTO.TAKEOFF'
            cmd = ''
            print_msg()
            #print('Takeoff mode is disenabled now')
        elif key == 'b':
            cmd = 'OFFBOARD'
            print_msg()
            print('Offboard')
        elif key == 'n':
            cmd = 'AUTO.LAND'
            print_msg()
            print('Landing')
        elif key == 'g':
            ctrl_leader = not ctrl_leader
            print_msg()
        elif key in ['k', 's']:
            cmd_vel_mask = False
            forward   = 0.0
            leftward   = 0.0
            upward   = 0.0
            angular  = 0.0
            cmd = 'HOVER'
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            print('Hover')
        elif key == 'o':
            #offboard
            cmd = 'OFFBOARD'
            print('Offboard')
            # while count < 5:
            #     multi_cmd_pub[0].publish(cmd)
            #     count = count + 1
            #     rospy.Rate(5).sleep()
            multi_cmd_pub[0].publish(cmd)
            rospy.sleep(1.0)
            count = 0
            #arm
            cmd = 'ARM'
            print('Arming')
            # while count < 5:
            #     multi_cmd_pub[0].publish(cmd)
            #     count = count + 1
            #     rospy.Rate(5).sleep()
            multi_cmd_pub[0].publish(cmd)
            rospy.sleep(1.0)
            count = 0
            #takeoff
            cmd = ''
            upward = 0.4
            twist.linear.x = forward
            twist.linear.y = leftward
            twist.linear.z = upward
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            print('takingoff')
            multi_cmd_vel_flu_pub[0].publish(twist)
            while cur_state.pose.pose.position.z < 2.0 :
                rospy.Rate(5).sleep()
            # while cur_pose.pose.pose.position.z < 1.5 :
            #     multi_cmd_vel_flu_pub[0].publish(twist)
            #     multi_cmd_pub[0].publish(cmd)
            #     rospy.Rate(5).sleep()
            #hover
            cmd_vel_mask = False
            forward   = 0.0
            leftward   = 0.0
            upward   = 0.0
            angular  = 0.0
            cmd = 'HOVER'
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            print('Hover')
            twist.linear.x = forward
            twist.linear.y = leftward
            twist.linear.z = upward
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular
            multi_cmd_vel_flu_pub[0].publish(twist)
            multi_cmd_pub[0].publish(cmd)
        elif key == 'p':
            #  set
            dk = np.array([1,1,2,0,2,1,1,0])
            tbf = tf2.Buffer()
            listener = tf2.TransformListener(tbf)
            pdot_bsat_last = np.array([0,0,0,0])
            omega_bsat_last = np.array([0,0,0,0])
            while getKey() != 's' :
                #####listen to target pose
                tf02t = geometry_msgs.msg.TransformStamped()
                try:
                    tf02t = tbf.lookup_transform("state_0", "target_state", rospy.Time())
                except(tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
                    rospy.Rate(10).sleep()
                rot02t = tf02t.transform.rotation
                tsl02t = tf02t.transform.translation
                #####tar_pose
                q_d = np.array([rot02t.x, rot02t.y, rot02t.z, rot02t.w])
                # print("destiny:\t q_d x %.2f\t q_d y %.2f\t q_d z %.2f\t q_d w %.2f" % (
                #     q_d[0], q_d[1], q_d[2], q_d[3]))
                p_sd = np.array([tsl02t.x, tsl02t.y, tsl02t.z, 0])
                # print("destiny:\t p_sd x %.2f\t p_sd y %.2f\t p_sd z %.2f" % (
                #     p_sd[0], p_sd[1], p_sd[2]))
                #####tar_state
                lin02t = tar_state.twist.twist.linear
                ang02t = tar_state.twist.twist.angular
                omega_sd = np.array([ang02t.x, ang02t.y, ang02t.z, 0])
                pdot_sd = np.array([lin02t.x, lin02t.y, lin02t.z, 0])
                # omega_sd = np.array([0, 0, 0, 0])
                # pdot_sd = np.array([0, 0, 0, 0])
                xi_sd = np.hstack((omega_sd, pdot_sd + qcrossProd(p_sd, omega_sd)))
                #####callback from current state
                tsl02c = cur_state.pose.pose.position
                rot02c = cur_state.pose.pose.orientation
                # lin02c = cur_state.twist.twist.linear
                # ang02c = cur_state.twist.twist.angular
                ######cur_pose and cur_twist
                q = np.array([rot02c.x, rot02c.y, rot02c.z, rot02c.w])
                # print("currently:\t q x %.2f\t q y %.2f\t q z %.2f\t q w %.2f" % (
                #     q[0], q[1], q[2], q[3]))
                p_s = np.array([tsl02c.x, tsl02c.y, tsl02c.z, 0])
                # print("currently:\t p_s x %.2f\t p_s y %.2f\t p_s z %.2f" % (
                #     p_s[0], p_s[1], p_s[2]))
                # omega_sC = np.array([ang02c.x, ang02c.y, ang02c.z, 0])
                # pdot_sC = np.array([lin02c.x, lin02c.y, lin02c.z, 0])
                #####control scheme
                q_er = qprod(q_d, qconj(q))
                # print("error:\t q_er x %.2f\t q_er y %.2f\t q_er z %.2f\t q_er w %.2f" % (
                #     q_er[0], q_er[1], q_er[2], q_er[3]))
                # omega_se = omega_sd - qAd(q_er, omega_s)
                p_se = p_sd - qAd(q_er, p_s)
                # print("error:\t p_se x %.2f\t p_se y %.2f\t p_se z %.2f" % (
                #     p_se[0], p_se[1], p_se[2]))
                # pdot_se = pdot_sd - qcrossProd(omega_se, qAd(q_er, p_s)) - qAd(q_er, pdot_s)
                dq_er = dqconstr(q_er, p_se)
                xi_s = dqAd(dqconj(dq_er), xi_sd + 2 * dqdotProd(dk, dqln(lambdadq(dq_er))))
                omega_s = xi_s[:4]
                pdot_s = xi_s[4:8] + qcrossProd(p_s, omega_s)
                # print(
                # "currently:\t linear vel x %.2f\t linear vel y %.2f\t linear vel z %.2f\t angular vel x %.2f\t angular vel y %.2f\t angular vel z %.2f" % (
                #     pdot_s[0], pdot_s[1], pdot_s[2], omega_s[0], omega_s[1], omega_s[2]))
                #####saturate
                ang = np.sqrt(np.dot(omega_s, omega_s.T))
                lin = np.sqrt(np.dot(pdot_s, pdot_s.T))
                # print("linear vel: %.2f\t angular vel: %.2f" %(lin, ang))
                if ang > MAX_ANG_VEL:
                    omega_ssat = omega_s/ang*MAX_ANG_VEL
                else:
                    omega_ssat = omega_s
                # omega_ssat = omega_s
                if lin > MAX_LINEAR:
                    pdot_ssat = pdot_s/lin*MAX_LINEAR
                else:
                    pdot_ssat = pdot_s
                # pdot_ssat = pdot_s
                # # print("omega_ssat: %.2f\t %.2f\t %.2f\t pdot_ssat: %.2f\t %.2f\t %.2f" % (omega_ssat[0], omega_ssat[1],omega_ssat[2], pdot_ssat[0],pdot_ssat[1],pdot_ssat[2]))
                #####convert ang vel and lin vel to body frame
                pdot_bsat = qAd(qconj(q), pdot_ssat)
                omega_bsat = qAd(qconj(q), omega_ssat)
                #####Compare to last vel cmd    supply && make vel cmd smooth
                if (pdot_bsat[0] == 0) & (pdot_bsat[1] == 0):
                    pdot_bsat = pdot_bsat_last
                else:
                    pdot_inc = pdot_bsat - pdot_bsat_last
                    pdot_inc_mag = np.sqrt(np.dot(pdot_inc, pdot_inc.T))
                    if pdot_inc_mag > MAX_LINEAR*0.1:
                        pdot_bsat = pdot_bsat_last + pdot_inc/pdot_inc_mag*MAX_LINEAR*0.1
                if (omega_bsat[2] == 0) & (omega_bsat[1] == 0):
                    omega_bsat = omega_bsat_last
                else:
                    omega_inc = omega_bsat - omega_bsat_last
                    omega_inc_mag = np.sqrt(np.dot(omega_inc, omega_inc.T))
                    if omega_inc_mag > MAX_ANG_VEL*0.1:
                        omega_bsat = omega_bsat_last + omega_inc/omega_inc_mag*MAX_ANG_VEL*0.1
                ####output
                cmd = ''
                twist.linear.x = pdot_bsat[0]
                twist.linear.y = pdot_bsat[1]
                twist.linear.z = pdot_bsat[2]
                twist.angular.x = omega_bsat[0]
                twist.angular.y = omega_bsat[1]
                twist.angular.z = omega_bsat[2]
                print('control scheme: press "s" to hover')
                print("currently:\t linear vel x %.2f\t linear vel y %.2f\t linear vel z %.2f\t angular vel x %.2f\t angular vel y %.2f\t angular vel z %.2f" % (
                    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z))
                multi_cmd_vel_flu_pub[0].publish(twist)
                #####remember
                pdot_bsat_last = pdot_bsat
                omega_bsat_last = omega_bsat
                rospy.Rate(100).sleep()
        else:
            for i in range(10):
                if key == str(i):
                    # cmd = formation_configs[i]
                    print_msg()
                    # print(cmd)
                    # cmd_vel_mask = True
            if (key == '\x03'):
                break

        if forward > MAX_LINEAR:
            forward = MAX_LINEAR
        elif forward < -MAX_LINEAR:
            forward = -MAX_LINEAR
        if leftward > MAX_LINEAR:
            leftward = MAX_LINEAR
        elif leftward < -MAX_LINEAR:
            leftward = -MAX_LINEAR
        if upward > MAX_LINEAR:
            upward = MAX_LINEAR
        elif upward < -MAX_LINEAR:
            upward = -MAX_LINEAR
        if angular > MAX_ANG_VEL:
            angular = MAX_ANG_VEL
        elif angular < -MAX_ANG_VEL:
            angular = - MAX_ANG_VEL
            
        twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
        twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular

        for i in range(multirotor_num):
            if ctrl_leader:
                if control_type == 'vel':
                    leader_cmd_vel_flu_pub.publish(twist)
                else:
                    leader_cmd_accel_flu_pub.publish(twist)
                leader_cmd_pub.publish(cmd)
            else:
                if not cmd_vel_mask:
                    if control_type == 'vel':
                        multi_cmd_vel_flu_pub[i].publish(twist)   
                    else:
                        multi_cmd_accel_flu_pub[i].publish(twist)
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
