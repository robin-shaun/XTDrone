import rclpy
import time
import sys
import os
import numpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String,Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Twist,Vector3


def load_frame_offset(uav_id):
    """Constant offset from the leader's local frame to this vehicle's, in ENU metres.

    PX4 sets each vehicle's EKF origin from its own first GPS fix, so every vehicle has a
    different local frame. XTDrone's law adds the leader's position, which is in the
    leader's frame, to this vehicle's position, which is in its own. Without the
    correction every follower carries a constant bias of (leader spawn - own spawn) and
    parks in the wrong place. Measured once by calibrate_frames.py; see frame_offsets.yaml.
    """
    path = os.environ.get(
        "XTD_FRAME_OFFSETS",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "frame_offsets.yaml"),
    )
    try:
        with open(path, "r", encoding="utf-8") as f:
            offsets = yaml.safe_load(f)["offsets"]
    except Exception as exc:  # noqa: BLE001
        print(f"{path}: not usable ({exc}); assuming a shared local frame")
        return (0.0, 0.0, 0.0)
    value = offsets.get(uav_id, offsets.get(str(uav_id), (0.0, 0.0, 0.0)))
    return (float(value[0]), float(value[1]), float(value[2]))


class Follower(Node):
    def __init__(self,uav_type,uav_id,uav_num):
        self.hover="HOVER"
        self.uav_type=uav_type
        self.uav_num=uav_num
        self.id=uav_id
        self.f=30
        self.pose = PoseStamped()
        self.cmd_vel_enu = Twist()
        self.avoid_vel = Vector3()
        self.formation_pattern = None
        self.Kp = 1.0
        self.Kp_avoid = 2.0
        self.vel_max = 1.0
        self.leader_pose = PoseStamped()
        self.timer_period=0.3
        self.frame_offset = load_frame_offset(uav_id)

        super().__init__("follower"+str(self.id-1))
        self.pose_sub=self.create_subscription(PoseStamped,self.uav_type+'_'+str(self.id)+"/mavros/local_position/pose",self.pose_callback,qos_profile_sensor_data)
        self.avoid_vel_sub=self.create_subscription(Vector3,"/xtdrone/"+self.uav_type+'_'+str(self.id)+"/avoid_vel",self.avoid_vel_callback,10)
        self.formation_pattern_sub=self.create_subscription(Float32MultiArray,"/xtdrone/formation_pattern",self.formation_pattern_callback,10)

        self.vel_enu_pub=self.create_publisher(Twist,'/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd_vel_enu',10)
        self.info_pub=self.create_publisher(String,'/xtdrone/'+self.uav_type+'_'+str(self.id)+'/info',10)
        self.cmd_pub=self.create_publisher(String,'/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd',10)
        self.leader_pose_sub=self.create_subscription(PoseStamped,self.uav_type+"_0/mavros/local_position/pose",self.leader_pose_callback,qos_profile_sensor_data)
        self.timer=self.create_timer(self.timer_period,self.timer_callback)
        
    def pose_callback(self,msg):
        self.pose=msg

    def avoid_vel_callback(self,msg):
        self.avoid_vel=msg

    def formation_pattern_callback(self,msg):
        # dtype=float is required. rclpy hands float32[] over as a numpy array of dtype
        # float32, and numpy.float32 is NOT a subclass of Python float (numpy.float64 is).
        # Without the cast, the arithmetic below yields numpy.float32, and publishing the
        # Twist aborts the process in geometry_msgs__msg__vector3__convert_from_py on
        # PyFloat_Check(field).
        self.formation_pattern=numpy.array(msg.data,dtype=float).reshape(3,self.uav_num-1)

    def leader_pose_callback(self,msg):
        self.leader_pose=msg

    def timer_callback(self):
        if (not self.formation_pattern is None):
            # frame_offset moves the leader's pose into this vehicle's local frame; without
            # it the two positions being subtracted live in different coordinate systems.
            lx = self.leader_pose.pose.position.x + self.frame_offset[0]
            ly = self.leader_pose.pose.position.y + self.frame_offset[1]
            lz = self.leader_pose.pose.position.z + self.frame_offset[2]
            self.cmd_vel_enu.linear.x = self.Kp * ((lx + self.formation_pattern[0, self.id - 1]) - self.pose.pose.position.x)
            self.cmd_vel_enu.linear.y = self.Kp * ((ly + self.formation_pattern[1, self.id - 1]) - self.pose.pose.position.y)
            self.cmd_vel_enu.linear.z = self.Kp * ((lz + self.formation_pattern[2, self.id - 1]) - self.pose.pose.position.z)
            self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.Kp_avoid * self.avoid_vel.x
            self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.Kp_avoid * self.avoid_vel.y
            self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.Kp_avoid * self.avoid_vel.z                
            cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5 
            if cmd_vel_magnitude > 3**0.5 * self.vel_max:
                self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max
                
            self.vel_enu_pub.publish(self.cmd_vel_enu)



def main():
    rclpy.init()
    node=Follower(sys.argv[1],int(sys.argv[2]),int(sys.argv[3]))
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()


