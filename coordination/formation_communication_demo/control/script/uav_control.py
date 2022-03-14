from PingPro.msg import AllNeighborMsg
from geometry_msgs.msg import Vector3,PoseStamped,TwistStamped
import rospy

def uav_control():
    neighbor_num = len(self.all_neighbor_data.data)
    local_dersireRelaPose = self.all_desired_position[self.uav_id]
    for i in range(neighbor_num):
        self.local_velocity.twist.linear.x += self.all_neighbor_data.data[i].uav_data.pose.position.x - local_pose.pose.position.x + self.all_neighbor_data.data[i].dersireRelaPose.x - local_dersireRelaPose.x
        self.local_velocity.twist.linear.y += self.all_neighbor_data.data[i].uav_data.pose.position.y - local_pose.pose.position.y + self.all_neighbor_data.data[i].dersireRelaPose.y - local_dersireRelaPose.y
        self.local_velocity.twist.linear.z += self.all_neighbor_data.data[i].uav_data.pose.position.z - local_pose.pose.position.z + self.all_neighbor_data.data[i].dersireRelaPose.z - local_dersireRelaPose.z
    self.local_velocity.twist.linear.x = self.local_velocity.twist.linear.x * self.Kpx
    self.local_velocity.twist.linear.y = self.local_velocity.twist.linear.y * self.Kpy
    self.local_velocity.twist.linear.z = self.local_velocity.twist.linear.z * self.Kpz
    if self.local_velocity.twist.linear.x > self.velxy_max:
        self.local_velocity.twist.linear.x = self.velxy_max
    elif self.local_velocity.twist.linear.x < -self.velxy_max:
        self.local_velocity.twist.linear.x = -self.velxy_max
    if self.local_velocity.twist.linear.y > self.velxy_max:
        self.local_velocity.twist.linear.y = self.velxy_max
    elif self.local_velocity.twist.linear.y < -self.velxy_max:
        self.local_velocity.twist.linear.y = -self.velxy_max
    if self.local_velocity.twist.linear.z > self.velz_max:
        self.local_velocity.twist.linear.z = self.velz_max
    elif self.local_velocity.twist.linear.z < -self.velz_max:
        self.local_velocity.twist.linear.z = -self.velz_max

if __name__ == '__main__':
    allneighbor = AllNeighborMsg()
    uav_control(allneighbor)

