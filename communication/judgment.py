from geometry_msgs.msg import Point
from std_msgs.msg import Header
from darknet_ros_msgs import BoundingBoxes

class Judgment():
    def __init__(self):
        self.header = Header()
        self.vehicle_id = ''
        self.position = Point()
        self.actor_id = ''
        self.actor_bbox = BoundingBoxes()
