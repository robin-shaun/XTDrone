import rospy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformListener, Buffer
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

class Tracker():
    def __init__(self, vehicle_type, vehicle_id):
        self.K = np.eye(3)
        self.lines_3d = np.array([[[0.05,0,0],[0.05,0.05,0]],[[0.05,0.05,0],[0.05,0.05,0.05]],[[0.05,0.05,0.05],[0.05,0,0.05]],[[0.05,0,0.05],[0.05,0,0]],[[0,0,0],[0,0.05,0]],[[0,0.05,0],[0,0.05,0.05]],[[0,0.05,0.05],[0,0,0.05]],[[0,0,0.05],[0,0,0]],[[0,0,0],[0.05,0,0]],[[0,0.05,0],[0.05,0.05,0]],[[0,0,0.05],[0.05,0,0.05]],[[0,0.05,0.05],[0.05,0.05,0.05]]])
        self.plot_3d()
        rospy.init_node(vehicle_type+'_'+vehicle_id+'_visual_servo')
        rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/le_arm/camera/image_raw', Image, image_callback)
        rospy.Subscriber(vehicle_type+'_'+vehicle_id+'/le_arm/camera/camera_info', CameraInfo, camera_info_callback)
        self.image_pub = rospy.Publisher(vehicle_type+'_'+vehicle_id+'/le_arm/visual_servo/image', Image, queue_size=2)

        rate = rospy.Rate(20) 
        
        while not rospy.is_shutdown():
            rate.sleep()

    def plot_3d(self):
        for line_3d in self.lines_3d:
            print(zip(line_3d[0],line_3d[1]))
            ax.plot(*zip(line_3d[0],line_3d[1]),color="b")
        plt.show()

    def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
        line_img = np.zeros(
            (
                img.shape[0],
                img.shape[1],
                3
            ),
            dtype=np.uint8
        )
        img = np.copy(img)
        if lines is None:
            return
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
        img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
        return img

    def camera_info_callback(self, msg):
        self.K = np.array(msg.K).reshape([3,3])

    def image_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        cannyed_image = cv2.Canny(gray_image, 100, 200)
        # cv2.imshow("cannyed_image", cannyed_image)
        # cv2.waitKey(0)
        # lines = cv2.HoughLinesP(
        #     cannyed_image,
        #     rho=6,
        #     theta=np.pi / 180,
        #     threshold=50,
        #     lines=np.array([]),
        #     minLineLength=5,
        #     maxLineGap=50
        # )
        # if lines is None:
        #     return
        # line_image = draw_lines(cv_image,lines,thickness=5)

        img_ros = bridge.cv2_to_imgmsg(cannyed_image)
        image_pub.publish(img_ros)
    

if __name__ == '__main__':
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    tracker = Tracker(vehicle_type, vehicle_id)

