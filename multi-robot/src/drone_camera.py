#!/usr/bin/env python

import rospy
import cv2, cv_bridge
from sensor_msgs.msg import Image

class drone_camera:
    def __init__(self, drone_N):
        assert int(drone_N) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
        rospy.init_node("drone{}_camera_vision".format(drone_N), anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/drone{}/front_cam/camera/image'.format(drone_N), Image, self.image_callback)

    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        cv2.namedWindow("drone{}_camera_window".format(drone_N), 1)
        cv2.imshow("drone{}_camera_window".format(drone_N), image)
        cv2.waitKey(1)


if __name__ == '__main__':
    print("Let's view your drone camera")
    drone_N = input("Select a drone camera to view. (Options 1, 2, 3, 4, 5, 6, 7, 8, 9, 10): ")
    follower = drone_camera(drone_N)
    rospy.spin()


