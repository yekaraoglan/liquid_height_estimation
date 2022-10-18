#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int64, Float64

import cv2
from cv_bridge import CvBridge, CvBridgeError

class InfoWindow:
    def __init__(self):
        rospy.init_node('opencv_example', anonymous=True)
        rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_cb)
        rospy.Subscriber("/height_detector/percentage", Int64, self.percentage_cb)
        rospy.Subscriber("/height_detector/height", Float64, self.height_cb)
        self.image_pub = rospy.Publisher("/height_detector/info_image", Image, queue_size=10)

        self.bridge = CvBridge()
        self.percentage = None
        self.height = None

    def percentage_cb(self, msg):
        self.percentage = msg.data

    def height_cb(self, msg):
        self.height = msg.data

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        cv2.putText(img=cv_image, text="percentage: "+str(self.percentage)+"%", org=(350, 25), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=1, color=(0, 95, 220),thickness=2)
        cv2.putText(img=cv_image, text="height: "+str(self.height)[:6], org=(350, 58), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=1, color=(0, 95, 220),thickness=2)
        img = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
        self.image_pub.publish(img)


if __name__ == '__main__':
    try:
        InfoWindow()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("ROS Interrupt Exception")
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
        
            
