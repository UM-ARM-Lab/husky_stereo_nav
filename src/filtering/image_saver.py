#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageSaver:
    def __init__(self):
        # rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.br = CvBridge()
        
    def image_callback(self, msg: Image):
        img = self.br.imgmsg_to_cv2(msg)
        cv2.imwrite(f"data/rgb_image{msg.header.stamp}.png", img)
        

def main():
    rospy.init_node("image_saver")
    saver = ImageSaver()
    msg = rospy.wait_for_message("/zed2i/zed_node/rgb/image_rect_color", Image)
    saver.image_callback(msg)

if __name__ == "__main__":
    main()