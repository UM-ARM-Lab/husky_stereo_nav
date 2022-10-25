import rospy
import cv2
from cv_bridge import CvBridge

def main():
    rospy.init_node("image_saver")
    rospy.Subscriber("/zed2i/zed_node/rgb/")

if __name__ == "__main__":
    main()