#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import numpy as np
import cv2
import threading

class CVImage:
    def __init__(self):
        """
        Initialize the CVImage class, setting up ROS node, subscribers, and publishers.
        """
        self._last_msg = None
        self._msg_lock = threading.Lock()
        self.bridge = CvBridge()
        self._publish_rate = rospy.get_param('~publish_rate', 50)

        # Subscribe to image topic
        self.image_sub = rospy.Subscriber(
            "~image_topic", Image, self._image_callback, queue_size=1
        )
        # Publisher for debug image
        self.image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)

    def convert_image_cv(self, ros_image, encoder="passthrough"):
        """
        Convert a ROS Image message to an OpenCV image.

        Args:
            ros_image (sensor_msgs.msg.Image): The ROS Image message to convert.
            encoder (str): Encoding type for the conversion.

        Returns:
            np.ndarray: The converted OpenCV image.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, encoder)
            return cv_image
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
            return None

    def _image_callback(self, msg):
        """
        Callback function for the image subscriber.

        Args:
            msg (sensor_msgs.msg.Image): The received Image message.
        """
        rospy.logdebug("Received an image")
        with self._msg_lock:
            self._last_msg = msg

    def service_handler(self, request):
        """
        Service handler for image processing requests.

        Args:
            request: The service request containing an image.

        Returns:
            The result of processing the image.
        """
        return self._image_callback(request.image)

    def run(self):
        """
        Main run loop for processing and publishing images.
        """
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            with self._msg_lock:
                msg = self._last_msg
                self._last_msg = None

            if msg is not None:
                img = self.convert_image_cv(msg)
                if img is not None:
                    rospy.logdebug(f"Image shape: {img.shape}")
                    self.publish_image(img, self.image_pub, msg.header)

            self.sleep(rate)

    def publish_image(self, image, publisher, camera_header, encoder="passthrough"):
        """
        Publish an OpenCV image as a ROS Image message.

        Args:
            image (np.ndarray): The OpenCV image to publish.
            publisher (rospy.Publisher): The ROS publisher to use.
            camera_header (std_msgs.msg.Header): The header for the ROS Image message.
            encoder (str): Encoding type for the conversion.
        """
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, encoder)
            image_msg.header = camera_header
            publisher.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error publishing image: {e}")

    def sleep(self, rate):
        """
        Sleep for the specified rate, handling exceptions.

        Args:
            rate (rospy.Rate): The rate to sleep for.
        """
        try:
            rate.sleep()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(f"Interrupted during sleep: {e}")
        except rospy.ROSTimeMovedBackwardsException as e:
            rospy.logerr(f"Time moved backwards: {e}")

def main():
    """
    Main entry point for the script.
    """
    rospy.init_node('cv_image')
    obj = CVImage()
    obj.run()

if __name__ == '__main__':
    main()
