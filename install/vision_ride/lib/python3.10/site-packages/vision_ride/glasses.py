import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
from time import time


class Glasses(Node):  # Publish to <glasses_source>
    def __init__(self) -> object:
        super().__init__('glasses')
        self.publisher = self.create_publisher(Image, 'glasses_source', 1)
        self.timer = self.create_timer(0.01, self.publish_image)

        self.cap = cv.VideoCapture(0)
        self.bridge = CvBridge()
        self.frame = None
        
        self.fps = None
        self.fps_tmr = time()

    def publish_image(self):
        ret, self.frame = self.cap.read()
        if not ret:
            pass

        dst = self.put_fps()
        self.publisher.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))

        # self.publisher.publish(self.bridge.cv2_to_imgmsg(dst))

    def put_fps(self, src=None) -> int:
        self.fps = int(1 // (time() - self.fps_tmr))
        dst = src if src is not None else self.frame

        cv.putText(dst,
                   str(self.fps),
                   (5, dst.shape[0] - 5),
                   cv.FONT_HERSHEY_SIMPLEX,
                   0.5,
                   (255, 255, 255),
                   1)
        self.fps_tmr = time()
        return dst


def main():
    rclpy.init()

    glasses = Glasses()
    rclpy.spin(glasses)
    
    glasses.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
