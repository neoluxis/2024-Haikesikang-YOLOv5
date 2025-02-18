import rclpy
from rclpy.node import Node
from gz.threCam import ThreadCap

from sensor_msgs.msg import Image

import cv2 as cv
import numpy as np


def cv2ros(Img):
    rimg = cv.imencode(".jpeg", Img)[1].tobytes()
    return rimg


class QrcCam(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("cam_idx", 8)
        self.declare_parameter("fps", 240)
        self.declare_parameter("img_width", 640)
        self.declare_parameter("img_height", 400)
        # self.declare_parameter("img_fourcc", cv.VideoWriter.fourcc(*"MJPG"))

        self.get_logger().info(f"QrcCam Node {name}")
        self.cam = ThreadCap(
            self.get_parameter("cam_idx").get_parameter_value().integer_value,
            self.get_parameter("img_width").get_parameter_value().integer_value,
            self.get_parameter("img_height").get_parameter_value().integer_value,
            self.get_parameter("fps").get_parameter_value().integer_value,
        )

        self.qrc_image_pub = self.create_publisher(Image, "qrc_image", 10)
        timer_period = 0.01  # seconds
        self.qrc_image_pub_timer = self.create_timer(
            timer_period, self.qrc_image_pub_callback
        )
        self.msg = Image()
        
        self.frame_count = 0
        self.fps_timer = self.create_timer(
            1, self.fps_callback
        )

    def qrc_image_pub_callback(self):
        _, frame = self.cam.read()
        if frame is None:
            return
        frame = cv.resize(frame, (0, 0),fx= 0.2, fy=0.2)
        # cv.imshow("fr", frame)
        # cv.waitKey(1)
        self.msg.data = cv2ros(frame)
        self.msg.width = frame.shape[1]
        self.msg.height = frame.shape[0]
        self.msg.encoding = 'jpeg'
        self.msg.step = frame.shape[1] * 2
        self.qrc_image_pub.publish(self.msg)
        self.frame_count += 1
        
    def fps_callback(self):
        self.get_logger().info(f'QRC Cam: Pub FPS: {self.frame_count}')
        self.frame_count = 0


def main():
    rclpy.init()
    qrccam = QrcCam("qrc_cam")
    rclpy.spin(qrccam)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
