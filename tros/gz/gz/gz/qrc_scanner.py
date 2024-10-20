import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2 as cv
import numpy as np
import pyzbar.pyzbar as pyzbar
import time


def ros2cv(RImg: Image):
    if RImg is None:
        return None
    np_arr = np.frombuffer(RImg.data, np.uint8)
    cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
    return cv_image


class QrcScanner(Node):
    def __init__(self, name):
        super().__init__(name)

        self.qrc_img_sub = self.create_subscription(
            Image,
            "qrc_image",
            self.scan_code,
            10,
        )
        self.res_pub = self.create_publisher(String, "qrc_result", 10)

    def scan_code(self, msg):
        cv_img = ros2cv(msg)
        if cv_img is None:
            return
        t0 = time.time()
        decoded_objects = pyzbar.decode(cv_img)
        if decoded_objects:
            text = decoded_objects[0].data.decode("utf-8")
            self.get_logger().info(f"Detected: {text}")
            self.res_pub.publish(String(data=text))
        self.get_logger().info(f"Scan time: {time.time() - t0:.3f}")

        # cv.imshow("qrc", cv_img)
        # cv.waitKey(1)


def main():
    rclpy.init()
    qrc_scanner = QrcScanner("qrc_cam")
    rclpy.spin(qrc_scanner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
