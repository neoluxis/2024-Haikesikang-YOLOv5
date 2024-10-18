import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets 

from serial import Serial

ser_dev = '/dev/ttyS3'

class GzSerial(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f'Init serial port, node {name}')
        self.ser = Serial(ser_dev, 115200)
        self.get_logger().info(f'Serial port {ser_dev} init')
        self.model_res = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.det_callback, 10)

    def det_callback(self, msg):
        self.get_logger().info("Det recvd!")
        #print(msg.targets)
        for tg in msg.targets:
            roi = tg.rois[0].rect
            ctx = roi.x_offset + roi.width // 2
            cty = roi.y_offset + roi.height // 2
            print(tg.type, 
                ctx, cty)
    
    def send(self, name, pos):
        print(name, pos)


def main(args=None):
    rclpy.init(args=args)
    node = GzSerial("gz_serial")
    rclpy.spin(node)
    rclpy.shutdown()


