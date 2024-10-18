import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets 

from serial import Serial

ser_dev = '/dev/ttyS3'

class ByteArray(bytearray):
    def __init__(self, data):
        super().__init__(data)
        self.data = data

    def __str__(self):
        return f"[{', '.join([f'0x{byte:02X}' for byte in self.data])}]"

class GzSerial(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f'Init serial port, node {name}')
        self.ser = Serial(ser_dev, 115200)
        self.get_logger().info(f'Serial port {ser_dev} init')
        self.model_res = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.det_callback, 10)

    def det_callback(self, msg):
        #self.get_logger().info("Det recvd!")
        #print(msg.targets)
        for tg in msg.targets:
            roi = tg.rois[0].rect
            ctx = roi.x_offset + roi.width // 2
            cty = roi.y_offset + roi.height // 2
            conf = tg.rois[0].confidence  
            self.get_logger().info(f'{tg.type}, {ctx}, {cty}, {conf:.2f}')
            self.send(tg.type, ctx, cty)
    
    def name2ser(self, c):
        if c == 'rcf':
            return 0x31  # 49
        elif c == 'gcf':
            return 0x32  # 50
        elif c == 'bcf':
            return 0x33  # 51
        elif c == 'rof':
            return 0x34  # 52
        elif c == 'gof':
            return 0x35  # 53
        elif c == 'bof':
            return 0x36  # 54
        return 0x00

    def send(self, name, ctx, cty):
        sent = [0xFF]
        sent.append(self.name2ser(name))
        sent.append(int(ctx * 253 / 640))
        sent.append(int(cty * 253 / 480))
        sent.append(0xFE)
        self.get_logger().info(f'Data: {sent}')
        sent = ByteArray(sent)
        self.get_logger().info(f'Sent: {sent}')
    


def main(args=None):
    rclpy.init(args=args)
    node = GzSerial("gz_serial")
    rclpy.spin(node)
    rclpy.shutdown()


