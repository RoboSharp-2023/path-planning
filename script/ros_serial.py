#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial

class UARTNode(Node):
    def __init__(self):
        super().__init__('uart_node')
        
        serial.Serial
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        
        self.create_timer(1.0, self.send_data)
        self.create_timer(1.0, self.recv_data)

    def send_data(self):
        self.get_logger().info("send")
        data = b'a'
        self.serial_port.write(data)
    
    def recv_data(self):
        data = self.serial_port.read(1)
        self.get_logger().info('data: ' + str(data))

def main(args=None):
    rclpy.init(args=args)
    node = UARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
