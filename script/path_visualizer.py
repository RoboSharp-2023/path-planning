#!/usr/bin/env python3

import matplotlib.pyplot as pl
import rclpy
from rclpy.node import Node
from path_planning.msg import PathData

class visualizer(Node):
    is_showed = False
    
    def __init__(self) -> None:
        super().__init__('visualizer')
        
        self.subscription = self.create_subscription(
            msg_type=PathData, topic='path_', callback=self.callback, qos_profile=10
        )
        
        self.subscription
    
    def callback(self, msg):
        if not self.is_showed:
            
            pl.plot(msg.x, msg.y)
            pl.show()
        self.get_logger().info("callback")
        self.is_showed=True

def main(args=None):
    rclpy.init(args=args)
    
    sub = visualizer()
    rclpy.spin(sub)
    
    sub.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

