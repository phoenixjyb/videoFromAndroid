#!/usr/bin/env python3
"""
ROS2 Target ROI Listener
=========================

Listens to the /target_roi topic and displays received target coordinates
in a human-readable format.

Usage:
    python3 listen_target_roi.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest


class TargetROIListener(Node):
    """ROS2 node for listening to target ROI messages"""
    
    def __init__(self):
        super().__init__('target_roi_listener')
        self.subscription = self.create_subscription(
            RegionOfInterest,
            '/target_roi',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.msg_count = 0
        self.get_logger().info('🎯 Target ROI Listener started')
        self.get_logger().info('Listening on /target_roi topic...')
        self.get_logger().info('Tap on CamViewer to see messages\n')
    
    def listener_callback(self, msg: RegionOfInterest):
        """Display received ROI message - just log the raw data"""
        self.msg_count += 1
        
        self.get_logger().info(
            f'🎯 ROI #{self.msg_count}: '
            f'x_offset={msg.x_offset}, '
            f'y_offset={msg.y_offset}, '
            f'width={msg.width}, '
            f'height={msg.height}, '
            f'do_rectify={msg.do_rectify}'
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    listener = TargetROIListener()
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        listener.get_logger().info('\n👋 Shutting down listener...')
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
