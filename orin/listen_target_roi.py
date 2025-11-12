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
        """Process received ROI message"""
        self.msg_count += 1
        
        # Convert back to normalized coordinates (0.0-1.0)
        x_norm = msg.x_offset / 10000.0
        y_norm = msg.y_offset / 10000.0
        width_norm = msg.width / 10000.0
        height_norm = msg.height / 10000.0
        
        # Calculate center point from top-left corner
        center_x = x_norm + (width_norm / 2.0)
        center_y = y_norm + (height_norm / 2.0)
        
        # Determine if this is a tap point or full ROI
        is_tap_point = abs(width_norm - 0.1) < 0.001 and abs(height_norm - 0.1) < 0.001
        
        if is_tap_point:
            self.get_logger().info(
                f'📍 Tap #{self.msg_count}: '
                f'Center=({center_x:.3f}, {center_y:.3f}) '
                f'[default 10% box]'
            )
        else:
            self.get_logger().info(
                f'🔲 ROI #{self.msg_count}: '
                f'TopLeft=({x_norm:.3f}, {y_norm:.3f}) '
                f'Size=({width_norm:.3f}×{height_norm:.3f}) '
                f'Center=({center_x:.3f}, {center_y:.3f})'
            )
        
        # Raw values for debugging
        self.get_logger().debug(
            f'   Raw: x_offset={msg.x_offset}, y_offset={msg.y_offset}, '
            f'width={msg.width}, height={msg.height}'
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
