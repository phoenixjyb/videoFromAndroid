#!/usr/bin/env python3
"""
Test script for ROS2 camera control.
Demonstrates how to control the Android camera via ROS2 topics.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import time
import sys


class CameraControlTester(Node):
    def __init__(self):
        super().__init__('camera_control_tester')
        
        # Publishers for camera control commands (RELIABLE for commands)
        self.zoom_pub = self.create_publisher(String, '/recomo/rgb/cmd/zoom', 10)
        self.camera_pub = self.create_publisher(String, '/recomo/rgb/cmd/camera', 10)
        self.lock_pub = self.create_publisher(String, '/recomo/rgb/cmd/lock', 10)
        self.record_pub = self.create_publisher(String, '/recomo/rgb/cmd/record', 10)
        self.profile_pub = self.create_publisher(String, '/recomo/rgb/cmd/profile', 10)
        self.keyframe_pub = self.create_publisher(String, '/recomo/rgb/cmd/keyframe', 10)
        
        # Subscriber for telemetry (BEST_EFFORT to match publisher)
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.telemetry_sub = self.create_subscription(
            String,
            '/recomo/rgb/telemetry',
            self.telemetry_callback,
            telemetry_qos
        )
        
        self.get_logger().info("Camera control tester initialized")
        self.get_logger().info("Waiting 2 seconds for connections...")
        time.sleep(2)
    
    def telemetry_callback(self, msg):
        """Display received telemetry"""
        self.get_logger().info(f"Telemetry: {msg.data}")
    
    def set_zoom(self, ratio):
        """Set zoom ratio (e.g., 2.0 for 2x zoom)"""
        msg = String()
        msg.data = str(ratio)
        self.zoom_pub.publish(msg)
        self.get_logger().info(f"✓ Sent zoom command: {ratio}x")
    
    def switch_camera(self, facing):
        """Switch camera ('front' or 'back')"""
        msg = String()
        msg.data = facing
        self.camera_pub.publish(msg)
        self.get_logger().info(f"✓ Sent camera switch command: {facing}")
    
    def set_ae_lock(self, enabled):
        """Set auto-exposure lock"""
        msg = String()
        msg.data = f"ae:{'true' if enabled else 'false'}"
        self.lock_pub.publish(msg)
        self.get_logger().info(f"✓ Sent AE lock command: {enabled}")
    
    def set_awb_lock(self, enabled):
        """Set auto-white-balance lock"""
        msg = String()
        msg.data = f"awb:{'true' if enabled else 'false'}"
        self.lock_pub.publish(msg)
        self.get_logger().info(f"✓ Sent AWB lock command: {enabled}")
    
    def start_recording(self):
        """Start recording on device"""
        msg = String()
        msg.data = 'start'
        self.record_pub.publish(msg)
        self.get_logger().info("✓ Sent start recording command")
    
    def stop_recording(self):
        """Stop recording on device"""
        msg = String()
        msg.data = 'stop'
        self.record_pub.publish(msg)
        self.get_logger().info("✓ Sent stop recording command")
    
    def set_profile(self, width, height, fps):
        """Set video profile (resolution and FPS)"""
        msg = String()
        msg.data = f"{width}x{height}@{fps}"
        self.profile_pub.publish(msg)
        self.get_logger().info(f"✓ Sent profile command: {width}x{height}@{fps}")
    
    def request_keyframe(self):
        """Request a keyframe"""
        msg = String()
        msg.data = ''
        self.keyframe_pub.publish(msg)
        self.get_logger().info("✓ Sent keyframe request")
    
    def run_test_sequence(self):
        """Run a demonstration sequence of camera controls"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Starting Camera Control Test Sequence")
        self.get_logger().info("="*60 + "\n")
        
        # Test 1: Zoom control
        self.get_logger().info("[Test 1] Zoom Control")
        self.set_zoom(1.0)
        time.sleep(2)
        self.set_zoom(2.5)
        time.sleep(2)
        self.set_zoom(1.0)
        time.sleep(2)
        
        # Test 2: Camera switch
        self.get_logger().info("\n[Test 2] Camera Switch")
        self.switch_camera('front')
        time.sleep(3)
        self.switch_camera('back')
        time.sleep(2)
        
        # Test 3: Exposure lock
        self.get_logger().info("\n[Test 3] Auto-Exposure Lock")
        self.set_ae_lock(True)
        time.sleep(3)
        self.set_ae_lock(False)
        time.sleep(2)
        
        # Test 4: Keyframe request
        self.get_logger().info("\n[Test 4] Keyframe Request")
        self.request_keyframe()
        time.sleep(1)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Test Sequence Complete!")
        self.get_logger().info("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    tester = CameraControlTester()
    
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        # Run automated test sequence
        tester.run_test_sequence()
    else:
        # Interactive mode
        print("\n" + "="*60)
        print("Camera Control Tester - Interactive Mode")
        print("="*60)
        print("\nAvailable commands:")
        print("  zoom <ratio>          - Set zoom (e.g., 'zoom 2.5')")
        print("  camera <front|back>   - Switch camera")
        print("  ae <on|off>           - Auto-exposure lock")
        print("  awb <on|off>          - Auto-white-balance lock")
        print("  record <start|stop>   - Recording control")
        print("  profile <W>x<H>@<FPS> - Set video profile (e.g., '1920x1080@30')")
        print("  keyframe              - Request keyframe")
        print("  demo                  - Run automated test sequence")
        print("  quit                  - Exit")
        print("\n")
        
        while rclpy.ok():
            try:
                cmd = input(">>> ").strip().lower()
                if not cmd:
                    continue
                
                parts = cmd.split()
                action = parts[0]
                
                if action == 'quit':
                    break
                elif action == 'demo':
                    tester.run_test_sequence()
                elif action == 'zoom' and len(parts) > 1:
                    tester.set_zoom(float(parts[1]))
                elif action == 'camera' and len(parts) > 1:
                    tester.switch_camera(parts[1])
                elif action == 'ae' and len(parts) > 1:
                    tester.set_ae_lock(parts[1] in ['on', 'true', '1'])
                elif action == 'awb' and len(parts) > 1:
                    tester.set_awb_lock(parts[1] in ['on', 'true', '1'])
                elif action == 'record' and len(parts) > 1:
                    if parts[1] == 'start':
                        tester.start_recording()
                    elif parts[1] == 'stop':
                        tester.stop_recording()
                elif action == 'profile' and len(parts) > 1:
                    # Parse WxH@FPS format
                    profile_parts = parts[1].replace('x', '@').split('@')
                    if len(profile_parts) >= 3:
                        tester.set_profile(int(profile_parts[0]), int(profile_parts[1]), int(profile_parts[2]))
                elif action == 'keyframe':
                    tester.request_keyframe()
                else:
                    print("Unknown command or missing parameters")
                
                # Allow time for command to be sent
                rclpy.spin_once(tester, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
