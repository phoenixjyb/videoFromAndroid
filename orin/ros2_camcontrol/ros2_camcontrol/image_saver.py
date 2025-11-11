#!/usr/bin/env python3
"""
Test subscriber that saves received images to disk with frame counting.
Each run creates a timestamped folder to organize saved images.
"""
import os
import sys
import time
from pathlib import Path
from datetime import datetime
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
import numpy as np

try:
    from PIL import Image as PILImage
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("Warning: PIL not available, will save raw RGB data instead of PNG")


class ImageSaver(Node):
    def __init__(self, topic_name='/recomo/rgb', output_dir='saved_images', max_images=None, save_format='png'):
        super().__init__('image_saver')
        
        # Create timestamped output directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = Path(output_dir) / f"run_{timestamp}"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Saving images to: {self.output_dir}")
        
        self.max_images = max_images
        self.save_format = save_format.lower()
        if self.save_format not in ['png', 'jpg', 'raw']:
            self.get_logger().warning(f"Unknown format '{self.save_format}', using 'png'")
            self.save_format = 'png'
        
        if self.save_format != 'raw' and not HAS_PIL:
            self.get_logger().warning("PIL not available, forcing raw format")
            self.save_format = 'raw'
        
        # Counters and timing
        self.frame_count = 0
        self.first_frame_time = None
        self.last_frame_time = None
        self.start_time = time.time()
        
        # Use BEST_EFFORT QoS to match publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            qos
        )
        
        self.get_logger().info(f"Subscribed to {topic_name} with BEST_EFFORT QoS")
        if max_images:
            self.get_logger().info(f"Will save up to {max_images} images")
        
        # Create stats file
        self.stats_file = self.output_dir / 'stats.txt'
        with open(self.stats_file, 'w') as f:
            f.write(f"Image Saver Statistics\n")
            f.write(f"Started: {datetime.now().isoformat()}\n")
            f.write(f"Topic: {topic_name}\n")
            f.write(f"Format: {self.save_format}\n")
            f.write(f"Max images: {max_images if max_images else 'unlimited'}\n")
            f.write(f"\n")

    def image_callback(self, msg):
        """Callback for received images"""
        current_time = time.time()
        
        # Track timing
        if self.first_frame_time is None:
            self.first_frame_time = current_time
            self.get_logger().info(f"First frame received: {msg.width}x{msg.height}, encoding={msg.encoding}")
        
        # Check if we should stop
        if self.max_images and self.frame_count >= self.max_images:
            if self.frame_count == self.max_images:
                self.print_summary()
                self.get_logger().info(f"Reached max_images={self.max_images}, stopping...")
                rclpy.shutdown()
            return
        
        # Save the image
        try:
            self.save_image(msg)
            self.frame_count += 1
            
            # Calculate rates
            if self.last_frame_time is not None:
                frame_delta = current_time - self.last_frame_time
                instant_hz = 1.0 / frame_delta if frame_delta > 0 else 0
            else:
                instant_hz = 0
            
            # Log progress every 10 frames
            if self.frame_count % 10 == 0:
                elapsed = current_time - self.first_frame_time
                avg_hz = self.frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f"Saved {self.frame_count} frames | "
                    f"Avg: {avg_hz:.2f} Hz | Instant: {instant_hz:.2f} Hz"
                )
            
            self.last_frame_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Failed to save frame {self.frame_count}: {e}")
    
    def save_image(self, msg):
        """Save image to disk"""
        # Convert ROS Image to numpy array
        if msg.encoding == 'rgb8':
            dtype = np.uint8
            channels = 3
        elif msg.encoding == 'rgba8':
            dtype = np.uint8
            channels = 4
        elif msg.encoding == 'mono8':
            dtype = np.uint8
            channels = 1
        else:
            self.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
            return
        
        # Handle row padding (step != width * channels)
        if msg.step == msg.width * channels:
            # No padding, direct reshape
            img_array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        else:
            # Has padding, need to handle step
            img_array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step)
            img_array = img_array[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        
        # Save based on format
        filename = f"frame_{self.frame_count:06d}.{self.save_format}"
        filepath = self.output_dir / filename
        
        if self.save_format == 'raw':
            # Save raw numpy array
            np.save(filepath.with_suffix('.npy'), img_array)
        else:
            # Save using PIL
            if channels == 1:
                img_array = img_array.squeeze()
            pil_img = PILImage.fromarray(img_array)
            pil_img.save(filepath, quality=95 if self.save_format == 'jpg' else None)
    
    def print_summary(self):
        """Print and save summary statistics"""
        end_time = time.time()
        total_time = end_time - self.start_time
        
        if self.first_frame_time:
            capture_time = end_time - self.first_frame_time
            avg_hz = self.frame_count / capture_time if capture_time > 0 else 0
        else:
            capture_time = 0
            avg_hz = 0
        
        summary = [
            f"\n{'='*60}",
            f"Image Saver Summary",
            f"{'='*60}",
            f"Total frames saved: {self.frame_count}",
            f"Output directory: {self.output_dir}",
            f"Total runtime: {total_time:.2f}s",
            f"Capture duration: {capture_time:.2f}s",
            f"Average rate: {avg_hz:.2f} Hz",
            f"{'='*60}\n"
        ]
        
        summary_text = '\n'.join(summary)
        self.get_logger().info(summary_text)
        
        # Append to stats file
        with open(self.stats_file, 'a') as f:
            f.write(f"\nCompleted: {datetime.now().isoformat()}\n")
            f.write(f"Total frames: {self.frame_count}\n")
            f.write(f"Capture duration: {capture_time:.2f}s\n")
            f.write(f"Average rate: {avg_hz:.2f} Hz\n")


def main(args=None):
    parser = argparse.ArgumentParser(description='Save ROS2 images to disk')
    parser.add_argument('--topic', default='/recomo/rgb', help='Topic to subscribe to')
    parser.add_argument('--output-dir', default='saved_images', help='Base output directory')
    parser.add_argument('--max-images', type=int, help='Maximum number of images to save')
    parser.add_argument('--format', default='png', choices=['png', 'jpg', 'raw'], 
                        help='Image format (png, jpg, or raw numpy)')
    
    # Parse args (handle both ROS args and our args)
    parsed_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    
    try:
        node = ImageSaver(
            topic_name=parsed_args.topic,
            output_dir=parsed_args.output_dir,
            max_images=parsed_args.max_images,
            save_format=parsed_args.format
        )
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Interrupted by user")
        finally:
            node.print_summary()
            node.destroy_node()
    
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
