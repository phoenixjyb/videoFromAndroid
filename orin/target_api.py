#!/usr/bin/env python3
"""
Orin Target API Server
======================

REST API server that receives target coordinates from the CamViewer Android app
and publishes them to a ROS2 topic for vision processing.

Usage:
    python3 target_api.py [--port PORT] [--ros2]

Options:
    --port PORT    Port to listen on (default: 8080)
    --ros2         Enable ROS2 publishing (requires ROS2 installation)
    --no-ros2      Disable ROS2 publishing (testing mode)
"""

import asyncio
import logging
from typing import Optional
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, field_validator
import uvicorn
import argparse

# ROS2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import RegionOfInterest
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 not available - running in test mode")


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# Request/Response models
class TargetCoordinates(BaseModel):
    """Target coordinates/bounding box from CamViewer (normalized 0.0-1.0)"""
    x: float  # Center x coordinate (or bounding box x_offset)
    y: float  # Center y coordinate (or bounding box y_offset)
    width: Optional[float] = None  # Bounding box width (optional)
    height: Optional[float] = None  # Bounding box height (optional)
    
    @field_validator('x', 'y', 'width', 'height')
    @classmethod
    def validate_range(cls, v):
        if v is not None and not 0.0 <= v <= 1.0:
            raise ValueError('Coordinate/dimension must be between 0.0 and 1.0')
        return v


class TargetResponse(BaseModel):
    """Response after receiving target coordinates"""
    status: str
    x: float
    y: float
    width: Optional[float] = None
    height: Optional[float] = None
    message: str


# ROS2 Publisher Node (optional)
class TargetPublisher(Node):
    """ROS2 node for publishing target ROI (Region of Interest)"""
    
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(RegionOfInterest, '/target_roi', 10)
        self.get_logger().info('Target ROI publisher node initialized')
    
    def publish_target(self, x: float, y: float, width: Optional[float] = None, height: Optional[float] = None):
        """
        Publish target ROI to ROS2 topic
        
        Args:
            x: Center x coordinate or bounding box x_offset (normalized 0.0-1.0)
            y: Center y coordinate or bounding box y_offset (normalized 0.0-1.0)
            width: Bounding box width (normalized, optional)
            height: Bounding box height (normalized, optional)
        """
        msg = RegionOfInterest()
        
        # Store normalized coordinates (0.0-1.0)
        # x_offset and y_offset represent the top-left corner of the ROI
        # For a simple tap point, use small default box around the point
        if width is None or height is None:
            # Default: 10% box centered on tap point
            msg.x_offset = int((x - 0.05) * 10000)  # Scale to pseudo-pixel for storage
            msg.y_offset = int((y - 0.05) * 10000)
            msg.width = int(0.1 * 10000)  # 10% default width
            msg.height = int(0.1 * 10000)  # 10% default height
        else:
            # Use provided bounding box (assume x,y is top-left corner)
            msg.x_offset = int(x * 10000)
            msg.y_offset = int(y * 10000)
            msg.width = int(width * 10000)
            msg.height = int(height * 10000)
        
        msg.do_rectify = False  # Not using image rectification
        
        self.publisher_.publish(msg)
        
        if width is not None and height is not None:
            self.get_logger().info(
                f'Published ROI: x={x:.4f}, y={y:.4f}, w={width:.4f}, h={height:.4f}'
            )
        else:
            self.get_logger().info(
                f'Published target point: ({x:.4f}, {y:.4f}) with default 10% box'
            )


# Global ROS2 node instance
ros2_node: Optional[TargetPublisher] = None
ros2_enabled = False


# Lifespan context manager for startup/shutdown
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage ROS2 lifecycle"""
    global ros2_node, ros2_enabled
    
    # Startup
    if ros2_enabled and ROS2_AVAILABLE:
        try:
            rclpy.init()
            ros2_node = TargetPublisher()
            logger.info("ROS2 publisher initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize ROS2: {e}")
            ros2_enabled = False
    else:
        logger.info("Running in test mode (ROS2 disabled)")
    
    yield
    
    # Shutdown
    if ros2_node:
        try:
            ros2_node.destroy_node()
            rclpy.shutdown()
            logger.info("ROS2 publisher shutdown successfully")
        except Exception as e:
            logger.error(f"Error during ROS2 shutdown: {e}")


# FastAPI app with lifespan
app = FastAPI(
    title="Orin Target API",
    description="Receives target coordinates from CamViewer and publishes to ROS2",
    version="1.0.0",
    lifespan=lifespan
)

# Enable CORS for mobile device access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify allowed origins
    allow_methods=["*"],
    allow_headers=["*"],
)


# API Endpoints
@app.get("/")
async def root():
    """Root endpoint - API info"""
    return {
        "name": "Orin Target API",
        "version": "1.0.0",
        "ros2_enabled": ros2_enabled,
        "endpoints": {
            "POST /target": "Receive target coordinates",
            "GET /health": "Health check"
        }
    }


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "ok",
        "ros2_enabled": ros2_enabled,
        "ros2_available": ROS2_AVAILABLE
    }


@app.post("/target", response_model=TargetResponse)
async def receive_target(coords: TargetCoordinates):
    """
    Receive target coordinates/ROI from CamViewer app.
    
    Accepts either:
    1. Simple tap point (x, y) - creates a default 10% box around the point
    2. Full bounding box (x, y, width, height) - represents the ROI
    
    Coordinates are normalized (0.0-1.0) where:
    - (0, 0) is top-left corner
    - (1, 1) is bottom-right corner
    
    If ROS2 is enabled, publishes to /target_roi topic.
    """
    if coords.width is not None and coords.height is not None:
        logger.info(
            f"Received target ROI: x={coords.x:.4f}, y={coords.y:.4f}, "
            f"w={coords.width:.4f}, h={coords.height:.4f}"
        )
    else:
        logger.info(f"Received target point: x={coords.x:.4f}, y={coords.y:.4f}")
    
    # Publish to ROS2 if enabled
    if ros2_enabled and ros2_node:
        try:
            ros2_node.publish_target(coords.x, coords.y, coords.width, coords.height)
        except Exception as e:
            logger.error(f"Failed to publish to ROS2: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Failed to publish to ROS2 topic: {str(e)}"
            )
    else:
        logger.debug("ROS2 disabled - coordinates logged only")
    
    return TargetResponse(
        status="success",
        x=coords.x,
        y=coords.y,
        width=coords.width,
        height=coords.height,
        message="Target ROI received" if coords.width else "Target coordinates received"
    )


def main():
    """Main entry point"""
    global ros2_enabled
    
    parser = argparse.ArgumentParser(
        description='Orin Target API Server',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--port',
        type=int,
        default=8080,
        help='Port to listen on (default: 8080)'
    )
    parser.add_argument(
        '--ros2',
        action='store_true',
        help='Enable ROS2 publishing (requires ROS2 installation)'
    )
    parser.add_argument(
        '--no-ros2',
        action='store_true',
        help='Disable ROS2 publishing (testing mode)'
    )
    parser.add_argument(
        '--host',
        type=str,
        default='0.0.0.0',
        help='Host to bind to (default: 0.0.0.0)'
    )
    
    args = parser.parse_args()
    
    # Determine ROS2 mode (default: enabled if available)
    if args.no_ros2:
        ros2_enabled = False
        logger.info("ROS2 explicitly disabled")
    elif args.ros2:
        ros2_enabled = True
        if not ROS2_AVAILABLE:
            logger.error("ROS2 requested but not available!")
            return 1
    else:
        # Default: enable ROS2 if available
        ros2_enabled = ROS2_AVAILABLE
        if ros2_enabled:
            logger.info("ROS2 enabled by default (use --no-ros2 to disable)")
        else:
            logger.info("ROS2 not available - running in test mode")
    
    logger.info(f"Starting Orin Target API on {args.host}:{args.port}")
    logger.info(f"ROS2 publishing: {'enabled' if ros2_enabled else 'disabled'}")
    
    try:
        uvicorn.run(
            app,
            host=args.host,
            port=args.port,
            log_level="info"
        )
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
