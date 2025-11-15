#!/usr/bin/env python3
"""
Orin Service Control API
Provides REST API for remote management of Orin services

Endpoints:
- GET  /api/services/status  - Get service status
- POST /api/services/start   - Start all services
- POST /api/services/stop    - Stop all services
- GET  /api/services/logs    - Get recent logs

Usage:
    python3 service_control_api.py --port 8083
"""

import asyncio
import subprocess
import os
import sys
import socket
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
import argparse

from fastapi import FastAPI, HTTPException, Header
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# Get script directory
SCRIPT_DIR = Path(__file__).parent.absolute()

# Service Control PIN (set via environment variable or leave empty to disable)
SERVICE_PIN = os.environ.get("SERVICE_CONTROL_PIN", "")

# Automatically start ROS bridge unless explicitly disabled
AUTO_START_ROS_BRIDGE = os.environ.get("SERVICE_CONTROL_START_ROS_BRIDGE", "1")

# Service configuration
SERVICES = {
    "target_api": {
        "name": "Target API",
        "port": 8082,
        "log_file": SCRIPT_DIR / "target_api.log",
        "pid_file": SCRIPT_DIR / ".target_api.pid",
        "check": "http"
    },
    "media_api": {
        "name": "Media API",
        "port": 8081,
        "log_file": SCRIPT_DIR / "media_api.log",
        "pid_file": SCRIPT_DIR / ".media_api.pid",
        "check": "http"
    },
    "camera_relay": {
        "name": "Camera Relay",
        "port": None,  # No specific port, it's a ROS2 node
        "log_file": SCRIPT_DIR / "camera_relay.log",
        "pid_file": SCRIPT_DIR / ".camera_relay.pid",
        "check": "ros_node",
        "process_match": "camera_control_relay.py"
    },
    "ros_bridge": {
        "name": "ROS Bridge",
        "port": None,
        "log_file": SCRIPT_DIR / "ros_bridge.log",
        "pid_file": SCRIPT_DIR / ".ros_bridge.pid",
        "check": "ros_node",
        "process_match": "ros2_camcontrol.ws_to_image"
    }
}

app = FastAPI(
    title="Orin Service Control API",
    description="Remote control and monitoring for Orin services",
    version="1.0.0"
)

# Enable CORS for CamViewer
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ServiceStatus(BaseModel):
    """Service status model"""
    name: str
    running: bool
    pid: Optional[int]
    uptime_seconds: Optional[float]
    port: Optional[int]  # Optional for ROS2 nodes without HTTP ports
    last_log_lines: List[str] = []

class ServiceControlResponse(BaseModel):
    """Service control response"""
    success: bool
    message: str
    services: Dict[str, ServiceStatus]

def is_process_running(pid: int) -> bool:
    """Check if process is running"""
    try:
        os.kill(pid, 0)
        return True
    except (OSError, ProcessLookupError):
        return False

def is_port_listening(port: int) -> bool:
    """Check if a port is listening (for HTTP services)"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.5)
            result = sock.connect_ex(('localhost', port))
            return result == 0
    except Exception:
        return False

def process_matches(pid: int, expected_substring: Optional[str]) -> bool:
    """Check if the process command line contains the expected substring"""
    if not is_process_running(pid):
        return False
    if not expected_substring:
        return True
    try:
        with open(f'/proc/{pid}/cmdline', 'r') as f:
            cmdline = f.read()
            return expected_substring in cmdline
    except Exception:
        return True

def get_service_status(service_id: str, config: dict) -> ServiceStatus:
    """Get status of a single service"""
    pid = None
    running = False
    uptime = None
    last_logs = []
    
    # Check PID file
    check_type = config.get("check", "process")
    if config["pid_file"].exists():
        try:
            pid = int(config["pid_file"].read_text().strip())
            
            if check_type == "http":
                port = config.get("port")
                running = is_process_running(pid) and (port is None or is_port_listening(port))
            elif check_type == "ros_node":
                running = process_matches(pid, config.get("process_match"))
            else:
                running = is_process_running(pid)
            
            if not running:
                pid = None
        except (ValueError, IOError):
            pass
    
    # Get recent logs
    if config["log_file"].exists():
        try:
            with open(config["log_file"], 'r') as f:
                all_lines = f.readlines()
                last_logs = [line.strip() for line in all_lines[-10:]]
        except IOError:
            pass
    
    return ServiceStatus(
        name=config["name"],
        running=running,
        pid=pid,
        uptime_seconds=uptime,
        port=config["port"],
        last_log_lines=last_logs
    )

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "service": "Orin Service Control API",
        "version": "1.0.0",
        "endpoints": [
            "/api/services/status",
            "/api/services/start",
            "/api/services/stop",
        ]
    }

def verify_pin(x_service_pin: Optional[str] = None) -> bool:
    """Verify PIN if SERVICE_PIN is set"""
    if not SERVICE_PIN:
        return True  # No PIN required
    return x_service_pin == SERVICE_PIN

@app.get("/api/services/status", response_model=Dict[str, ServiceStatus])
async def get_status():
    """Get status of all services"""
    status = {}
    for service_id, config in SERVICES.items():
        status[service_id] = get_service_status(service_id, config)
    return status

@app.post("/api/services/start", response_model=ServiceControlResponse)
async def start_services(x_service_pin: Optional[str] = Header(None)):
    """Start all Orin services (PIN required if SERVICE_PIN env var set)"""
    if not verify_pin(x_service_pin):
        raise HTTPException(status_code=403, detail="Invalid or missing PIN")
    
    try:
        # Check if services are already running
        current_status = {}
        for service_id, config in SERVICES.items():
            current_status[service_id] = get_service_status(service_id, config)
        
        # If all running, return success immediately
        if all(svc.running for svc in current_status.values()):
            return ServiceControlResponse(
                success=True,
                message="All services are already running",
                services=current_status
            )
        
        # First, stop any running services to avoid port conflicts
        stop_script = SCRIPT_DIR / "stop_all_services.sh"
        if stop_script.exists():
            result = await asyncio.create_subprocess_exec(
                str(stop_script),
                cwd=str(SCRIPT_DIR),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            try:
                stdout, stderr = await asyncio.wait_for(result.communicate(), timeout=15.0)
                if result.returncode != 0:
                    print(f"Stop script warning: {stderr.decode()[:200]}")
            except asyncio.TimeoutError:
                result.kill()
                await result.wait()
            
            await asyncio.sleep(1)
        
        # Run start script completely detached
        # Use simple double-fork to prevent being part of service cgroup
        script_path = SCRIPT_DIR / "start_all_services.sh"
        if not script_path.exists():
            raise HTTPException(status_code=500, detail="Start script not found")
        
        # Execute script in background, fully detached
        # Use bash -c with setsid and background to detach completely
        bridge_env = "START_ROS_BRIDGE=1" if AUTO_START_ROS_BRIDGE not in ("0", "false", "False") else "START_ROS_BRIDGE=0"
        process = await asyncio.create_subprocess_shell(
            f"bash -c 'cd {SCRIPT_DIR} && {bridge_env} setsid ./start_all_services.sh </dev/null >/dev/null 2>&1 &'",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        # Wait for the shell command to complete (not the services)
        try:
            await asyncio.wait_for(process.wait(), timeout=2.0)
        except asyncio.TimeoutError:
            pass
        
        # Wait a short time for services to start
        await asyncio.sleep(5)
        
        # Get updated status
        services = {}
        for service_id, config in SERVICES.items():
            services[service_id] = get_service_status(service_id, config)
        
        all_running = all(svc.running for svc in services.values())
        
        return ServiceControlResponse(
            success=all_running,
            message="Services started successfully" if all_running else "Services are starting (may take a moment)",
            services=services
        )
        
    except HTTPException:
        raise
    except Exception as e:
        import traceback
        error_details = f"{type(e).__name__}: {str(e)}\n{traceback.format_exc()}"
        print(f"Error in start_services: {error_details}")
        raise HTTPException(status_code=500, detail=f"Failed to start services: {type(e).__name__}: {str(e) or 'Unknown error'}")

@app.post("/api/services/stop", response_model=ServiceControlResponse)
async def stop_services(x_service_pin: Optional[str] = Header(None)):
    """Stop all Orin services (PIN required if SERVICE_PIN env var set)"""
    if not verify_pin(x_service_pin):
        raise HTTPException(status_code=403, detail="Invalid or missing PIN")
    
    try:
        # Run stop script
        script_path = SCRIPT_DIR / "stop_all_services.sh"
        if not script_path.exists():
            raise HTTPException(status_code=500, detail="Stop script not found")
        
        result = subprocess.run(
            [str(script_path)],
            cwd=str(SCRIPT_DIR),
            capture_output=True,
            text=True,
            timeout=10
        )
        
        # Wait a moment for services to stop
        await asyncio.sleep(1)
        
        # Get updated status
        services = {}
        for service_id, config in SERVICES.items():
            services[service_id] = get_service_status(service_id, config)
        
        all_stopped = all(not svc.running for svc in services.values())
        
        return ServiceControlResponse(
            success=all_stopped,
            message="Services stopped successfully" if all_stopped else "Some services may still be running",
            services=services
        )
        
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="Stop script timed out")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop services: {str(e)}")

@app.get("/api/services/logs/{service_id}")
async def get_logs(service_id: str, lines: int = 50):
    """Get recent logs for a service"""
    if service_id not in SERVICES:
        raise HTTPException(status_code=404, detail="Service not found")
    
    config = SERVICES[service_id]
    if not config["log_file"].exists():
        return {"lines": []}
    
    try:
        with open(config["log_file"], 'r') as f:
            all_lines = f.readlines()
            recent_lines = [line.strip() for line in all_lines[-lines:]]
        return {"lines": recent_lines}
    except IOError as e:
        raise HTTPException(status_code=500, detail=f"Failed to read logs: {str(e)}")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Orin Service Control API")
    parser.add_argument("--port", type=int, default=8083, help="Port to listen on")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    args = parser.parse_args()
    
    print("=" * 60)
    print("Orin Service Control API")
    print("=" * 60)
    print(f"Listening on: http://{args.host}:{args.port}")
    print(f"API Docs: http://{args.host}:{args.port}/docs")
    print()
    print("Endpoints:")
    print(f"  GET  /api/services/status")
    print(f"  POST /api/services/start")
    print(f"  POST /api/services/stop")
    print(f"  GET  /api/services/logs/{{service_id}}")
    print("=" * 60)
    
    uvicorn.run(app, host=args.host, port=args.port)

if __name__ == "__main__":
    main()
