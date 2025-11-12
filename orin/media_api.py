#!/usr/bin/env python3
"""
Orin Media API Server
Provides HTTP REST API for browsing and downloading recorded media files

Usage:
    python3 orin/media_api.py

Endpoints:
    GET /media/list?page=0&pageSize=50&type=video&sortBy=timestamp&sortOrder=descending
    GET /media/{media_id}
    GET /media/{media_id}/download
    GET /media/{media_id}/thumbnail
    DELETE /media/{media_id}

The server scans the saved_videos/ directory for recorded media files.
"""

import os
import sys
import hashlib
import mimetypes
from datetime import datetime
from pathlib import Path
from typing import List, Optional
from dataclasses import dataclass, asdict

from fastapi import FastAPI, HTTPException, Query, Response
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

app = FastAPI(
    title="Orin Media API",
    description="REST API for browsing and downloading recorded media",
    version="1.0.0"
)

# CORS middleware for cross-origin requests from Android app
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins in development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration
# Use absolute path to project root's saved_videos directory
PROJECT_ROOT = Path(__file__).parent.parent
MEDIA_DIR = PROJECT_ROOT / "saved_videos"
THUMBNAIL_CACHE_DIR = MEDIA_DIR / ".thumbnails"
BASE_URL = "http://172.16.30.234:8081"  # Orin IP

# Ensure directories exist
MEDIA_DIR.mkdir(parents=True, exist_ok=True)
THUMBNAIL_CACHE_DIR.mkdir(parents=True, exist_ok=True)


@dataclass
class MediaItem:
    """Represents a media file"""
    id: str
    filename: str
    type: str  # "VIDEO" or "IMAGE" (uppercase to match Android enum)
    timestamp: int  # Unix timestamp in milliseconds
    size: int  # File size in bytes
    resolution: Optional[dict] = None  # {"width": 1920, "height": 1080}
    duration: Optional[int] = None  # Duration in seconds (for videos)
    codec: Optional[str] = None  # "h264" or "h265"
    fps: Optional[int] = None  # Frame rate
    bitrate: Optional[int] = None  # Bitrate in kbps
    thumbnailUrl: Optional[str] = None
    downloadUrl: str = ""


def get_file_id(filepath: Path) -> str:
    """Generate a unique ID for a file based on its path"""
    return hashlib.md5(str(filepath).encode()).hexdigest()


def parse_filename_metadata(filename: str) -> dict:
    """
    Parse metadata from filename pattern:
    20251108_133520_h265_1920x1080_30fps_5Mbps_5_0s-1762580117103.mp4
    
    Returns dict with extracted metadata
    """
    metadata = {}
    
    try:
        # Split by underscore
        parts = filename.split('_')
        
        # Date and time
        if len(parts) > 1:
            date_str = parts[0]
            time_str = parts[1]
            dt_str = f"{date_str}{time_str}"
            dt = datetime.strptime(dt_str, "%Y%m%d%H%M%S")
            metadata['timestamp'] = int(dt.timestamp() * 1000)
        
        # Codec
        if len(parts) > 2:
            metadata['codec'] = parts[2]  # h264 or h265
        
        # Resolution
        if len(parts) > 3:
            res_str = parts[3]
            if 'x' in res_str:
                w, h = res_str.split('x')
                metadata['resolution'] = {"width": int(w), "height": int(h)}
        
        # FPS
        if len(parts) > 4:
            fps_str = parts[4].replace('fps', '')
            metadata['fps'] = int(fps_str)
        
        # Bitrate
        if len(parts) > 5:
            bitrate_str = parts[5].replace('Mbps', '')
            metadata['bitrate'] = int(float(bitrate_str) * 1000)  # Convert to kbps
        
        # Duration
        if len(parts) > 6:
            duration_str = parts[6].replace('s', '').split('-')[0]
            metadata['duration'] = int(float(duration_str))
            
    except Exception as e:
        print(f"Warning: Failed to parse filename metadata: {e}")
    
    return metadata


def scan_media_files() -> List[MediaItem]:
    """
    Scan media directory and return list of media items
    """
    media_items = []
    
    if not MEDIA_DIR.exists():
        return media_items
    
    for filepath in MEDIA_DIR.glob("*.mp4"):
        if filepath.name.startswith('.'):
            continue  # Skip hidden files
        
        file_stat = filepath.stat()
        file_id = get_file_id(filepath)
        
        # Parse metadata from filename
        metadata = parse_filename_metadata(filepath.name)
        
        media_item = MediaItem(
            id=file_id,
            filename=filepath.name,
            type="VIDEO",  # Use uppercase to match Android MediaType enum
            timestamp=metadata.get('timestamp', int(file_stat.st_mtime * 1000)),
            size=file_stat.st_size,
            resolution=metadata.get('resolution'),
            duration=metadata.get('duration'),
            codec=metadata.get('codec'),
            fps=metadata.get('fps'),
            bitrate=metadata.get('bitrate'),
            thumbnailUrl=f"{BASE_URL}/media/{file_id}/thumbnail",
            downloadUrl=f"{BASE_URL}/media/{file_id}/download"
        )
        
        media_items.append(media_item)
    
    return media_items


def get_media_by_id(media_id: str) -> Optional[tuple[MediaItem, Path]]:
    """Get media item and its file path by ID"""
    media_items = scan_media_files()
    for item in media_items:
        if item.id == media_id:
            filepath = MEDIA_DIR / item.filename
            return item, filepath
    return None


@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "service": "Orin Media API",
        "version": "1.0.0",
        "status": "running",
        "media_directory": str(MEDIA_DIR.absolute()),
        "media_count": len(scan_media_files())
    }


@app.get("/media/list")
async def list_media(
    page: int = Query(0, ge=0, description="Page number (0-indexed)"),
    pageSize: int = Query(50, ge=1, le=200, description="Items per page"),
    type: Optional[str] = Query(None, description="Filter by type: video or image"),
    fromTimestamp: Optional[int] = Query(None, description="Filter from timestamp (ms)"),
    toTimestamp: Optional[int] = Query(None, description="Filter to timestamp (ms)"),
    sortBy: str = Query("timestamp", description="Sort field: timestamp, filename, size, duration"),
    sortOrder: str = Query("descending", description="Sort order: ascending or descending")
):
    """
    List all media files with pagination and filtering
    """
    # Scan all media files
    all_items = scan_media_files()
    
    # Apply filters
    filtered_items = all_items
    
    if type:
        filtered_items = [item for item in filtered_items if item.type == type.lower()]
    
    if fromTimestamp:
        filtered_items = [item for item in filtered_items if item.timestamp >= fromTimestamp]
    
    if toTimestamp:
        filtered_items = [item for item in filtered_items if item.timestamp <= toTimestamp]
    
    # Sort
    reverse = (sortOrder.lower() == "descending")
    sort_key = sortBy.lower()
    
    if sort_key == "timestamp":
        filtered_items.sort(key=lambda x: x.timestamp, reverse=reverse)
    elif sort_key == "filename":
        filtered_items.sort(key=lambda x: x.filename, reverse=reverse)
    elif sort_key == "size":
        filtered_items.sort(key=lambda x: x.size, reverse=reverse)
    elif sort_key == "duration":
        filtered_items.sort(key=lambda x: x.duration or 0, reverse=reverse)
    
    # Paginate
    start = page * pageSize
    end = start + pageSize
    page_items = filtered_items[start:end]
    
    return {
        "items": [asdict(item) for item in page_items],
        "total": len(filtered_items),
        "page": page,
        "pageSize": pageSize
    }


@app.get("/media/{media_id}")
async def get_media_details(media_id: str):
    """
    Get details for a specific media item
    """
    result = get_media_by_id(media_id)
    if not result:
        raise HTTPException(status_code=404, detail="Media not found")
    
    media_item, _ = result
    return asdict(media_item)


@app.get("/media/{media_id}/download")
async def download_media(media_id: str):
    """
    Download a media file
    """
    result = get_media_by_id(media_id)
    if not result:
        raise HTTPException(status_code=404, detail="Media not found")
    
    media_item, filepath = result
    
    if not filepath.exists():
        raise HTTPException(status_code=404, detail="Media file not found on disk")
    
    return FileResponse(
        path=filepath,
        filename=media_item.filename,
        media_type="video/mp4"
    )


@app.get("/media/{media_id}/thumbnail")
async def get_thumbnail(media_id: str):
    """
    Get thumbnail for a media item
    Currently returns a placeholder since ffmpeg is not used
    TODO: Generate thumbnails using ffmpeg
    """
    result = get_media_by_id(media_id)
    if not result:
        raise HTTPException(status_code=404, detail="Media not found")
    
    # Check if thumbnail exists in cache
    thumbnail_path = THUMBNAIL_CACHE_DIR / f"{media_id}.jpg"
    
    if thumbnail_path.exists():
        return FileResponse(path=thumbnail_path, media_type="image/jpeg")
    
    # TODO: Generate thumbnail using ffmpeg if not cached
    # For now, return 404 to indicate no thumbnail available
    raise HTTPException(status_code=404, detail="Thumbnail not available")


@app.delete("/media/{media_id}")
async def delete_media(media_id: str):
    """
    Delete a media file
    """
    result = get_media_by_id(media_id)
    if not result:
        raise HTTPException(status_code=404, detail="Media not found")
    
    media_item, filepath = result
    
    if filepath.exists():
        filepath.unlink()
        print(f"Deleted media file: {filepath}")
    
    # Also delete thumbnail if exists
    thumbnail_path = THUMBNAIL_CACHE_DIR / f"{media_id}.jpg"
    if thumbnail_path.exists():
        thumbnail_path.unlink()
    
    return {"status": "success", "message": f"Deleted {media_item.filename}"}


if __name__ == "__main__":
    print("=" * 60)
    print("Orin Media API Server")
    print("=" * 60)
    print(f"Media directory: {MEDIA_DIR.absolute()}")
    print(f"Media files found: {len(scan_media_files())}")
    print(f"Server URL: {BASE_URL}")
    print("=" * 60)
    print("\nStarting server on 0.0.0.0:8081...")
    print("Press Ctrl+C to stop\n")
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8081,
        log_level="info"
    )
