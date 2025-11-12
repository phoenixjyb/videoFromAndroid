# Orin Media API

REST API server for browsing and downloading recorded media files from CamControl.

## Overview

The Media API provides HTTP endpoints for:
- Listing recorded videos/images
- Downloading media files
- Getting thumbnails (placeholder implementation)
- Deleting media files

Media files are stored in `saved_videos/` directory.

## Installation

```bash
# Install Python dependencies
pip install -r orin/requirements-media-api.txt
```

## Usage

### Start the server

```bash
# From project root
python3 orin/media_api.py
```

Server runs on: `http://0.0.0.0:8081`

### Configuration

Edit `orin/media_api.py` to change:
- `MEDIA_DIR`: Location of media files (default: `saved_videos/`)
- `BASE_URL`: Server base URL (default: `http://172.16.30.234:8081`)
- Port: Change in `uvicorn.run()` call (default: 8081)

## API Endpoints

### 1. Health Check
```http
GET /
```

Response:
```json
{
  "service": "Orin Media API",
  "version": "1.0.0",
  "status": "running",
  "media_directory": "/path/to/saved_videos",
  "media_count": 5
}
```

### 2. List Media Files
```http
GET /media/list?page=0&pageSize=50&type=video&sortBy=timestamp&sortOrder=descending
```

Query Parameters:
- `page` (int, default: 0): Page number (0-indexed)
- `pageSize` (int, default: 50, max: 200): Items per page
- `type` (string, optional): Filter by "video" or "image"
- `fromTimestamp` (int, optional): Filter from Unix timestamp (ms)
- `toTimestamp` (int, optional): Filter to Unix timestamp (ms)
- `sortBy` (string, default: "timestamp"): Sort field (timestamp, filename, size, duration)
- `sortOrder` (string, default: "descending"): "ascending" or "descending"

Response:
```json
{
  "items": [
    {
      "id": "abc123...",
      "filename": "20251108_133520_h265_1920x1080_30fps_5Mbps_5_0s-1762580117103.mp4",
      "type": "video",
      "timestamp": 1699454120000,
      "size": 3145728,
      "resolution": {"width": 1920, "height": 1080},
      "duration": 5,
      "codec": "h265",
      "fps": 30,
      "bitrate": 5000,
      "thumbnailUrl": "http://172.16.30.234:8081/media/abc123.../thumbnail",
      "downloadUrl": "http://172.16.30.234:8081/media/abc123.../download"
    }
  ],
  "total": 10,
  "page": 0,
  "pageSize": 50
}
```

### 3. Get Media Details
```http
GET /media/{media_id}
```

Response: Same as single item in list endpoint

### 4. Download Media File
```http
GET /media/{media_id}/download
```

Returns: Video file (Content-Type: video/mp4)

### 5. Get Thumbnail
```http
GET /media/{media_id}/thumbnail
```

Returns: JPEG thumbnail image

**Note**: Currently returns 404. Thumbnail generation with ffmpeg not yet implemented.

### 6. Delete Media File
```http
DELETE /media/{media_id}
```

Response:
```json
{
  "status": "success",
  "message": "Deleted filename.mp4"
}
```

## Filename Metadata Parsing

The API automatically parses metadata from CamControl's filename pattern:
```
YYYYMMDD_HHMMSS_codec_WxH_fps_bitrate_duration_timestamp.mp4
```

Example:
```
20251108_133520_h265_1920x1080_30fps_5Mbps_5_0s-1762580117103.mp4
```

Extracted fields:
- **Date/Time**: 2025-11-08 13:35:20
- **Codec**: h265
- **Resolution**: 1920x1080
- **FPS**: 30
- **Bitrate**: 5 Mbps (5000 kbps)
- **Duration**: 5 seconds

## Testing

### Using curl

```bash
# List all media
curl http://172.16.30.234:8081/media/list

# List first page of 10 items
curl "http://172.16.30.234:8081/media/list?page=0&pageSize=10"

# List videos only, sorted by size
curl "http://172.16.30.234:8081/media/list?type=video&sortBy=size&sortOrder=descending"

# Get media details
curl http://172.16.30.234:8081/media/abc123...

# Download media
curl -O http://172.16.30.234:8081/media/abc123.../download

# Delete media
curl -X DELETE http://172.16.30.234:8081/media/abc123...
```

### Using Python

```python
import requests

# List media
response = requests.get("http://172.16.30.234:8081/media/list")
media_list = response.json()

for item in media_list['items']:
    print(f"{item['filename']} - {item['size']} bytes")

# Download media
media_id = media_list['items'][0]['id']
response = requests.get(f"http://172.16.30.234:8081/media/{media_id}/download")

with open('downloaded.mp4', 'wb') as f:
    f.write(response.content)
```

## Integration with CamViewer

CamViewer Android app connects to this API:

1. **Configuration**: Set Orin Media URL in settings: `http://172.16.30.234:8081`
2. **Browse**: Media Gallery screen fetches list via `/media/list`
3. **Download**: Tap download button to fetch via `/media/{id}/download`
4. **View**: Downloaded files stored in app's Downloads directory

## Network Configuration

### Firewall (if enabled)
```bash
# Allow port 8081
sudo ufw allow 8081/tcp
```

### Test connectivity from CamViewer phone
```bash
# From Android device (via adb)
adb shell
curl http://172.16.30.234:8081/
```

## Troubleshooting

### Server won't start

**Check if port is in use:**
```bash
lsof -i :8081
netstat -tuln | grep 8081
```

**Kill existing process:**
```bash
kill $(lsof -t -i:8081)
```

### Can't connect from CamViewer

**Check Orin IP:**
```bash
ip addr show
```

**Verify server is listening:**
```bash
curl http://localhost:8081/
```

**Test from CamViewer phone:**
```bash
# On Mac (forwarding from phone to Orin)
ping 172.16.30.234
curl http://172.16.30.234:8081/
```

### No media files found

**Check media directory:**
```bash
ls -lh saved_videos/
```

**Verify permissions:**
```bash
chmod 755 saved_videos
chmod 644 saved_videos/*.mp4
```

## Future Enhancements

1. **Thumbnail Generation**: Use ffmpeg to generate video thumbnails
```python
# Generate thumbnail with ffmpeg
ffmpeg -i input.mp4 -ss 00:00:01 -vframes 1 thumbnail.jpg
```

2. **Streaming**: Add HLS/DASH streaming support for direct playback
3. **Metadata Extraction**: Use ffprobe for accurate video metadata
4. **Authentication**: Add API key or token-based auth
5. **Database**: Use SQLite for faster media indexing
6. **Search**: Full-text search on filenames and metadata

## API Specification

Full OpenAPI documentation available at:
```
http://172.16.30.234:8081/docs
```

Interactive API explorer (Swagger UI):
```
http://172.16.30.234:8081/redoc
```

## License

Part of CamControl project
