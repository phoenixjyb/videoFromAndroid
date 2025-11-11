# CameraCommand.msg
# Request to control camera parameters

# Command type
string cmd

# For setZoomRatio
float32 zoom_ratio

# For switchCamera
string camera_facing  # "back" or "front"

# For setAeLock, setAwbLock
bool lock_value

# For setVideoProfile
int32 width
int32 height
int32 fps
bool high_speed

# For codec change
string codec  # "h264" or "h265"
