# CameraTelemetry.msg
# Camera status and telemetry data from Android

std_msgs/Header header

# Autofocus state
string af_state  # e.g., "PASSIVE_SCAN", "FOCUSED_LOCKED", etc.

# Auto exposure state
string ae_state  # e.g., "CONVERGED", "SEARCHING", etc.

# ISO sensitivity
int32 iso

# Exposure time in nanoseconds
int64 exposure_ns

# Current zoom ratio
float32 zoom_ratio

# Frame rate (approximate)
float32 fps

# Current camera facing
string camera_facing  # "back" or "front"

# Recording status
bool is_recording
