# Device Information

Hardware capabilities and specifications for tested devices.

## Files

### `sms9160Capability.txt`
Camera capabilities for device SMS9160:
- Supported resolutions
- Frame rate ranges
- Camera features
- Hardware encoding capabilities

### `sms9280Capability.txt`
Camera capabilities for device SMS9280:
- Supported resolutions
- Frame rate ranges  
- Camera features
- Hardware encoding capabilities

### `systemChart.txt`
System architecture and component diagram:
- Data flow between Android and Orin
- Network topology
- Component relationships

## Usage

These files document the hardware capabilities used for:
- Validating supported video profiles
- Understanding device-specific limitations
- Debugging camera configuration issues
- Planning feature implementations

## Adding New Devices

To document a new device:
1. Run `adb shell dumpsys media.camera` on the connected device
2. Save output to `device_info/<MODEL>Capability.txt`
3. Update this README with device model and key features
