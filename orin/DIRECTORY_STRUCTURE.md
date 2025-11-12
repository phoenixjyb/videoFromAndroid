# Directory Structure - Camera Control on Orin

## File Organization

```
camControl/                              ← Root of repository
└── orin/                                ← Orin deployment folder (YOU WORK HERE)
    ├── .venv/                           ← Python virtual environment
    │   ├── bin/
    │   │   ├── activate                 ← source .venv/bin/activate
    │   │   └── python3
    │   └── lib/python3.x/site-packages/ ← websockets, fastapi installed here
    │
    ├── requirements.txt                 ← Python dependencies
    │
    ├── camera_control_relay.py          ← NEW: ROS2 → Phone relay
    ├── setup_camera_relay.sh            ← NEW: One-time setup
    ├── start_camera_relay.sh            ← NEW: Start relay
    ├── QUICKSTART.md                    ← NEW: Quick reference
    ├── SETUP_GUIDE.md                   ← NEW: Detailed guide
    └── CAMERA_CONTROL_RELAY_README.md   ← NEW: API docs
```

## Important Path Rules

### ✅ ALWAYS work from `orin/` directory

```bash
cd /path/to/camControl/orin    # ← You must be HERE
source .venv/bin/activate       # ← Activates ./venv (which is orin/.venv)
python3 camera_control_relay.py # ← Runs from orin/, finds packages in orin/.venv
```

### Virtual Environment Location

- **Full path**: `/path/to/camControl/orin/.venv/`
- **When in orin/ folder, use**: `.venv/`
- **Created by**: Running `./setup_camera_relay.sh` from `orin/` folder

## Quick Commands (Copy-Paste)

### First Time Setup
```bash
cd /path/to/camControl/orin
./setup_camera_relay.sh
```

### Daily Usage
```bash
cd /path/to/camControl/orin
./start_camera_relay.sh 172.16.30.28
```

### Manual Start
```bash
cd /path/to/camControl/orin
source .venv/bin/activate
python3 camera_control_relay.py --phone-host 172.16.30.28
```

## Verify Setup

```bash
cd /path/to/camControl/orin
ls -la .venv/                    # Should exist
source .venv/bin/activate
which python3                    # Should show: .../orin/.venv/bin/python3
pip list | grep websockets       # Should show: websockets 12.x
```

## See Also

- `QUICKSTART.md` - Quick reference
- `SETUP_GUIDE.md` - Detailed setup instructions
- `CAMERA_CONTROL_RELAY_README.md` - API documentation
