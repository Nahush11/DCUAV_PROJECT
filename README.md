# DCUAV — Autonomous Indoor Drone Flight with Vicon

Autonomous indoor quadcopter flight using a Vicon motion-capture system for
positioning — no GPS, no satellites, fully indoors. Built for the "Design and
Control of UAVs" university project.

<!-- DRAG YOUR BEST VIDEO HERE (hover or spiral). Just drop the file into
     this line while editing the README on GitHub and it embeds automatically. -->

---

## What it does

- **Stable autonomous hover** — takes off and holds position within ±5 cm
- **Upward spiral trajectory** — 1 m radius, climbing 0.5 m → 1.5 m over
  3 windings, flown autonomously
- All positioning comes from a **Vicon motion-capture system**, injected into
  the flight controller as if it were GPS — so the drone knows exactly where
  it is indoors.

## Hardware

- Raspberry Pi 5 (companion computer)
- Pixhawk 6C flight controller running ArduPilot
- QAV250 quadcopter frame
- RadioMaster XR1 (ExpressLRS) receiver
- Vicon motion-capture system

## Software stack

- **DroneKit** + **pymavlink** — talking to the flight controller
- **pyvicon-datastream** — reading live position from Vicon
- **pyproj** — converting lab coordinates to the GPS frame ArduPilot expects
- Python 3, running in a virtual environment on the Pi

## How it works

```
Vicon camera system
      │  (live x, y, z + orientation)
      ▼
Raspberry Pi 5  ──►  converts position into a MAVLink GPS_INPUT message
      │
      ▼  (USB)
Pixhawk 6C (ArduPilot)
      │  EKF fuses the injected position (GPS_TYPE = MAV)
      ▼
GUIDED mode  ──►  autonomous hover / spiral
```

The key idea: ArduPilot's EKF is told to treat the Vicon feed as its GPS
source (`GPS_TYPE = 14`), so all the normal autonomous-flight machinery works
indoors with centimetre-accurate motion-capture data instead of satellites.

## The scripts

| File | What it does |
|------|--------------|
| `hover1.py` | Arms, takes off to 1 m, holds a stable hover, lands |
| `spiral1.py` | Flies the full upward spiral trajectory |

## Running it

```bash
cd ~/uav_project
source dronekit_env/bin/activate
python hover1.py     # or spiral1.py
```

The Pixhawk connects over USB (`/dev/ttyACM0`); the Vicon feed streams over
the lab network.

## More footage

<!-- Drop additional clips here — build photos, the spiral from another angle,
     bench testing, etc. Each dragged-in video/image embeds inline. -->

---

*Design and Control of UAVs — university project.*
