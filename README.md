# DCUAV — Autonomous Indoor Drone Flight with Vicon

Autonomous indoor quadcopter flight using a Vicon motion-capture system for
positioning — no GPS, no satellites, fully indoors. Built for the **Design and
Control of UAVs** university project.

## Project Overview

<!-- Add a short project showcase video here -->
<!-- Drag & drop your overview video below this line -->

---

## What it does

- **Stable autonomous hover** — takes off and holds position within ±5 cm
- **Upward spiral trajectory** — 1 m radius, climbing from 0.5 m to 1.5 m over
  3 windings, flown autonomously
- Uses a **Vicon motion-capture system** for precise indoor positioning.

---

# Hardware

## Drone Photos

| Front View | Side View |
|------------|-----------|
| ![Drone ](<img width="900" height="1600" alt="WhatsApp Image 2026-08-14 at 1 57 37 PM" src="https://github.com/user-attachments/assets/aef2f64f-c319-425d-b153-3a1bea401133" />
) | ![Drone](<img width="1200" height="1600" alt="IMG-20260814-WA0011" src="https://github.com/user-attachments/assets/d8438e4e-56fd-444d-9b04-c66b0b0319de" />
) |

**

### Components

- Raspberry Pi 5 (Companion Computer)
- Pixhawk 6C Flight Controller (ArduPilot)
- QAV250 Quadcopter Frame
- RadioMaster XR1 (ExpressLRS) Receiver
- Vicon Motion-Capture System

---

# Software Stack

- **DroneKit** + **pymavlink** — Communication with Pixhawk
- **pyvicon-datastream** — Live Vicon position streaming
- **pyproj** — Coordinate conversion for GPS injection
- Python 3 running in a virtual environment on Raspberry Pi

---

# System Architecture

```text
Vicon camera system
      │  (live x, y, z + orientation)
      ▼
Raspberry Pi 5 ──► Converts position into MAVLink GPS_INPUT
      │
      ▼ (USB)
Pixhawk 6C (ArduPilot)
      │ EKF fuses injected GPS data
      ▼
GUIDED Mode ──► Autonomous Hover / Spiral
```

The key idea is that ArduPilot's EKF treats the Vicon position as its GPS source
(`GPS_TYPE = 14`), enabling fully autonomous indoor flight with centimetre-level accuracy.

---

# Flight Demonstrations

## Autonomous Hover

**Video**

<!-- Drag & drop your hover video here -->

---

## Autonomous Spiral Flight

**Video**

<!-- Drag & drop your spiral flight video here -->

---

# Project Scripts

| File | Description |
|------|-------------|
| `hover1.py` | Arms the drone, takes off to 1 m, hovers, then lands |
| `spiral1.py` | Executes the complete autonomous upward spiral trajectory |

---

# Running the Project

```bash
cd ~/uav_project
source dronekit_env/bin/activate

python hover1.py      # Hover mission
python spiral1.py     # Spiral mission
```

The Pixhawk communicates over USB (`/dev/ttyACM0`), while the Vicon system streams pose data over the lab network.

---

## Project Gallery

You can add additional images or videos here, such as:

- Bench testing
- Vicon lab setup
- Flight controller wiring
- Raspberry Pi integration

---

*Design and Control of UAVs — University Project*
