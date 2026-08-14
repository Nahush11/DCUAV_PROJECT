import threading
from dronekit import connect, VehicleMode
import time
import os
import numpy as np
import pyvicon_datastream as pv
from pyproj import Geod

VICON_TRACKER_IP = "111.111.111.111"
OBJECT_NAME = "dynamics1"
HOVER_ALT = 1.0          # metres
HOVER_SECONDS = 15       # how long to hold before landing

vicon_client = pv.PyViconDatastream()
if vicon_client.connect(VICON_TRACKER_IP) != pv.Result.Success:
    raise RuntimeError("Vicon connect failed")
vicon_client.enable_segment_data()
vicon_client.set_stream_mode(pv.StreamMode.ServerPush)
vicon_client.set_axis_mapping(pv.Direction.Forward, pv.Direction.Left, pv.Direction.Up)
print("Vicon connected")

geoid = Geod(ellps='WGS84')
lab_lat = 53.834763521554876
lab_lon = 10.697331742076214

os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'

vehicle = connect("/dev/ttyACM0", baud=115200, wait_ready=True, timeout=80, rate=10)

@vehicle.on_message('STATUSTEXT')
def _st(self, name, message):
    print(f"[FC] {message.text}")

last_position = None
last_position_time = None


def pos_to_gps_coords(x, y):
    angle = np.arctan2(y, x)
    az = np.degrees(angle)
    if az < 0:
        az += 360
    dist = np.sqrt(x**2 + y**2)
    lon, lat, _ = geoid.fwd(lab_lon, lab_lat, az, dist)
    return lon, lat


def vicon_feed():
    global last_position, last_position_time
    while True:
        try:
            vicon_client.get_frame()
            position = vicon_client.get_segment_global_translation(OBJECT_NAME, OBJECT_NAME)
            if position is None:
                time.sleep(0.05)
                continue
            position = position * np.array([1/1000, -1/1000, 1/1000])
            rotation = vicon_client.get_segment_global_rotation_euler_xyz(OBJECT_NAME, OBJECT_NAME)
            if rotation is None:
                time.sleep(0.05)
                continue
            rotation = rotation * np.array([1, -1, -1])
            velocities = np.array([0.0, 0.0, 0.0])
            if last_position is not None and last_position_time is not None:
                dt = time.time() - last_position_time
                if dt > 0:
                    velocities = (position - last_position) / dt
            last_position = position
            last_position_time = time.time()
            rotation = rotation * 180 / np.pi
            for i in range(3):
                if rotation[i] > 180:
                    rotation[i] -= 360
                if rotation[i] < 0:
                    rotation[i] += 360
            lon, lat = pos_to_gps_coords(position[0], position[1])
            vehicle.send_mavlink(
                vehicle.message_factory.gps_input_encode(
                    0, 0, 0b00000000, 0, 0, 5,
                    int(lat * 1e7), int(lon * 1e7), position[2],
                    0.1, 0.1,
                    velocities[0], velocities[1], -velocities[2],
                    0.1, 0.1, 0.1, 25,
                    int(rotation[2] * 1e2),
                )
            )
            time.sleep(0.05)
        except Exception as e:
            print(f"vicon feed error: {e}")
            time.sleep(0.05)


threading.Thread(target=vicon_feed, daemon=True).start()
time.sleep(5)

try:
    print("Switching to GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)

    print("Pre-arm checks")
    while not vehicle.is_armable:
        print(f" waiting (ekf_ok={vehicle.ekf_ok}, gps_fix={vehicle.gps_0.fix_type})")
        time.sleep(1)

    print("Arming")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    print(f"Taking off to {HOVER_ALT} m")
    vehicle.simple_takeoff(HOVER_ALT)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" alt={alt:.2f}")
        if alt >= HOVER_ALT * 0.95:
            break
        time.sleep(0.3)

    print(f"HOVERING for {HOVER_SECONDS}s")
    for i in range(HOVER_SECONDS):
        alt = vehicle.location.global_relative_frame.alt
        print(f" hover {i+1}/{HOVER_SECONDS}  alt={alt:.2f}")
        time.sleep(1)

finally:
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(8)
    vehicle.close()
