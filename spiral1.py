import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import os
import numpy as np
import pyvicon_datastream as pv
from pyproj import Geod

# ---------------- Config ----------------
VICON_TRACKER_IP = "111.111.111.111"
OBJECT_NAME = "dynamics1"

SPIRAL_RADIUS_M = 1.0
SPIRAL_START_ALT_M = 0.5
SPIRAL_END_ALT_M = 1.5
SPIRAL_WINDINGS = 3
SPIRAL_DURATION_S = 60.0
LOOKAHEAD_S = 1.5
SEND_RATE_HZ = 5.0
GROUNDSPEED = 0.4
SETTLE_S = 15          # EKF settle time before arming (from today's testing)

# ---------------- Vicon ----------------
vicon_client = pv.PyViconDatastream()
if vicon_client.connect(VICON_TRACKER_IP) != pv.Result.Success:
    raise RuntimeError(f"Vicon connect to {VICON_TRACKER_IP} failed")
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


def pos_to_gps_coords(x, y):
    angle = np.arctan2(y, x)
    az = np.degrees(angle)
    if az < 0:
        az += 360
    dist = np.sqrt(x**2 + y**2)
    lon, lat, _ = geoid.fwd(lab_lon, lab_lat, az, dist)
    return lon, lat


def spiral_target(t):
    """(x, y, z) lab-frame position along the spiral at time t (s)."""
    t = min(t, SPIRAL_DURATION_S)
    frac = t / SPIRAL_DURATION_S
    theta = frac * SPIRAL_WINDINGS * 2 * np.pi
    x = SPIRAL_RADIUS_M * np.cos(theta)
    y = SPIRAL_RADIUS_M * np.sin(theta)
    z = SPIRAL_START_ALT_M + frac * (SPIRAL_END_ALT_M - SPIRAL_START_ALT_M)
    return x, y, z


def vicon_feed():
    """Vicon -> GPS_INPUT. Velocity zeroed (today's confirmed EKF-stability fix)."""
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
            rotation = rotation * np.array([1, -1, -1]) * 180 / np.pi
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
                    0, 0, 0,               # velocity zeroed
                    0.1, 0.1, 0.1, 25,
                    int(rotation[2] * 1e2),
                )
            )
            time.sleep(0.05)
        except Exception as e:
            print(f"vicon feed error: {e}")
            time.sleep(0.05)


def arm_and_takeoff(target_alt):
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

    print(f"Taking off to {target_alt} m")
    vehicle.simple_takeoff(target_alt)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" alt={alt:.2f}")
        if alt >= target_alt * 0.95:
            print("Reached start altitude")
            break
        time.sleep(0.3)


def fly_spiral():
    print(f"Flying spiral: r={SPIRAL_RADIUS_M}m, {SPIRAL_START_ALT_M}->{SPIRAL_END_ALT_M}m, "
          f"{SPIRAL_WINDINGS} windings over {SPIRAL_DURATION_S}s")
    start_t = time.time()
    period = 1.0 / SEND_RATE_HZ
    while True:
        t = time.time() - start_t
        x, y, z = spiral_target(t + LOOKAHEAD_S)
        lon, lat = pos_to_gps_coords(x, y)
        vehicle.simple_goto(LocationGlobalRelative(lat, lon, z), groundspeed=GROUNDSPEED)
        if t >= SPIRAL_DURATION_S + LOOKAHEAD_S:
            print("Spiral complete")
            break
        time.sleep(period)


if __name__ == "__main__":
    threading.Thread(target=vicon_feed, daemon=True).start()
    time.sleep(SETTLE_S)

    try:
        arm_and_takeoff(SPIRAL_START_ALT_M)
        fly_spiral()
    finally:
        print("Landing")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(8)
        vehicle.close()
