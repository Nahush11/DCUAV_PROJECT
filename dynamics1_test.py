#!/usr/bin/env python3
"""
dynamics1_final.py — Vicon -> PX4/ArduPilot fake-GPS, two-waypoint test flight.
Clean consolidated version. Run props-off first, with a safety pilot on the RC.

Run:
    cd ~/uav_project
    source dronekit_env/bin/activate
    python dynamics1_final.py
"""

# --- dronekit Python 3.13 compat shim (must be before importing dronekit) ---
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

import threading
import time
import os

import numpy as np
import pyvicon_datastream as pv
from pyproj import Geod
from dronekit import connect, VehicleMode, LocationGlobalRelative

# --- Config ---------------------------------------------------------------
VICON_TRACKER_IP = "111.111.111.111"
OBJECT_NAME = "dynamics1"
FC_CONNECTION = "/dev/ttyACM0"
FC_BAUD = 115200

LAB_LAT = 53.834763521554876
LAB_LON = 10.697331742076214

TAKEOFF_ALT = 1.5
POINT1_XY = (-1.2, 1.2)

os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'

geoid = Geod(ellps='WGS84')
last_position = None
last_position_time = None


def pos_to_gps_coords(x, y):
    angle = np.arctan2(y, x)
    az = np.degrees(angle)
    if az < 0:
        az += 360
    dist = np.sqrt(x ** 2 + y ** 2)
    lon, lat, return_az = geoid.fwd(LAB_LON, LAB_LAT, az, dist)
    return lon, lat, return_az


def vicon_to_fake_gps(vehicle, vicon_client):
    """Reads Vicon, injects position into the FC as GPS_INPUT. Per-iteration
    error handling so one bad frame never kills the whole feed thread."""
    global last_position, last_position_time
    while True:
        try:
            vicon_client.get_frame()
            position = vicon_client.get_segment_global_translation(OBJECT_NAME, OBJECT_NAME)
            if position is None:
                time.sleep(0.05)
                continue
            position = position * np.array([1 / 1000, -1 / 1000, 1 / 1000])
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

            longitude, latitude, _ = pos_to_gps_coords(position[0], position[1])

            vehicle.send_mavlink(
                vehicle.message_factory.gps_input_encode(
                    0, 0, 0b00000000, 0, 0, 5,
                    int(latitude * 1e7),
                    int(longitude * 1e7),
                    position[2],
                    0.1, 0.1,
                    velocities[0], velocities[1], -velocities[2],
                    0.1, 0.1, 0.1, 25,
                    int(rotation[2] * 1e2),
                )
            )
            time.sleep(0.05)
        except Exception as e:
            print(f"[vicon feed] iteration error: {e}")
            time.sleep(0.05)


def arm_and_takeoff(vehicle, target_altitude):
    print("Switching to GUIDED mode")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(" waiting for GUIDED...")
        time.sleep(1)

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(f" waiting to initialise (ekf_ok={vehicle.ekf_ok}, "
              f"gps_fix={vehicle.gps_0.fix_type})")
        time.sleep(1)

    print("Arming")
    vehicle.armed = True
    while not vehicle.armed:
        print(" waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" altitude: {alt:.3f}")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(0.3)


def goto_and_wait(vehicle, lon, lat, alt, label):
    vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt), groundspeed=0.5)
    while True:
        dist = geoid.inv(
            vehicle.location.global_relative_frame.lon,
            vehicle.location.global_relative_frame.lat,
            lon, lat,
        )[2]
        print(f" distance to {label}: {dist:.2f}")
        if dist < 0.2:
            print(f"Reached {label}")
            break
        time.sleep(0.2)


def main():
    print(f"Connecting to Vicon at {VICON_TRACKER_IP}...")
    vicon_client = pv.PyViconDatastream()
    if vicon_client.connect(VICON_TRACKER_IP) != pv.Result.Success:
        raise RuntimeError("Vicon connection failed")
    vicon_client.enable_segment_data()
    vicon_client.set_stream_mode(pv.StreamMode.ServerPush)
    vicon_client.set_axis_mapping(pv.Direction.Forward, pv.Direction.Left, pv.Direction.Up)
    print("Vicon connected.")

    print(f"Connecting to FC on {FC_CONNECTION}...")
    vehicle = connect(FC_CONNECTION, baud=FC_BAUD, wait_ready=True,
                      source_system=1, timeout=80, rate=10)

    @vehicle.on_message('STATUSTEXT')
    def statustext_listener(self, name, message):
        print(f"[FC] {message.text}")

    vicon_thread = threading.Thread(
        target=vicon_to_fake_gps, args=(vehicle, vicon_client), daemon=True)
    vicon_thread.start()

    print("Waiting 5s for EKF to initialise from Vicon feed...")
    time.sleep(5)

    try:
        arm_and_takeoff(vehicle, TAKEOFF_ALT)

        p1_lon, p1_lat, _ = pos_to_gps_coords(POINT1_XY[0], POINT1_XY[1])
        goto_and_wait(vehicle, p1_lon, p1_lat, 1.0, "point1")

        time.sleep(5)

        goto_and_wait(vehicle, LAB_LON, LAB_LAT, 0.5, "home")

    finally:
        print("Landing...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        vehicle.close()


if __name__ == "__main__":
    main()
