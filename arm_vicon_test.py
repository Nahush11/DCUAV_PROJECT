import time
import threading
import os
import numpy as np

import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
import pyvicon_datastream as pv
from pyproj import Geod


# ============================================================
# CONFIG
# ============================================================

VICON_IP = "111.111.111.111"
OBJECT_NAME = "dynamics1"

FC_PORT = "/dev/ttyACM0"
FC_BAUD = 115200

LAB_LAT = 53.834763521554876
LAB_LON = 10.697331742076214


# ============================================================
# MAVLINK
# ============================================================

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "common"


# ============================================================
# VICON
# ============================================================

print("==========================================")
print(" VICON + PIXHAWK ARM TEST")
print("==========================================")

print("\nConnecting to Vicon...")

vicon = pv.PyViconDatastream()

ret = vicon.connect(VICON_IP)

if ret != pv.Result.Success:
    print("ERROR: Vicon connection failed")
    raise SystemExit

print("VICON CONNECTED")

vicon.enable_segment_data()
vicon.set_stream_mode(pv.StreamMode.ServerPush)
vicon.set_axis_mapping(
    pv.Direction.Forward,
    pv.Direction.Left,
    pv.Direction.Up
)

geod = Geod(ellps="WGS84")


# ============================================================
# PIXHAWK
# ============================================================

print("\nConnecting to Pixhawk...")

vehicle = connect(
    FC_PORT,
    baud=FC_BAUD,
    wait_ready=False,
    timeout=30
)

print("PIXHAWK CONNECTED")


# ============================================================
# FC MESSAGE LISTENER
# ============================================================

def fc_message_listener(vehicle, name, message):
    text = str(message)

    if "STATUSTEXT" in text:
        print("[FC]", text)


vehicle.add_message_listener(
    "STATUSTEXT",
    fc_message_listener
)


# ============================================================
# VICON -> GPS INPUT
# ============================================================

running = True


def vicon_loop():

    print("\nStarting Vicon GPS feed...")

    while running:

        try:

            result = vicon.get_frame()

            if result != pv.Result.Success:
                continue

            position = vicon.get_segment_global_translation(
                OBJECT_NAME,
                OBJECT_NAME
            )

            if position is None:
                print("[VICON] NO POSITION:", OBJECT_NAME)
                time.sleep(0.05)
                continue

            # Vicon mm -> meters
            # Your previous axis convention
            x = float(position[0]) / 1000.0
            y = -float(position[1]) / 1000.0
            z = float(position[2]) / 1000.0

            # ------------------------------------------------
            # Convert XY position to fake GPS
            # ------------------------------------------------

            azimuth = np.degrees(np.arctan2(y, x))

            if azimuth < 0:
                azimuth += 360

            distance = np.sqrt(x*x + y*y)

            lon, lat, _ = geod.fwd(
                LAB_LON,
                LAB_LAT,
                azimuth,
                distance
            )

            # ------------------------------------------------
            # Send GPS_INPUT
            # ------------------------------------------------

            vehicle.send_mavlink(
                vehicle.message_factory.gps_input_encode(

                    0,              # time_boot_ms
                    0,              # gps_id

                    0,              # ignore_flags

                    0,              # time_week_ms
                    0,              # time_week

                    5,              # fix_type

                    int(lat * 1e7),
                    int(lon * 1e7),

                    z,              # altitude

                    0.1,            # hdop
                    0.1,            # vdop

                    0.0,            # vn
                    0.0,            # ve
                    0.0,            # vd

                    0.1,            # speed_accuracy
                    0.1,            # horiz_accuracy
                    0.1,            # vert_accuracy

                    25,             # satellites

                    0               # yaw: don't provide yaw
                )
            )

            time.sleep(0.05)

        except Exception as e:

            print("[VICON ERROR]", e)

            time.sleep(0.1)


# ============================================================
# START VICON THREAD
# ============================================================

thread = threading.Thread(
    target=vicon_loop,
    daemon=True
)

thread.start()


# ============================================================
# WAIT FOR EKF
# ============================================================

print("\n==========================================")
print(" WAITING FOR EKF POSITION")
print("==========================================")

ekf_ready = False

for i in range(60):

    print(
        f"{i:02d}s | "
        f"MODE={vehicle.mode.name} | "
        f"ARMED={vehicle.armed} | "
        f"ARMABLE={vehicle.is_armable} | "
        f"EKF={vehicle.ekf_ok} | "
        f"GPS={vehicle.gps_0}"
    )

    if vehicle.ekf_ok and vehicle.is_armable:

        ekf_ready = True

        print("\n*** EKF AND ARMABLE ARE TRUE ***")

        break

    time.sleep(1)


# ============================================================
# STOP IF EKF NOT READY
# ============================================================

if not ekf_ready:

    print("\n==========================================")
    print("FAILED: EKF NEVER BECAME READY")
    print("==========================================")

    running = False
    vehicle.close()

    raise SystemExit


# ============================================================
# SET STABILIZE FIRST
# ============================================================

print("\n==========================================")
print("SETTING STABILIZE")
print("==========================================")

vehicle.mode = VehicleMode("STABILIZE")

for i in range(20):

    print(
        f"MODE WAIT {i+1}/20: "
        f"{vehicle.mode.name}"
    )

    if vehicle.mode.name == "STABILIZE":
        break

    time.sleep(0.5)


# ============================================================
# WAIT AGAIN
# ============================================================

print("\nWaiting for EKF/position after mode change...")

for i in range(20):

    print(
        f"{i:02d}s | "
        f"MODE={vehicle.mode.name} | "
        f"ARMABLE={vehicle.is_armable} | "
        f"EKF={vehicle.ekf_ok} | "
        f"GPS={vehicle.gps_0}"
    )

    if vehicle.ekf_ok and vehicle.is_armable:
        break

    time.sleep(1)


# ============================================================
# SET GUIDED
# ============================================================

print("\n==========================================")
print("SETTING GUIDED")
print("==========================================")

vehicle.mode = VehicleMode("GUIDED")

for i in range(20):

    print(
        f"GUIDED WAIT {i+1}/20: "
        f"MODE={vehicle.mode.name}"
    )

    if vehicle.mode.name == "GUIDED":
        break

    time.sleep(0.5)


# ============================================================
# CHECK GUIDED
# ============================================================

print("\n==========================================")
print("FINAL STATE BEFORE ARM")
print("==========================================")

print("MODE    :", vehicle.mode.name)
print("ARMED   :", vehicle.armed)
print("ARMABLE :", vehicle.is_armable)
print("EKF     :", vehicle.ekf_ok)
print("GPS     :", vehicle.gps_0)


if vehicle.mode.name != "GUIDED":

    print("\nFAILED: GUIDED MODE NOT AVAILABLE")

    print("This is the important result.")

    running = False
    vehicle.close()

    raise SystemExit


if not vehicle.is_armable:

    print("\nFAILED: VEHICLE IS NOT ARMABLE")

    running = False
    vehicle.close()

    raise SystemExit


# ============================================================
# ARM
# ============================================================

print("\n==========================================")
print("ARMING")
print("==========================================")

vehicle.armed = True

for i in range(20):

    print(
        f"ARM WAIT {i+1}/20 | "
        f"ARMED={vehicle.armed} | "
        f"ARMABLE={vehicle.is_armable}"
    )

    if vehicle.armed:

        print("\n==========================================")
        print(" SUCCESS: VEHICLE ARMED")
        print("==========================================")

        break

    time.sleep(0.5)


# ============================================================
# DO NOT TAKE OFF
# ============================================================

if vehicle.armed:

    print("\nARM TEST SUCCESSFUL.")
    print("NO TAKEOFF COMMAND WAS SENT.")
    print("Disarm from RC / Mission Planner when finished.")


else:

    print("\nARM FAILED.")


# ============================================================
# CLEANUP
# ============================================================

try:

    running = False

    time.sleep(1)

    vehicle.close()

except Exception:
    pass
