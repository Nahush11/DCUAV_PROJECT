import time
import numpy as np
import pyvicon_datastream as pv
from pyproj import Geod
from dronekit import connect

VICON_IP = "111.111.111.111"
OBJECT_NAME = "dynamics1"

LAB_LAT = 53.834763521554876
LAB_LON = 10.697331742076214

print("========================================")
print(" VICON + PIXHAWK DIAGNOSTIC")
print("========================================")

# --------------------------------------------------
# VICON
# --------------------------------------------------

print("\nConnecting to Vicon...")

vicon = pv.PyViconDatastream()
ret = vicon.connect(VICON_IP)

if ret != pv.Result.Success:
    print("VICON CONNECTION FAILED")
    raise SystemExit

print("VICON CONNECTED")

vicon.enable_segment_data()
vicon.set_stream_mode(pv.StreamMode.ServerPush)
vicon.set_axis_mapping(
    pv.Direction.Forward,
    pv.Direction.Left,
    pv.Direction.Up
)

# --------------------------------------------------
# PIXHAWK
# --------------------------------------------------

print("\nConnecting to Pixhawk...")

vehicle = connect(
    "/dev/ttyACM0",
    baud=115200,
    wait_ready=False,
    timeout=30
)

print("PIXHAWK CONNECTED")

geod = Geod(ellps="WGS84")

# --------------------------------------------------
# FC messages
# --------------------------------------------------

def fc_message(vehicle, name, message):
    print("[FC]", message)

vehicle.add_message_listener(
    "STATUSTEXT",
    fc_message
)

# --------------------------------------------------
# VICON -> GPS
# --------------------------------------------------

print("\nStarting Vicon position test...")
print("OBJECT =", OBJECT_NAME)
print()

last_time = time.time()

for i in range(120):

    # Get Vicon frame
    result = vicon.get_frame()

    try:
        position = vicon.get_segment_global_translation(
            OBJECT_NAME,
            OBJECT_NAME
        )
    except Exception as e:
        print("VICON READ ERROR:", e)
        time.sleep(0.1)
        continue

    if position is None:
        print("NO VICON POSITION")
        time.sleep(0.1)
        continue

    # Vicon is mm -> meters
    position = np.array(position, dtype=float)

    x = position[0] / 1000.0
    y = -position[1] / 1000.0
    z = position[2] / 1000.0

    # Convert local XY -> GPS
    distance = np.sqrt(x*x + y*y)

    if distance < 0.001:
        azimuth = 0.0
    else:
        azimuth = np.degrees(np.arctan2(y, x))

    if azimuth < 0:
        azimuth += 360.0

    lon, lat, back_az = geod.fwd(
        LAB_LON,
        LAB_LAT,
        azimuth,
        distance
    )

    # --------------------------------------------------
    # Print everything
    # --------------------------------------------------

    print(
        f"{i:03d} | "
        f"VICON raw=({position[0]:.1f}, "
        f"{position[1]:.1f}, "
        f"{position[2]:.1f}) mm | "
        f"XYZ=({x:.3f}, {y:.3f}, {z:.3f}) m | "
        f"dist={distance:.3f} m | "
        f"az={azimuth:.1f} | "
        f"GPS=({lat:.7f}, {lon:.7f})"
    )

    # --------------------------------------------------
    # Send GPS_INPUT
    # --------------------------------------------------

    vehicle.send_mavlink(
        vehicle.message_factory.gps_input_encode(
            int(time.time() * 1000) & 0xFFFFFFFF,
            0,
            0,
            0,
            0,
            3,                  # FIX TYPE = 3D FIX
            int(lat * 1e7),
            int(lon * 1e7),
            0.0,                # altitude MSL
            0.5,                # HDOP
            0.5,                # VDOP
            0.0,                # VN
            0.0,                # VE
            0.0,                # VD
            0.5,                # speed accuracy
            0.5,                # horizontal accuracy
            0.5,                # vertical accuracy
            25,
            
        )
    )

    # --------------------------------------------------
    # FC state
    # --------------------------------------------------

    print(
        f"     FC: "
        f"GPS={vehicle.gps_0} | "
        f"EKF={vehicle.ekf_ok} | "
        f"ARMABLE={vehicle.is_armable} | "
        f"MODE={vehicle.mode.name}"
    )

    time.sleep(0.2)

print("\nTEST FINISHED")

vehicle.close()
vicon.disconnect()

