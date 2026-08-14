# PROPS-OFF EKF stability test. Does NOT arm, does NOT fly.
# Feeds Vicon with velocity ZEROED, then just watches whether ekf_ok
# stays True steadily instead of dropping out. This tests whether the
# injected velocity was the cause of the EKF variance problem.
import threading, time, os
import numpy as np
import pyvicon_datastream as pv
from pyproj import Geod
from dronekit import connect

VICON_TRACKER_IP = "111.111.111.111"
OBJECT_NAME = "dynamics1"

vc = pv.PyViconDatastream()
vc.connect(VICON_TRACKER_IP)
vc.enable_segment_data()
vc.set_stream_mode(pv.StreamMode.ServerPush)
vc.set_axis_mapping(pv.Direction.Forward, pv.Direction.Left, pv.Direction.Up)
print("Vicon connected")

geoid = Geod(ellps='WGS84')
lab_lat, lab_lon = 53.834763521554876, 10.697331742076214
os.environ['MAVLINK20'] = '1'
v = connect("/dev/ttyACM0", baud=115200, wait_ready=True, timeout=80, rate=10)

def pos_to_gps(x, y):
    a = np.degrees(np.arctan2(y, x))
    if a < 0: a += 360
    d = np.sqrt(x**2 + y**2)
    lon, lat, _ = geoid.fwd(lab_lon, lab_lat, a, d)
    return lon, lat

def feed():
    while True:
        try:
            vc.get_frame()
            p = vc.get_segment_global_translation(OBJECT_NAME, OBJECT_NAME)
            if p is None: time.sleep(0.05); continue
            p = p*np.array([1/1000,-1/1000,1/1000])
            r = vc.get_segment_global_rotation_euler_xyz(OBJECT_NAME, OBJECT_NAME)
            if r is None: time.sleep(0.05); continue
            r = r*np.array([1,-1,-1])*180/np.pi
            for i in range(3):
                if r[i]>180: r[i]-=360
                if r[i]<0: r[i]+=360
            lon, lat = pos_to_gps(p[0], p[1])
            # VELOCITY ZEROED (0,0,0) instead of computed frame-difference
            v.send_mavlink(v.message_factory.gps_input_encode(
                0,0,0b00000000,0,0,5,int(lat*1e7),int(lon*1e7),p[2],
                0.1,0.1, 0,0,0, 0.1,0.1,0.1,25,int(r[2]*1e2)))
            time.sleep(0.05)
        except Exception as e:
            print("feed err", e); time.sleep(0.05)

threading.Thread(target=feed, daemon=True).start()
time.sleep(5)

print("\nWatching EKF stability for 40s (NOT arming). Watch if ekf_ok stays True:")
ok_count = 0
for i in range(40):
    ok = v.ekf_ok
    if ok: ok_count += 1
    print(f" {i+1}/40  ekf_ok={ok}  gps_fix={v.gps_0.fix_type}  armable={v.is_armable}")
    time.sleep(1)

print(f"\nRESULT: ekf_ok was True {ok_count}/40 samples.")
print("If close to 40/40 with no drops, zeroing velocity fixed the variance issue.")
v.close()

