#!/usr/bin/env python3
"""Set the EKF3 / AHRS origin for GPS-DENIED external-nav flight.

WHY THIS IS NEEDED (traced through AP_DDS):
  /ap/pose/filtered is written by AP_DDS_Client::update_topic() ONLY when
  ahrs.get_relative_position_NED_home() returns true. That function (AP_AHRS.cpp)
  requires BOTH _home_is_set AND get_location() to succeed. get_location() needs
  an EKF ORIGIN. With no GPS, nothing sets the origin automatically, so
  get_location() returns false -> the position field is never written ->
  /ap/pose/filtered is frozen at (0,0,0) no matter what /ap/tf publishes.

  This sends MAVLink SET_GPS_GLOBAL_ORIGIN, which gives EKF3 an absolute origin.
  After that, get_location() works; once home is set (auto when the EKF has a
  valid position, and guaranteed at arm) /ap/pose/filtered reports live pose.

NOTE: the origin is NOT persistent — re-run this after every FC reboot
(i.e. after set_extnav_params.py reboots the controller).

  python3 set_origin.py

This only sets the origin. It does not arm or fly.
"""

import sys
import time

MAVLINK_ENDPOINT = "udp:127.0.0.1:14552"   # MAVProxy: output add 127.0.0.1:14552

# Origin location. Defaults to the SITL/CMAC home used by the Gazebo worlds
# (mars.sdf <spherical_coordinates>), so absolute lat/lon line up with the sim.
# The exact value does not matter for relative nav — it just anchors the frame.
ORIGIN_LAT_DEG = -35.3632621
ORIGIN_LON_DEG = 149.1652374
ORIGIN_ALT_M = 10.0


def main():
    try:
        from pymavlink import mavutil
    except ImportError:
        print("ERROR: pymavlink not installed (`pip install pymavlink`).", file=sys.stderr)
        sys.exit(1)

    print(f"Connecting to {MAVLINK_ENDPOINT} ...")
    master = mavutil.mavlink_connection(MAVLINK_ENDPOINT)
    if master.wait_heartbeat(timeout=10) is None:
        print(f"ERROR: no heartbeat on {MAVLINK_ENDPOINT} within 10 s. "
              "Is SITL up and did you 'output add 127.0.0.1:14552' in MAVProxy?",
              file=sys.stderr)
        sys.exit(1)
    print(f"Heartbeat from system {master.target_system}.")

    lat_e7 = int(round(ORIGIN_LAT_DEG * 1e7))
    lon_e7 = int(round(ORIGIN_LON_DEG * 1e7))
    alt_mm = int(round(ORIGIN_ALT_M * 1e3))

    print(f"Sending SET_GPS_GLOBAL_ORIGIN: lat={ORIGIN_LAT_DEG}, "
          f"lon={ORIGIN_LON_DEG}, alt={ORIGIN_ALT_M} m ...")
    # Send a few times in case the EKF is still booting and ignores early ones.
    for _ in range(5):
        try:
            master.mav.set_gps_global_origin_send(
                master.target_system, lat_e7, lon_e7, alt_mm, 0)
        except TypeError:
            # older dialects without the time_usec field
            master.mav.set_gps_global_origin_send(
                master.target_system, lat_e7, lon_e7, alt_mm)
        time.sleep(0.5)

    # Verify: ask for GPS_GLOBAL_ORIGIN (msg 49) and confirm the FC echoes it.
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        49, 0, 0, 0, 0, 0, 0)

    got = None
    deadline = time.time() + 5.0
    while time.time() < deadline:
        msg = master.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=5.0)
        if msg is not None:
            got = msg
            break

    if got is not None:
        print("\n[OK] EKF origin is set. FC reports GPS_GLOBAL_ORIGIN:")
        print(f"     lat={got.latitude * 1e-7:.7f}, lon={got.longitude * 1e-7:.7f}, "
              f"alt={got.altitude * 1e-3:.1f} m")
        print("\nget_location() will now succeed. /ap/pose/filtered will report "
              "live pose once home is set (auto when the EKF has a valid position, "
              "guaranteed at arm).")
    else:
        print("\n[WARN] No GPS_GLOBAL_ORIGIN echo received. The origin may still "
              "have been accepted (some builds don't echo). Check "
              "`ros2 topic echo /ap/pose/filtered` after arming; if still 0,0,0, "
              "re-run this and confirm the EKF has booted.", file=sys.stderr)


if __name__ == "__main__":
    main()
