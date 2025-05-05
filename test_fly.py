from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time, threading
import pyproj

TARGET_COORDINATES = 50.443326, 30.448078

ged = pyproj.Geod(ellps='WGS84')


# ——— helper fns ———

def get_distance_and_azimuth(aLocation1, aLocation2):
    """
    Returns the geodesic distance (m) and forward azimuth (°) from aLocation1 to aLocation2.
    """
    fwd_az, back_az, dist = ged.inv(
        aLocation1.lon, aLocation1.lat,
        aLocation2.lon, aLocation2.lat
    )
    bearing = fwd_az % 360
    return dist, bearing


def alt_hold_pid(vehicle, target_alt, kp=0.8):
    current_alt = vehicle.location.global_relative_frame.alt
    error = target_alt - current_alt
    pwm = int(1500 + kp * error * 100)
    pwm = max(1000, min(2000, pwm))
    vehicle.channels.overrides['3'] = pwm


def altitude_hold_loop(vehicle, target_alt, stop_event):
    while not stop_event.is_set():
        alt_hold_pid(vehicle, target_alt)
        time.sleep(0.1)


def rotate_to_heading_alt_hold(vehicle, target_heading, threshold=2, pwm_rate=1600):
    """
    Yaws in ALT_HOLD by RC override on channel 4 until
    vehicle.heading is within ±threshold of target_heading.
    pwm_rate: e.g. 1600µs for moderate yaw speed.
    """

    def shortest_dir(current, target):
        diff = (target - current + 360) % 360
        return 1 if diff <= 180 else -1

    while True:
        current = vehicle.heading
        delta = abs((current - target_heading + 180) % 360 - 180)
        if delta <= threshold:
            print(f"→ Yaw reached: {current:.1f}° ±{threshold}°")
            break

        direction = shortest_dir(current, target_heading)
        # direction=1 → CW → ch4 > 1500; direction=-1 → CCW → ch4 < 1500
        pwm = 1500 + direction * (pwm_rate - 1500)
        vehicle.channels.overrides['4'] = pwm

        print(f"Yawing: current {current:.1f}°, target {target_heading}°, delta {delta:.1f}°")
        time.sleep(0.1)

    # center yaw stick
    vehicle.channels.overrides['4'] = 1500


def move_forward_with_course_correction(vehicle, dest_lat, dest_lon,
                                        tolerance=1.0,
                                        pwm_forward=1300,
                                        roll_gain=0.3):
    """
    Moves forward by applying constant pitch (channel 2 override) and
    corrects course by adjusting roll (channel 1 override) based on
    bearing error. Continues until within tolerance (m) of target.
    """
    target = LocationGlobalRelative(dest_lat, dest_lon, 0)
    while True:
        loc = vehicle.location.global_relative_frame
        dist, bearing = get_distance_and_azimuth(loc, target)
        heading = vehicle.heading
        # compute shortest angular difference
        error = (bearing - heading + 540) % 360 - 180

        print(f"Distance: {dist:.2f} m | Bearing error: {error:.1f}°")
        if dist <= tolerance:
            print(f"→ Arrived at target (±{tolerance} m)")
            break

        # forward pitch
        vehicle.channels.overrides['2'] = pwm_forward
        # roll correction: map error to roll stick
        roll_offset = int(1500 + roll_gain * error * 10)
        roll_offset = max(1000, min(2000, roll_offset))
        vehicle.channels.overrides['1'] = roll_offset

        time.sleep(0.1)
    # center sticks
    vehicle.channels.overrides['2'] = 1500
    vehicle.channels.overrides['1'] = 1500


def condition_yaw(vehicle, heading, yaw_rate=10, direction=1, relative=False):
    is_rel = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading, yaw_rate, direction, is_rel,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)


def wait_for_yaw(vehicle, target_heading, threshold=2):
    while True:
        current = vehicle.heading
        delta = abs((current - target_heading + 180) % 360 - 180)
        if delta <= threshold:
            print(f"→ Yaw reached: {current:.1f}° ±{threshold}°")
            break
        print(f"→ Rotating... Current: {current:.1f}° | Target: {target_heading:.1f}° | Δ: {delta:.1f}°")
        time.sleep(0.2)


def arm_and_takeoff(vehicle, target_alt):
    print("Arming motors…")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)
    print(f"Taking off to {target_alt} m…")
    vehicle.simple_takeoff(target_alt)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Alt = {alt:.1f} m")
        if alt >= target_alt - 1.0:
            print("Reached target altitude.")
            break
        time.sleep(0.5)


def land_and_disarm(vehicle, stop_event):
    print("Landing…")
    stop_event.set()
    land_mode = VehicleMode("LAND")
    vehicle.channels.overrides = {}
    vehicle.mode = land_mode
    while vehicle.mode != VehicleMode("LAND"):
        time.sleep(0.5)
    while vehicle.armed:
        time.sleep(0.5)
    print("Landed and disarmed.")


def main():
    # Connect to vehicle
    print("Connecting to vehicle…")
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

    # Parameters
    target_alt = 15  # takeoff altitude (m)
    target_lat, target_lon = TARGET_COORDINATES
    yaw_tolerance = 2  # deg
    forward_pwm = 1300  # pitch override for forward motion
    roll_gain = 0.4  # tuning for turn correction

    # Take off
    arm_and_takeoff(vehicle, target_alt)

    # Compute bearing to waypoint and rotate in place
    loc = vehicle.location.global_relative_frame
    _, bearing = get_distance_and_azimuth(loc, LocationGlobalRelative(target_lat, target_lon, 0))
    print(f"Rotating to face waypoint (bearing {bearing:.1f}°)…")
    condition_yaw(vehicle, bearing)
    wait_for_yaw(vehicle, bearing, yaw_tolerance)

    # Switch to ALT_HOLD and start altitude-hold loop
    print("Switching to ALT_HOLD and holding altitude…")
    vehicle.mode = VehicleMode("ALT_HOLD")
    time.sleep(1)
    stop_event = threading.Event()
    th = threading.Thread(
        target=altitude_hold_loop,
        args=(vehicle, target_alt, stop_event),
        daemon=True
    )
    th.start()

    # Move forward with course correction
    print("Moving forward to waypoint with course correction…")
    move_forward_with_course_correction(
        vehicle, target_lat, target_lon,
        pwm_forward=forward_pwm,
        roll_gain=roll_gain
    )

    print("Rotating to 350° in ALT_HOLD…")
    rotate_to_heading_alt_hold(vehicle, 350, threshold=yaw_tolerance, pwm_rate=1600)
    print("Mission complete.")


if __name__ == '__main__':
    main()
