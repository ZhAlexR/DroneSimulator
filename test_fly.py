from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
from pyproj import Geod

TARGET_COORDINATES = 50.443326, 30.448078


def get_distance_and_azimuth(loc1, loc2):
    geod = Geod(ellps="WGS84")
    azimuth, _, distance = geod.inv(
        lons1=loc1.lon,
        lats1=loc1.lat,
        lons2=loc2.lon,
        lats2=loc2.lat,
    )
    return distance, azimuth


def send_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,  # x, y, z positions
        vx, vy, vz,  # x, y, z velocity
        0, 0, 0,  # acceleration
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


# Helper: Set yaw to a specific heading (azimuth in degrees)
def condition_yaw(vehicle, heading, relative=False):
    print(f"Rotating to {heading} degrees...")
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading, 0, 1,
        is_relative, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"Current azimuth is {heading} degrees")


def connect_to_vehicle(connection: str):
    print("Connecting to vehicle ...")
    vehicle = connect(connection, wait_ready=True)
    print(f"Connected to vehicle: {connection}")
    return vehicle


def arm_and_takeoff(vehicle, target_altitude):
    print("Arming ...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != VehicleMode("GUIDED"):
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for vehicle to be armed...")
        time.sleep(1)

    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Current altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            break
        time.sleep(1)
    print(f"Target altitude: {target_altitude} reached!")


    vehicle.mode = VehicleMode("ALT_HOLD")
    while vehicle.mode != VehicleMode("ALT_HOLD"):
        print("Waiting for vehicle to enter ALT_HOLD mode...")
        time.sleep(1)


def move_to_target_point(vehicle, target_location):
    print("Navigating to target...")
    while True:
        current = vehicle.location.global_relative_frame
        dist, az = get_distance_and_azimuth(current, target_location)
        new_az = (az + 360) % 360
        condition_yaw(vehicle, new_az)
        print(f"Distance to target: {dist:.2f} m, azimuth: {new_az}, altitude: {vehicle.location.global_relative_frame.alt}")

        if dist < 50:
            send_velocity_body(vehicle, 1, 0, 0)
            time.sleep(1)
            continue

        if dist < 1.0:
            send_velocity_body(vehicle, 0, 0, 0)
            print("Arrived at target.")
            break

        # Move forward slowly
        send_velocity_body(vehicle, 50, 0, 0)
        time.sleep(1)


def land_vehicle(vehicle):
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        time.sleep(1)

    print("Landed and disarmed.")


def main():
    connection_string = "tcp:127.0.0.1:5762"
    vehicle = connect_to_vehicle(connection_string)
    arm_and_takeoff(vehicle, target_altitude=15)
    target_location = LocationGlobalRelative(*TARGET_COORDINATES)

    current = vehicle.location.global_relative_frame
    dist, az = get_distance_and_azimuth(current, target_location)
    new_az = (az + 360) % 360
    condition_yaw(vehicle, new_az)
    time.sleep(2)

    move_to_target_point(vehicle, target_location)
    condition_yaw(vehicle, 350)
    time.sleep(2)
    land_vehicle(vehicle)
    vehicle.close()


if __name__ == "__main__":
    main()
