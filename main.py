import time

from dronekit import LocationGlobalRelative

from constants import TARGET_COORDINATES, DEFAULT_TAKEOFF_ALT
from drone_controller import DroneController


def main() -> None:
    controller = DroneController('tcp:127.0.0.1:5762')

    controller.arm()
    controller.set_mode("GUIDED")
    controller.takeoff(DEFAULT_TAKEOFF_ALT)

    current_loc = controller.vehicle.location.global_relative_frame
    target_loc = LocationGlobalRelative(*TARGET_COORDINATES, 0)

    _, bearing = controller.geodesy.inverse(current_loc, target_loc)
    print(f"Rotating to waypoint bearing {bearing:.1f}°...")
    controller.rotate_to(bearing)

    controller.set_mode("ALT_HOLD")

    print("Moving to waypoint with course correction...")
    controller.move_to(target_loc)

    print("Final yaw to 350° in GUIDED...")
    controller.set_mode("GUIDED")
    controller.rotate_to(350)

    time.sleep(20)
    print("Mission complete.")


if __name__ == '__main__':
    main()
