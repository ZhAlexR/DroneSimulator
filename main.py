import time

from dronekit import LocationGlobalRelative

from constants import TARGET_COORDINATES, DEFAULT_TAKEOFF_ALT, TARGET_BEARING_AFTER_ARRIVING
from drone_controller import DroneController
from logger import logger


def main() -> None:
    logger.info("Mission script started.")
    controller = DroneController('tcp:127.0.0.1:5762')

    controller.arm()
    controller.set_mode("GUIDED")
    controller.takeoff(DEFAULT_TAKEOFF_ALT)

    current_loc = controller.vehicle.location.global_relative_frame
    target_loc = LocationGlobalRelative(*TARGET_COORDINATES, 0)

    _, bearing = controller.geodesy.inverse(current_loc, target_loc)
    logger.info("Rotating to waypoint bearing %.1f°...", bearing)
    controller.rotate_to(bearing)

    controller.set_mode("ALT_HOLD")

    logger.info("Moving to waypoint with course correction...")
    controller.move_to(target_loc)

    logger.info("Final yaw to 350° in GUIDED...")
    controller.set_mode("GUIDED")
    controller.rotate_to(TARGET_BEARING_AFTER_ARRIVING)

    time.sleep(20)
    logger.info("Mission complete.")


if __name__ == '__main__':
    main()
