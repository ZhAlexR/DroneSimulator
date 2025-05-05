import time
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

from constants import (
    CHANNEL_ROLL,
    CHANNEL_PITCH,
    CHANNEL_THROTTLE,
    CENTER_PWM,
    DEFAULT_YAW_TOLERANCE,
    DEFAULT_FORWARD_PWM,
    DEFAULT_ROLL_GAIN,
    DEFAULT_SLEEP,
)
from geodesy import Geodesy, calculate_heading_error, shortest_rotation_direction
from logger import logger


class DroneController:
    def __init__(self, connection_str: str):
        logger.info("Connecting to vehicle on %s", connection_str)
        self.vehicle = connect(connection_str, wait_ready=True)
        self.geodesy = Geodesy()
        self._alt_hold_stop = threading.Event()

    def arm(self):
        logger.info("Arming motors...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(DEFAULT_SLEEP)
        logger.info("Motors armed.")

    def set_mode(self, mode_name: str):
        logger.info("Switching to %s mode...", mode_name)
        if mode_name == "ALT_HOLD":
            self._start_alt_hold()
            return
        if mode_name == "GUIDED":
            self._start_guided()
            return
        logger.warning("Mode %s not recognized.", mode_name)

    def takeoff(self, target_alt):
        logger.info("Taking off to %.1f m...", target_alt)
        self.vehicle.simple_takeoff(target_alt)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            logger.info("  Alt = %.1f m", alt)
            if alt >= target_alt - 1.0:
                logger.info("Reached target altitude.")
                break
            time.sleep(DEFAULT_SLEEP)

    def rotate_to(
            self,
            heading: float,
            tolerance: float = DEFAULT_YAW_TOLERANCE,
            yaw_rate: int = 10,
    ):
        logger.info("Rotating to heading %.1f° ±%.1f°", heading, tolerance)
        self._send_yaw_command(heading, yaw_rate)
        while True:
            curr = self.vehicle.heading
            error = abs(calculate_heading_error(heading, curr))
            if error <= tolerance:
                logger.info("→ Yaw reached: %.1f° ±%.1f°", curr, tolerance)
                break
            logger.debug(
                "Rotating... Curr %.1f°, Target %.1f°, Δ %.1f°", curr, heading, error
            )
            time.sleep(DEFAULT_SLEEP / 10)

    def move_to(
            self,
            target: LocationGlobalRelative,
            tolerance: float = 1.0,
            forward_pwm: int = DEFAULT_FORWARD_PWM,
            slow_pwm: int = None,
            roll_gain: float = DEFAULT_ROLL_GAIN,
    ):
        if slow_pwm is None:
            slow_pwm = int((CENTER_PWM + forward_pwm) / 2)

        logger.info(
            "Moving to %s with tolerance ±%.1f m (slow_pwm=%d @ ≤10 m)",
            target, tolerance, slow_pwm
        )
        while True:
            loc = self.vehicle.location.global_relative_frame
            distance, bearing = self.geodesy.inverse(loc, target)

            error = calculate_heading_error(bearing, self.vehicle.heading)
            logger.debug(
                "Distance: %.2f m | Bearing error: %.1f°", distance, error
            )
            if distance <= tolerance:
                logger.info("→ Arrived within ±%.1f m", tolerance)
                break

            if distance <= 10.0:
                pitch_pwm = slow_pwm
            else:
                pitch_pwm = forward_pwm

            self.vehicle.channels.overrides[CHANNEL_PITCH] = pitch_pwm
            roll_pwm = int(CENTER_PWM + roll_gain * error * 10)
            roll_pwm = max(1000, min(2000, roll_pwm))
            self.vehicle.channels.overrides[CHANNEL_ROLL] = roll_pwm

            time.sleep(DEFAULT_SLEEP)

        for ch in (CHANNEL_PITCH, CHANNEL_ROLL):
            self.vehicle.channels.overrides[ch] = CENTER_PWM
        logger.info("Movement overrides cleared.")

    def _start_alt_hold(self):
        logger.info("Starting ALT_HOLD mode and altitude hold loop.")
        current_alt = self.vehicle.location.global_relative_frame.alt
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        time.sleep(DEFAULT_SLEEP)
        threading.Thread(target=self._alt_hold_loop, args=(current_alt,), daemon=True).start()

    def _stop_alt_hold(self):
        logger.info("Stopping ALT_HOLD mode.")
        self._alt_hold_stop.set()
        self.vehicle.channels.overrides.pop(CHANNEL_THROTTLE, None)

    def _start_guided(self):
        logger.info("Starting GUIDED mode.")
        if self.vehicle.mode == VehicleMode("ALT_HOLD"):
            self._stop_alt_hold()
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(DEFAULT_SLEEP)

    def _alt_hold_pid(self, kp: float = 0.8, target_altitude: float = None):
        curr_alt = self.vehicle.location.global_relative_frame.alt
        error = target_altitude - curr_alt
        pwm = int(CENTER_PWM + kp * error * 100)
        pwm = max(1000, min(2000, pwm))
        self.vehicle.channels.overrides[CHANNEL_THROTTLE] = pwm
        logger.debug("ALT_HOLD PID: curr=%.1f, target=%.1f, PWM=%d", curr_alt, target_altitude, pwm)

    def _alt_hold_loop(self, target_altitude: float):
        while not self._alt_hold_stop.is_set():
            self._alt_hold_pid(target_altitude=target_altitude)
            time.sleep(DEFAULT_SLEEP / 10)

    def _send_yaw_command(
            self,
            heading: float,
            yaw_rate: int = 10,
    ):
        direction = shortest_rotation_direction(heading, self.vehicle.heading)
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading, yaw_rate, direction, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg)
        logger.debug("Yaw command sent: heading=%.1f, rate=%d, dir=%d", heading, yaw_rate, direction)
