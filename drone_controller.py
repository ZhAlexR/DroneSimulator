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


class DroneController:
    def __init__(self, connection_str: str):
        self.vehicle = connect(connection_str, wait_ready=True)
        self.geodesy = Geodesy()
        self._alt_hold_stop = threading.Event()

    def arm(self):
        print("Arming motors...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(DEFAULT_SLEEP)

    def set_mode(self, mode_name: str):
        print(f"Switching to {mode_name} mode...")
        if mode_name == "ALT_HOLD":
            self._start_alt_hold()
            return
        if mode_name == "GUIDED":
            self._start_guided()
            return

    def takeoff(self, target_alt):
        print(f"Taking off to {target_alt} m...")
        self.vehicle.simple_takeoff(target_alt)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"  Alt = {alt:.1f} m")
            if alt >= target_alt - 1.0:
                print("Reached target altitude.")
                break
            time.sleep(DEFAULT_SLEEP)

    def rotate_to(
            self,
            heading: float,
            tolerance: float = DEFAULT_YAW_TOLERANCE,
            yaw_rate: int = 10,
    ):

        self._send_yaw_command(heading, yaw_rate)
        while True:
            curr = self.vehicle.heading
            error = abs(calculate_heading_error(heading, curr))
            if error <= tolerance:
                print(f"→ Yaw reached: {curr:.1f}° ±{tolerance}°")
                break
            print(f"Rotating... Curr {curr:.1f}°, Target {heading:.1f}°, Δ {error:.1f}°")
            time.sleep(DEFAULT_SLEEP / 10)

    def move_to(
            self,
            target: LocationGlobalRelative,
            tolerance: float = 1.0,
            forward_pwm: int = DEFAULT_FORWARD_PWM,
            roll_gain: float = DEFAULT_ROLL_GAIN,
    ):
        while True:
            loc = self.vehicle.location.global_relative_frame
            distance, bearing = self.geodesy.inverse(loc, target)

            error = calculate_heading_error(bearing, self.vehicle.heading)

            print(f"Distance: {distance:.2f} m | Bearing error: {error:.1f}°")
            if distance <= tolerance:
                print(f"→ Arrived within ±{tolerance} m")
                break

            self.vehicle.channels.overrides[CHANNEL_PITCH] = forward_pwm
            roll_pwm = int(CENTER_PWM + roll_gain * error * 10)
            roll_pwm = max(1000, min(2000, roll_pwm))
            self.vehicle.channels.overrides[CHANNEL_ROLL] = roll_pwm

            time.sleep(DEFAULT_SLEEP)

        for ch in (CHANNEL_PITCH, CHANNEL_ROLL):
            self.vehicle.channels.overrides[ch] = CENTER_PWM

    def _start_alt_hold(self):
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        time.sleep(DEFAULT_SLEEP)
        threading.Thread(target=self._alt_hold_loop, daemon=True).start()

    def _stop_alt_hold(self):
        self._alt_hold_stop.set()
        self.vehicle.channels.overrides.pop(CHANNEL_THROTTLE, None)

    def _start_guided(self):
        if self.vehicle.mode == VehicleMode("ALT_HOLD"):
            self._stop_alt_hold()
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(DEFAULT_SLEEP)

    def _alt_hold_pid(self, kp: float = 0.8, target_altitude: float = None):
        curr_alt = self.vehicle.location.global_relative_frame.alt

        if target_altitude is None:
            target_altitude = curr_alt

        error = target_altitude - curr_alt
        pwm = int(CENTER_PWM + kp * error * 100)
        pwm = max(1000, min(2000, pwm))
        self.vehicle.channels.overrides[CHANNEL_THROTTLE] = pwm

    def _alt_hold_loop(self):
        while not self._alt_hold_stop.is_set():
            self._alt_hold_pid()
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
