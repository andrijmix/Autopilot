from dataclasses import dataclass
from typing import Dict, Optional

from .dronekit_compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle

from .config import RcOverrideConfig


@dataclass
class RcCommand:
    roll: int
    pitch: int
    throttle: int
    yaw: int


class RcOverrideAdapter:
    """Applies saturated and rate-limited RC override commands to a vehicle."""

    CHANNEL_MAP = {
        "roll": "1",
        "pitch": "2",
        "throttle": "3",
        "yaw": "4",
    }

    def __init__(self, vehicle: Vehicle, cfg: RcOverrideConfig):
        self._vehicle = vehicle
        self._cfg = cfg
        self._last_cmd = RcCommand(
            roll=cfg.neutral.roll,
            pitch=cfg.neutral.pitch,
            throttle=cfg.neutral.throttle,
            yaw=cfg.neutral.yaw,
        )

    def apply(self, roll: int, pitch: int, throttle: int, yaw: int) -> RcCommand:
        cmd = RcCommand(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
        cmd = self._saturate(cmd)
        cmd = self._rate_limit(cmd)
        self._send(cmd)
        self._last_cmd = cmd
        return cmd

    def neutral_command(self) -> RcCommand:
        return RcCommand(
            roll=self._cfg.neutral.roll,
            pitch=self._cfg.neutral.pitch,
            throttle=self._cfg.neutral.throttle,
            yaw=self._cfg.neutral.yaw,
        )

    def safe_reset(self) -> None:
        self._vehicle.channels.overrides = {}
        self._last_cmd = self.neutral_command()

    def _send(self, cmd: RcCommand) -> None:
        self._vehicle.channels.overrides = {
            self.CHANNEL_MAP["roll"]: cmd.roll,
            self.CHANNEL_MAP["pitch"]: cmd.pitch,
            self.CHANNEL_MAP["throttle"]: cmd.throttle,
            self.CHANNEL_MAP["yaw"]: cmd.yaw,
        }

    def _saturate(self, cmd: RcCommand) -> RcCommand:
        return RcCommand(
            roll=self._clip(cmd.roll),
            pitch=self._clip(cmd.pitch),
            throttle=self._clip(cmd.throttle),
            yaw=self._clip(cmd.yaw),
        )

    def _rate_limit(self, cmd: RcCommand) -> RcCommand:
        step = self._cfg.max_delta_per_step
        return RcCommand(
            roll=self._limit_delta(self._last_cmd.roll, cmd.roll, step),
            pitch=self._limit_delta(self._last_cmd.pitch, cmd.pitch, step),
            throttle=self._limit_delta(self._last_cmd.throttle, cmd.throttle, step),
            yaw=self._limit_delta(self._last_cmd.yaw, cmd.yaw, step),
        )

    def _clip(self, value: int) -> int:
        return max(self._cfg.bounds.min_pwm, min(self._cfg.bounds.max_pwm, int(value)))

    @staticmethod
    def _limit_delta(previous: int, target: int, max_delta: int) -> int:
        delta = target - previous
        if delta > max_delta:
            return previous + max_delta
        if delta < -max_delta:
            return previous - max_delta
        return target


class DryRunRcOverrideAdapter:
    """Computes RC commands but never sends them to the vehicle."""

    def __init__(self, cfg: RcOverrideConfig):
        self._cfg = cfg
        self._last_cmd = RcCommand(
            roll=cfg.neutral.roll,
            pitch=cfg.neutral.pitch,
            throttle=cfg.neutral.throttle,
            yaw=cfg.neutral.yaw,
        )

    def apply(self, roll: int, pitch: int, throttle: int, yaw: int) -> RcCommand:
        cmd = RcCommand(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
        cmd = self._saturate(cmd)
        cmd = self._rate_limit(cmd)
        self._last_cmd = cmd
        return cmd

    def neutral_command(self) -> RcCommand:
        return RcCommand(
            roll=self._cfg.neutral.roll,
            pitch=self._cfg.neutral.pitch,
            throttle=self._cfg.neutral.throttle,
            yaw=self._cfg.neutral.yaw,
        )

    def safe_reset(self) -> None:
        self._last_cmd = self.neutral_command()

    def _saturate(self, cmd: RcCommand) -> RcCommand:
        return RcCommand(
            roll=self._clip(cmd.roll),
            pitch=self._clip(cmd.pitch),
            throttle=self._clip(cmd.throttle),
            yaw=self._clip(cmd.yaw),
        )

    def _rate_limit(self, cmd: RcCommand) -> RcCommand:
        step = self._cfg.max_delta_per_step
        return RcCommand(
            roll=self._limit_delta(self._last_cmd.roll, cmd.roll, step),
            pitch=self._limit_delta(self._last_cmd.pitch, cmd.pitch, step),
            throttle=self._limit_delta(self._last_cmd.throttle, cmd.throttle, step),
            yaw=self._limit_delta(self._last_cmd.yaw, cmd.yaw, step),
        )

    def _clip(self, value: int) -> int:
        return max(self._cfg.bounds.min_pwm, min(self._cfg.bounds.max_pwm, int(value)))

    @staticmethod
    def _limit_delta(previous: int, target: int, max_delta: int) -> int:
        delta = target - previous
        if delta > max_delta:
            return previous + max_delta
        if delta < -max_delta:
            return previous - max_delta
        return target
