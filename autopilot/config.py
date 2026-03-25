from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float


@dataclass(frozen=True)
class MissionConfig:
    point_a: GeoPoint
    point_b: GeoPoint
    target_altitude_m: float


@dataclass(frozen=True)
class PreflightConfig:
    heartbeat_max_age_s: float = 5.0
    require_armable: bool = True
    min_gps_fix_type: int = 2


@dataclass(frozen=True)
class TelemetryConfig:
    interval_s: float = 1.0
    session_duration_s: float = 15.0


@dataclass(frozen=True)
class MissionTimeoutsConfig:
    init_s: float = 5.0
    ready_s: float = 10.0
    arm_takeoff_stub_s: float = 20.0
    navigate_stub_s: float = 120.0
    land_stub_s: float = 20.0


@dataclass(frozen=True)
class PwmBoundsConfig:
    min_pwm: int = 1100
    max_pwm: int = 1900


@dataclass(frozen=True)
class RcNeutralConfig:
    roll: int = 1500
    pitch: int = 1500
    throttle: int = 1500
    yaw: int = 1500


@dataclass(frozen=True)
class RcOverrideConfig:
    bounds: PwmBoundsConfig = field(default_factory=PwmBoundsConfig)
    neutral: RcNeutralConfig = field(default_factory=RcNeutralConfig)
    max_delta_per_step: int = 40


@dataclass(frozen=True)
class GeoThresholdsConfig:
    navigate_arrival_radius_m: float = 30.0
    navigate_slowdown_radius_m: float = 100.0


@dataclass(frozen=True)
class MissionRuntimeConfig:
    tick_interval_s: float = 0.5
    arm_takeoff_stub_hold_s: float = 3.0
    land_stub_hold_s: float = 3.0


# ---------------------------------------------------------------------------
# Stage 3 config
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AltControllerConfig:
    """Tuning for PI altitude controller (throttle channel)."""

    kp: float = 5.0           # P gain: 1 m error → 5 PWM
    ki: float = 0.05          # I gain: mild integral correction
    hover_pwm: int = 1500     # PWM for level hover in Stabilize
    max_delta_pwm: int = 200  # Max throttle deviation from hover


@dataclass(frozen=True)
class Stage3TimeoutsConfig:
    """Per-state timeouts for Stage 3 FSM."""

    init_s: float = 10.0
    ready_s: float = 30.0
    takeoff_s: float = 90.0
    enroute_s: float = 480.0
    approach_s: float = 120.0
    landing_s: float = 120.0


@dataclass(frozen=True)
class Stage3NavConfig:
    """Navigation and control parameters for Stage 3."""

    # Takeoff
    takeoff_throttle_pwm: int = 1680          # RC throttle for initial climb
    takeoff_complete_fraction: float = 0.90   # Transition to ENROUTE at 90% target alt

    # Yaw controller (P gain, deg error → PWM delta)
    yaw_kp: float = 2.5

    # Pitch forward force
    pitch_delta_cruise: int = 120             # PWM delta during ENROUTE_TO_B (higher speed on long route)
    pitch_delta_fine: int = 80                # Max PWM delta during APPROACH_FINE (maintain momentum, reduce near landing)

    # Heading alignment guard
    align_threshold_deg: float = 20.0        # suppress forward pitch until within this many degrees of bearing

    # Phase transition radii [m]
    r_cruise_to_fine_m: float = 80.0          # Switch ENROUTE → APPROACH_FINE
    r_land_trigger_m: float = 8.0             # Trigger LANDING from APPROACH_FINE
    r_arrival_m: float = 6.0                  # Hard arrival radius

    # Landing conditions
    land_h_speed_threshold_ms: float = 0.5    # Max horizontal speed to approve landing
    land_complete_alt_m: float = 2.0          # Consider landed below this altitude [m]


@dataclass(frozen=True)
class Stage3Config:
    timeouts: Stage3TimeoutsConfig = field(default_factory=Stage3TimeoutsConfig)
    nav: Stage3NavConfig = field(default_factory=Stage3NavConfig)
    alt_ctrl: AltControllerConfig = field(default_factory=AltControllerConfig)


@dataclass(frozen=True)
class AppConfig:
    mission: MissionConfig
    preflight: PreflightConfig
    telemetry: TelemetryConfig
    mission_timeouts: MissionTimeoutsConfig
    rc_override: RcOverrideConfig
    geo_thresholds: GeoThresholdsConfig
    mission_runtime: MissionRuntimeConfig
    stage3: Stage3Config = field(default_factory=Stage3Config)


def build_default_config() -> AppConfig:
    return AppConfig(
        mission=MissionConfig(
            point_a=GeoPoint(lat=50.450739, lon=30.461242),
            point_b=GeoPoint(lat=50.443326, lon=30.448078),
            target_altitude_m=100.0,
        ),
        preflight=PreflightConfig(),
        telemetry=TelemetryConfig(),
        mission_timeouts=MissionTimeoutsConfig(),
        rc_override=RcOverrideConfig(),
        geo_thresholds=GeoThresholdsConfig(),
        mission_runtime=MissionRuntimeConfig(),
        stage3=Stage3Config(),
    )
