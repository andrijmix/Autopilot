from dataclasses import dataclass, field


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


@dataclass(frozen=True)
class AppConfig:
    mission: MissionConfig
    preflight: PreflightConfig
    telemetry: TelemetryConfig
    mission_timeouts: MissionTimeoutsConfig
    rc_override: RcOverrideConfig
    geo_thresholds: GeoThresholdsConfig
    mission_runtime: MissionRuntimeConfig


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
    )
