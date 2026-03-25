import argparse
import logging
import sys
from dataclasses import replace
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from autopilot.config import build_default_config
    from autopilot.connection import connect_vehicle
    from autopilot.mission_fsm import MissionFSM, MissionState
    from autopilot.preflight import run_preflight_checks
    from autopilot.rc_override import DryRunRcOverrideAdapter, RcOverrideAdapter
else:
    from .config import build_default_config
    from .connection import connect_vehicle
    from .mission_fsm import MissionFSM, MissionState
    from .preflight import run_preflight_checks
    from .rc_override import DryRunRcOverrideAdapter, RcOverrideAdapter


def configure_logging() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )
    return logging.getLogger("autopilot")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Autopilot Stage 2: preflight + FSM mission scaffold in Stabilize"
    )
    parser.add_argument("--connect", required=True, help="Vehicle connection string")
    parser.add_argument("--baud", type=int, default=57600, help="Connection baud rate")
    parser.add_argument(
        "--telemetry-duration",
        type=float,
        default=None,
        help="Override telemetry session duration in seconds",
    )
    parser.add_argument(
        "--telemetry-interval",
        type=float,
        default=None,
        help="Override telemetry log interval in seconds",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not send RC override to vehicle; compute and log commands only",
    )
    parser.add_argument(
        "--navigate-stub-timeout",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Override FSM NAVIGATE_STUB timeout (useful for tests)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logger = configure_logging()
    cfg = build_default_config()

    if args.telemetry_duration is not None:
        cfg = replace(
            cfg,
            telemetry=replace(cfg.telemetry, session_duration_s=args.telemetry_duration),
        )

    if args.telemetry_interval is not None:
        cfg = replace(cfg, telemetry=replace(cfg.telemetry, interval_s=args.telemetry_interval))

    if args.navigate_stub_timeout is not None:
        cfg = replace(
            cfg,
            mission_timeouts=replace(
                cfg.mission_timeouts,
                navigate_stub_s=args.navigate_stub_timeout,
            ),
        )

    vehicle = None
    try:
        logger.info(
            "Mission config: A=(%.6f, %.6f), B=(%.6f, %.6f), target_alt_m=%.1f",
            cfg.mission.point_a.lat,
            cfg.mission.point_a.lon,
            cfg.mission.point_b.lat,
            cfg.mission.point_b.lon,
            cfg.mission.target_altitude_m,
        )
        logger.info("Connecting to vehicle: %s", args.connect)
        vehicle = connect_vehicle(args.connect, baud=args.baud)
        logger.info("Connection established")

        report = run_preflight_checks(vehicle, cfg.preflight)
        for check in report.checks:
            level = logging.INFO if check.ok else logging.ERROR
            logger.log(level, "preflight check=%s ok=%s details=%s", check.name, check.ok, check.details)

        if not report.ok:
            logger.error("Pre-flight checks failed. Mission aborted before FSM start.")
            return 2

        if args.dry_run:
            logger.info("Stage 2 running in DRY-RUN mode: RC override commands will be logged only")
            rc_adapter = DryRunRcOverrideAdapter(cfg.rc_override)
        else:
            logger.info("Stage 2 running in NORMAL mode: RC override commands will be sent to vehicle")
            rc_adapter = RcOverrideAdapter(vehicle, cfg.rc_override)

        fsm = MissionFSM(vehicle=vehicle, cfg=cfg, rc_adapter=rc_adapter, logger=logger)
        result = fsm.run()

        if result == MissionState.COMPLETE:
            logger.info("Mission finished with COMPLETE state")
            return 0

        logger.error("Mission finished with ABORT state")
        return 3
    except Exception as exc:
        logger.exception("Fatal error: %s", exc)
        return 1
    finally:
        if vehicle is not None:
            vehicle.close()
            logger.info("Vehicle connection closed")


if __name__ == "__main__":
    sys.exit(main())
