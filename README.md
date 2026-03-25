# Copter Autopilot - Stage 2 (Mission Planner + DroneKit)

Stage 2 adds a controlled mission scaffold on top of Stage 1:
- FSM-based mission orchestration
- geo computations for A->B navigation context
- safe RC Override adapter (bounds + neutral + rate limit + reset)
- dry-run integration mode without real vehicle movement

Real takeoff/landing control and PID loops are intentionally deferred to the next stage.

## Mission Parameters

- Point A: `50.450739, 30.461242`
- Point B: `50.443326, 30.448078`
- Target altitude: `100 m`

These values are defined in `autopilot/config.py`.

## Project Structure

- `autopilot/config.py` - mission, FSM timeouts, RC safety, geo thresholds
- `autopilot/connection.py` - DroneKit vehicle connection
- `autopilot/preflight.py` - pre-flight checks
- `autopilot/telemetry.py` - telemetry snapshot helper
- `autopilot/geo.py` - distance, bearing, angle normalization
- `autopilot/rc_override.py` - safe RC override adapter + dry-run adapter
- `autopilot/mission_fsm.py` - mission state machine orchestrator
- `autopilot/main.py` - CLI entry point for stage 2 run

## Setup

1. Install Python 3.10+.
2. Install dependencies:

```bash
pip install -r requirements.txt
```

## Run (SITL or real vehicle)

### Dry-run mode (recommended first)

In dry-run mode the script computes and logs RC commands but does not send RC override to vehicle.

```bash
python -m autopilot.main --connect 127.0.0.1:14550 --dry-run
```

### Normal mode

In normal mode RC override is sent through DroneKit channel overrides.

```bash
python -m autopilot.main --connect 127.0.0.1:14550
```

Or run the file directly:

```bash
python autopilot/main.py --connect 127.0.0.1:14550
```

Optional flags:

- `--baud 57600`
- `--telemetry-duration 20`
- `--telemetry-interval 1`
- `--dry-run`

## Typical SITL start examples

If you already have ArduPilot SITL running and MAVProxy forwarding to `14550`, run only the Python command above.

Example with `sim_vehicle.py`:

```bash
sim_vehicle.py -v ArduCopter --console --map
```

Then run:

```bash
python -m autopilot.main --connect 127.0.0.1:14550
```

## FSM states in Stage 2

- `INIT`
- `READY`
- `ARM_TAKEOFF_STUB`
- `NAVIGATE_STUB`
- `LAND_STUB`
- `COMPLETE`
- `ABORT`

Each transition is logged with state-from/state-to and transition reason.
Each active state has its own timeout from config.

## Expected dry-run log example

```text
2026-03-24 12:00:00,000 | INFO | Mission config: A=(50.450739, 30.461242), B=(50.443326, 30.448078), target_alt_m=100.0
2026-03-24 12:00:00,200 | INFO | Connecting to vehicle: 127.0.0.1:14550
2026-03-24 12:00:01,100 | INFO | Connection established
2026-03-24 12:00:01,101 | INFO | preflight check=heartbeat ok=True details=Heartbeat age: 0.42s
2026-03-24 12:00:01,101 | INFO | preflight check=armable ok=True details=is_armable=True
2026-03-24 12:00:01,101 | INFO | preflight check=mode ok=True details=mode=STABILIZE
2026-03-24 12:00:01,101 | INFO | preflight check=gps ok=True details=fix_type=3, eph=120, satellites=10
2026-03-24 12:00:01,102 | INFO | Stage 2 running in DRY-RUN mode: RC override commands will be logged only
2026-03-24 12:00:01,102 | INFO | FSM started: state=INIT
2026-03-24 12:00:01,103 | INFO | FSM transition: INIT -> READY reason=initialization complete elapsed_in_state=0.00s
2026-03-24 12:00:01,603 | INFO | FSM transition: READY -> ARM_TAKEOFF_STUB reason=preflight passed, mission ready elapsed_in_state=0.50s
2026-03-24 12:00:02,104 | INFO | state=ARM_TAKEOFF_STUB stub_hold_s=0.5 rc(roll=1500 pitch=1500 throttle=1500 yaw=1500)
2026-03-24 12:00:05,106 | INFO | FSM transition: ARM_TAKEOFF_STUB -> NAVIGATE_STUB reason=takeoff stub hold complete elapsed_in_state=3.50s
2026-03-24 12:00:05,607 | INFO | state=NAVIGATE_STUB dist_m=1268.3 bearing_deg=228.1 heading_deg=220.0 yaw_error_deg=8.1 rc(roll=1500 pitch=1540 throttle=1500 yaw=1540)
2026-03-24 12:00:06,108 | INFO | state=NAVIGATE_STUB dist_m=1268.0 bearing_deg=228.0 heading_deg=222.0 yaw_error_deg=6.0 rc(roll=1500 pitch=1580 throttle=1500 yaw=1532)
2026-03-24 12:02:06,109 | INFO | FSM transition: NAVIGATE_STUB -> ABORT reason=timeout in NAVIGATE_STUB: 120.0s > 120.0s elapsed_in_state=120.00s
2026-03-24 12:02:06,610 | ERROR | FSM abort: RC override safely reset
2026-03-24 12:02:06,611 | ERROR | Mission finished with ABORT state
```
