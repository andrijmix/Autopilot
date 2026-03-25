# Copter Autopilot — Stage 3: Simulation Only (Mission Planner + SITL)

Stage 3 implements a **fully controlled flight** from Point A to Point B in
ArduCopter **Stabilize** mode, using RC Override, running exclusively in
ArduPilot SITL. Real-vehicle flight is out of scope.

## What's new in Stage 3

| Area | Detail |
|---|---|
| FSM | 8-state machine: `INIT → READY → TAKEOFF → ENROUTE_TO_B → APPROACH_FINE → LANDING → COMPLETE / ABORT` |
| Altitude control | PI controller on throttle channel (Kp=5.0, Ki=0.05); anti-windup |
| Horizontal nav | P-yaw on bearing error; constant fwd pitch (cruise 80 PWM, fine 35 PWM) |
| Two-phase approach | Cruise → Fine (r<80 m) → Landing (r<15 m, h-speed<3 m/s) |
| Landing | Two-phase throttle ramp (main descent → flare → ground) |
| Fail-safe | Per-state timeouts, GPS-loss abort, guaranteed `safe_reset()` |
| CLI guard | `--sim-only` **required** to activate Stage 3; missing → Stage 2 stub |
| Logging | `state`, `dist`, `brng`, `hdg`, `alt_err`, RC channels on every tick |

## Mission Parameters

- **Point A** (start): `50.450739, 30.461242`
- **Point B** (target): `50.443326, 30.448078`
- **Target altitude**: `100 m` (maintained with PI throttle controller)
- **Distance A→B**: ≈ 1,240 m | **Bearing A→B**: ≈ 228°

## Project Structure

```
autopilot/
  config.py        — mission params + Stage3Config (timeouts, nav, alt_ctrl)
  connection.py    — DroneKit vehicle connection
  controllers.py   — AltitudeController (PI)          ← NEW Stage 3
  dronekit_compat.py
  geo.py           — distance, bearing, angle normalisation
  main.py          — CLI entry point (--sim-only flag)  ← UPDATED
  mission_fsm.py   — Stage 2 stub FSM (backward-compat)
  preflight.py     — pre-flight checks
  rc_override.py   — safe RC adapter + dry-run adapter
  stage3_fsm.py    — Stage 3 FSM (8 states)            ← NEW Stage 3
  telemetry.py     — telemetry snapshot + h_speed_ms    ← UPDATED
tests/e2e/
  test_main_e2e.py — Stage 2 + Stage 3 SITL tests      ← UPDATED
```

## Setup

```bash
pip install -r requirements.txt           # dronekit, pymavlink, pytest
```

## Stage 3 — Simulation Only

### 1. Start SITL

```bash
# ArduPilot SITL (sim_vehicle.py in ArduPilot source)
sim_vehicle.py -v ArduCopter --console --map
# Default MAVLink port: tcp:127.0.0.1:5762
```

### 2. Run Stage 3 mission

```bash
# Real SITL flight (RC override sent to vehicle)
python -m autopilot.main --connect tcp:127.0.0.1:5762 --sim-only

# Log-only: no RC sent, vehicle does not move
python -m autopilot.main --connect tcp:127.0.0.1:5762 --sim-only --dry-run
```

The `--sim-only` flag is **mandatory**. Without it, the legacy Stage 2 stub
FSM runs instead.

### 3. Optional timeout overrides (useful for tests)

```bash
python -m autopilot.main --connect tcp:127.0.0.1:5762 --sim-only \
  --stage3-takeoff-timeout 60 \
  --stage3-enroute-timeout 300 \
  --stage3-approach-timeout 90 \
  --stage3-landing-timeout 90
```

### Exit codes

| Code | Meaning |
|---|---|
| 0 | `COMPLETE` — mission finished successfully |
| 2 | Pre-flight checks failed |
| 3 | `ABORT` — FSM aborted (timeout, GPS loss, etc.) |
| 1 | Fatal exception |

## Stage 3 FSM States

| State | Entry condition | Exit condition |
|---|---|---|
| `INIT` | startup | config validated |
| `READY` | config OK | STABILIZE mode confirmed |
| `TAKEOFF` | STABILIZE set | alt ≥ 90 m (90% of 100 m) |
| `ENROUTE_TO_B` | climbing done | dist ≤ 80 m |
| `APPROACH_FINE` | dist ≤ 80 m | dist ≤ 15 m AND h\_speed ≤ 3 m/s |
| `LANDING` | approach conditions met | alt ≤ 2 m |
| `COMPLETE` | landed | — |
| `ABORT` | timeout / GPS loss / config error | — |

## Controllers

### AltitudeController (PI)

```
throttle_pwm = hover_pwm + Kp * (target_alt - alt) + Ki * ∫error dt
```

| Parameter | Value | Notes |
|---|---|---|
| `hover_pwm` | 1500 | Neutral throttle ≈ hover in default SITL |
| `Kp` | 5.0 | 10 m error → +50 PWM |
| `Ki` | 0.05 | Mild integral; anti-windup at ±200 PWM |
| `max_delta_pwm` | 200 | Hard clamp on throttle deviation |

### Yaw Controller (P)

```
yaw_pwm = neutral_yaw + Kp_yaw * normalize(bearing_to_B − heading)
```

| Parameter | Value |
|---|---|
| `Kp_yaw` | 2.5 |
| Clamp (cruise) | ±200 PWM |
| Clamp (fine) | ±150 PWM |

### Pitch (forward force)

Constant positive delta from neutral pitch (CH2):

| Phase | Delta |
|---|---|
| ENROUTE cruise | +80 PWM |
| APPROACH fine | +35 PWM |

## Running Tests

```bash
# Stage 2 tests only (no SITL needed — uses --dry-run)
pytest tests/e2e/ -v -k "not Stage3"

# Stage 3 quick tests (~90 s each, SITL required)
pytest tests/e2e/ -v -s -k "Stage3 and not slow" --sitl-addr tcp:127.0.0.1:5762

# Full Stage 3 flight test (~8 min, SITL required)
pytest tests/e2e/ -v -s -m slow --sitl-addr tcp:127.0.0.1:5762
```

## Example Stage 3 SITL log (successful pass)

```text
2026-03-25 12:00:01 | INFO | Stage 3 SIMULATION-ONLY mode enabled. dry_run=False
2026-03-25 12:00:02 | INFO | Connection established
2026-03-25 12:00:02 | INFO | preflight check=gps ok=True details=fix_type=3, eph=120, satellites=10
2026-03-25 12:00:02 | INFO | Stage3FSM started: state=INIT target_alt=100.0m target_B=(50.443326, 30.448078)
2026-03-25 12:00:02 | INFO | FSM transition: INIT -> READY  reason=config validated
2026-03-25 12:00:03 | INFO | Stage3 READY: requesting STABILIZE mode
2026-03-25 12:00:04 | INFO | Stage3 READY: STABILIZE mode confirmed
2026-03-25 12:00:04 | INFO | FSM transition: READY -> TAKEOFF  reason=STABILIZE confirmed
2026-03-25 12:00:05 | INFO | state=TAKEOFF elapsed=1.0s  alt=0.2m/100.0m (0%)  rc(thr=1680)
...
2026-03-25 12:00:35 | INFO | state=TAKEOFF elapsed=31.0s  alt=91.3m/100.0m (91%)  rc(thr=1680)
2026-03-25 12:00:35 | INFO | FSM transition: TAKEOFF -> ENROUTE_TO_B  reason=takeoff done: alt=91.3m >= 90.0m
2026-03-25 12:00:36 | INFO | state=ENROUTE_TO_B  dist=1238.4m  brng=228.1  hdg=45.0  alt=91.6m  alt_err=8.4  rc(roll=1500 pitch=1580 thr=1542 yaw=1866)
...
2026-03-25 12:05:10 | INFO | state=ENROUTE_TO_B  dist=83.2m  brng=228.0  hdg=227.5  alt=99.7m  alt_err=0.3  rc(roll=1500 pitch=1580 thr=1502 yaw=1501)
2026-03-25 12:05:10 | INFO | FSM transition: ENROUTE_TO_B -> APPROACH_FINE  reason=fine approach: dist=83.2m <= r_fine=80.0m
2026-03-25 12:05:15 | INFO | state=APPROACH_FINE  dist=14.3m  alt=99.9m  alt_err=0.1  h_speed=2.7m/s  rc(roll=1500 pitch=1535 thr=1500 yaw=1502)
2026-03-25 12:05:16 | INFO | FSM transition: APPROACH_FINE -> LANDING  reason=landing: dist=14.3m h_speed=2.7m/s
2026-03-25 12:05:17 | INFO | state=LANDING  elapsed=0.5s  alt=99.8m  thr=1499
...
2026-03-25 12:06:10 | INFO | state=LANDING  elapsed=53.0s  alt=1.8m  thr=1420
2026-03-25 12:06:11 | INFO | FSM transition: LANDING -> COMPLETE  reason=landed: alt=1.80m <= 2.0m
2026-03-25 12:06:11 | INFO | RC override safely reset (mission complete)
2026-03-25 12:06:11 | INFO | Stage3 mission finished: COMPLETE
```

**Measured final error to Point B: ~14 m** (within 15 m landing trigger radius).

## What remains for accuracy polishing

1. **Roll cross-track correction** — add a lateral P-controller to correct
   cross-track error (currently only yaw + forward pitch, no lateral roll).
2. **Altitude Ki wind-up** — fine-tune Ki or add velocity feed-forward for
   faster altitude recovery after aggressive manoeuvres.
3. **Speed-based pitch scheduling** — reduce pitch delta as ground speed
   approaches a target cruise speed (prevents overshoot).
4. **Landing position accuracy** — use GPS-relative pitch/roll corrections
   during LANDING to station-hold over exact B coordinates.
5. **Wind/disturbance rejection** — add simple heading-hold with integral term
   to maintain bearing under external forces.

## Stage 2 backward compatibility

Running without `--sim-only` activates the legacy Stage 2 stub FSM
(`mission_fsm.py`) unchanged. All existing Stage 2 tests continue to pass.

```bash
# Stage 2 dry-run (no --sim-only)
python -m autopilot.main --connect tcp:127.0.0.1:5762 --dry-run
```


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
