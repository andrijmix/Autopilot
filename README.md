# Copter Autopilot — Stage 3: Simulation Only (Mission Planner + SITL)

Stage 3 implements a **fully controlled flight** from Point A to Point B in
ArduCopter **Stabilize** mode, using RC Override, running exclusively in
ArduPilot SITL. Real-vehicle flight is out of scope.

## What's implemented in Stage 3

| Area | Detail |
|---|---|
| FSM | 8-state machine: `INIT → READY → TAKEOFF → ENROUTE_TO_B → APPROACH_FINE → LANDING → COMPLETE / ABORT` |
| Altitude control | PI controller on throttle (Kp=6.5, Ki=0.08); anti-windup ±200 PWM |
| Takeoff pitch | PI controller on horizontal drift speed; corrects wind pushback during climb |
| Horizontal nav | P-yaw on bearing error + crab-angle wind compensation |
| Cruise pitch | +1200 PWM from neutral (ramp over 8 s); adaptive bleed when misaligned |
| Approach | Speed-targeted pitch: tapers track speed 2→0 m/s from 18 m to 10 m radius |
| Crab-angle controller | PI on lateral speed → heading offset; compensates steady crosswind |
| Landing | Elapsed-time throttle ramp (assertive descent → flare → ground) |
| Fail-safe | Per-state timeouts, GPS-loss abort, guaranteed `safe_reset()` |
| CLI guard | `--sim-only` **required** to activate Stage 3; missing → Stage 2 stub |
| Logging | `state`, `dist`, `brng`, `crab`, `alt_err`, `h_speed`, `lat_speed`, RC channels on every tick |

## Mission Parameters

- **Point A** (start): `50.450739, 30.461242`
- **Point B** (target): `50.443326, 30.448078`
- **Target altitude**: `300 m` (held with PI throttle controller)
- **Distance A→B**: ≈ 1,240 m | **Bearing A→B**: ≈ 228°

## Project Structure

```
autopilot/
  config.py          — mission params + Stage3Config (timeouts, nav, alt_ctrl)
  connection.py      — DroneKit vehicle connection
  controllers.py     — AltitudeController (PI) + LateralController (PI, crab-angle)
  dronekit_compat.py — DroneKit monkey-patches for SITL compatibility
  geo.py             — distance, bearing, angle normalisation
  main.py            — CLI entry point (--sim-only / --dry-run flags)
  mission_fsm.py     — Stage 2 stub FSM (backward-compat)
  preflight.py       — pre-flight checks
  rc_override.py     — safe RC adapter + dry-run adapter
  stage3_fsm.py      — Stage 3 FSM (8 states)
  telemetry.py       — telemetry snapshot helper

tests/
  conftest.py
  test_stage3_fsm.py — unit tests for Stage 3 FSM helpers (no SITL required)
```

## Setup

```bash
pip install -r requirements.txt   # dronekit, pymavlink, pytest
```

Requires Python 3.10+.

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
  --stage3-takeoff-timeout 120 \
  --stage3-enroute-timeout 480 \
  --stage3-approach-timeout 120 \
  --stage3-landing-timeout 120
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
| `READY` | config OK | STABILIZE mode confirmed; vehicle disarmed if needed |
| `TAKEOFF` | STABILIZE set | alt ≥ 97% of target (291 m) |
| `ENROUTE_TO_B` | climbing done | dist ≤ 18 m (r_cruise_to_fine) |
| `APPROACH_FINE` | dist ≤ 18 m | dist ≤ 10 m (r_land_trigger) OR dist ≤ 8 m (r_arrival) |
| `LANDING` | inside landing radius | alt ≤ 2 m |
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
| `Kp` | 6.5 | 10 m error → +65 PWM |
| `Ki` | 0.08 | Faster steady-state correction under aggressive pitch |
| `max_delta_pwm` | 200 | Hard clamp on throttle deviation from hover |

### LateralController — Crab-Angle Wind Compensation (PI)

Rather than using the roll channel (which causes banked turns in Stabilize),
lateral drift is corrected by offsetting the desired heading upwind so the
forward thrust vector cancels the crosswind component.

```
crab_deg = -(Kp_crab * lat_speed + Ki_crab * ∫lat_speed dt)
adjusted_bearing = bearing_to_B + crab_deg
yaw_error = adjusted_bearing − heading
```

| Parameter | Value | Notes |
|---|---|---|
| `crab_kp` | 5.0 | lateral m/s → degrees of heading offset |
| `crab_ki` | 1.0 | builds steady-wind feedforward within 5–10 s |
| `crab_max_deg` | 35° | limits extreme upwind pointing |

### Yaw Controller (P)

```
yaw_pwm = neutral_yaw + Kp_yaw * normalize(adjusted_bearing − heading)
```

| Parameter | Value |
|---|---|
| `Kp_yaw` | 2.5 |
| Clamp (cruise / approach / landing) | ±200 / ±150 / ±150 PWM |

### Pitch — Forward Force

**ENROUTE (cruise):** `+1200 PWM` from neutral, ramped over 8 s.
When misaligned, pitch is reduced to bleed forward speed down to ≈0.2 m/s until heading catches up.

**APPROACH (fine):** Speed-targeted PI — tapers desired track speed from
2 m/s (at 18 m) to 0 m/s (at 10 m); speed error feeds into pitch delta.

### Takeoff Pitch Correction (PI)

During climb, lateral drift toward or away from Point B is corrected by a PI
controller on `track_h_speed` (projection of velocity onto A→B bearing):

```
pitch_corr = Kp_pitch * (-track_h_speed) + Ki_pitch * ∫(-track_h_speed) dt
```

| Parameter | Value |
|---|---|
| `takeoff_pitch_kp` | 200.0 PWM / (m/s) |
| `takeoff_pitch_ki` | 40.0 PWM / m |
| `takeoff_pitch_correction_max` | 300 PWM |

## Running Tests

```bash
# All unit tests (no SITL required)
pytest tests/ -v

# Quick smoke check
pytest tests/test_stage3_fsm.py -v
```

## Stage 2 backward compatibility

Running without `--sim-only` activates the legacy Stage 2 stub FSM
(`mission_fsm.py`) unchanged.

```bash
# Stage 2 dry-run (no --sim-only)
python -m autopilot.main --connect tcp:127.0.0.1:5762 --dry-run
```

Stage 2 FSM states: `INIT → READY → ARM_TAKEOFF_STUB → NAVIGATE_STUB → LAND_STUB → COMPLETE / ABORT`

## Known limitations / future work

1. **Roll cross-track correction** — `LateralController` outputs a crab-angle
   (yaw offset) rather than direct roll; direct roll-based station-keeping
   during landing is not yet implemented.
2. **Landing position accuracy** — GPS-relative pitch/roll corrections during
   LANDING to station-hold over exact B coordinates are not implemented.
3. **Speed-based pitch scheduling** in ENROUTE — cruise pitch is currently
   fixed at +1200 PWM; a ground-speed target controller would prevent
   overshoot on short segments.
4. **Wind/disturbance rejection in heading-hold** — the yaw P controller has
   no integral term; steady crosswind requires the crab-angle controller to
   compensate indirectly.
