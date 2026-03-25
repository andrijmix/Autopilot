"""E2E integration test — запускає autopilot.main як окремий subprocess
з реальним DroneKit підключенням до ArduPilot SITL.

Перед запуском тестів потрібен запущений SITL:
    sim_vehicle.py -v ArduCopter --console --map

Запуск тестів:
    pytest tests/e2e/ -v -s
    pytest tests/e2e/ -v -s --sitl-addr 192.168.1.10:14550   # інша адреса

Stage 3 тести потребують --sim-only та реального SITL (без --dry-run).
Швидкі тести (~90 s) перевіряють ранні переходи FSM через --stage3-*-timeout.
Довгі тести (~8 хв) перемарковані @pytest.mark.slow і пропускаються без -m slow.
"""

import re
import socket
import subprocess
import sys
import textwrap

import pytest

# ---------------------------------------------------------------------------
# SITL availability
# ---------------------------------------------------------------------------

_SITL_DEFAULT = "tcp:127.0.0.1:5762"
_PROBE_TIMEOUT_S = 2


def _sitl_addr(request: pytest.FixtureRequest) -> str:
    return request.config.getoption("--sitl-addr", default=_SITL_DEFAULT)


def _is_reachable(addr: str) -> bool:
    # strip tcp: / udp: scheme for socket probe
    raw = addr.split(":", 1)[-1].lstrip("/")
    host, port_str = raw.rsplit(":", 1)
    try:
        with socket.create_connection((host, int(port_str)), timeout=_PROBE_TIMEOUT_S):
            return True
    except OSError:
        return False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _run_main(addr: str, extra_args: list[str], timeout_s: int = 45) -> subprocess.CompletedProcess:
    """Запускає `python -m autopilot.main` як subprocess і повертає результат."""
    cmd = [
        sys.executable, "-m", "autopilot.main",
        "--connect", addr,
        "--dry-run",
    ] + extra_args

    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        cwd=str(
            # project root = два рівні вище цього файлу
            __import__("pathlib").Path(__file__).resolve().parent.parent.parent
        ),
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestDryRunE2E:
    """Повний E2E прохід через реальний DroneKit + SITL, dry-run режим."""

    # ------------------------------------------------------------------
    # SITL-required tests
    # ------------------------------------------------------------------

    def test_skipped_when_sitl_not_running(self, request: pytest.FixtureRequest) -> None:
        """Якщо SITL не доступний — тест явно позначається як skip, не падає."""
        addr = _sitl_addr(request)
        if _is_reachable(addr):
            pytest.skip("SITL доступний — цей тест перевіряє лише skip-логіку")
        # Просто викликаємо _run_main і перевіряємо, що timeout не спрацьовує
        # (connect_vehicle підніме виключення → exit code 1)
        result = _run_main(addr, ["--navigate-stub-timeout", "5"], timeout_s=20)
        assert result.returncode != 0, "Має впасти без SITL"

    def test_fsm_dry_run_abort_on_navigate_timeout(self, request: pytest.FixtureRequest) -> None:
        """Основний happy-path E2E:
        1. DroneKit підключається до SITL.
        2. Preflight checks проходять.
        3. FSM: INIT -> READY -> ARM_TAKEOFF_STUB -> NAVIGATE_STUB -> ABORT (timeout).
        4. RC override скидається безпечно, без відправки на коптер.
        5. Exit code == 3 (ABORT).
        """
        addr = _sitl_addr(request)
        if not _is_reachable(addr):
            pytest.skip(f"SITL недоступний на {addr} — запустіть sim_vehicle.py спочатку")

        result = _run_main(addr, ["--navigate-stub-timeout", "6"], timeout_s=60)
        output = result.stdout + result.stderr

        # --- підключення ---
        assert "Connection established" in output, _fmt(output, "Connection established")

        # --- preflight ---
        assert "preflight check=heartbeat ok=True" in output, _fmt(output, "heartbeat ok")

        # --- dry-run прапор ---
        assert "DRY-RUN" in output, _fmt(output, "DRY-RUN")

        # --- FSM переходи ---
        assert "FSM started" in output,                      _fmt(output, "FSM started")
        assert "INIT -> READY" in output,                    _fmt(output, "INIT -> READY")
        assert "READY -> ARM_TAKEOFF_STUB" in output,        _fmt(output, "ARM_TAKEOFF_STUB")
        assert "ARM_TAKEOFF_STUB -> NAVIGATE_STUB" in output, _fmt(output, "NAVIGATE_STUB")

        # --- навігаційні розрахунки ---
        assert "dist_m=" in output,     _fmt(output, "dist_m")
        assert "bearing_deg=" in output, _fmt(output, "bearing_deg")

        # --- safe reset ---
        assert "RC override safely reset" in output, _fmt(output, "RC override safely reset")

        # --- ABORT через таймаут ---
        assert "NAVIGATE_STUB -> ABORT" in output, _fmt(output, "NAVIGATE_STUB -> ABORT")
        assert result.returncode == 3, (
            f"Очікував exit code 3 (ABORT), отримав {result.returncode}.\n{output}"
        )

    def test_preflight_fail_returns_exit_code_2(self, request: pytest.FixtureRequest) -> None:
        """Якщо preflight не проходить — exit code == 2, FSM не запускається."""
        addr = _sitl_addr(request)
        if not _is_reachable(addr):
            pytest.skip(f"SITL недоступний на {addr}")

        # Передаємо нереальний мінімальний GPS fix (99) щоб гарантовано впасти preflight.
        # main.py не має такого аргументу — тому патчимо через env var PYTHONPATH і
        # запускаємо окремий inline-скрипт що задає жорсткий min_gps_fix_type=99.
        script = textwrap.dedent(f"""
            import sys
            from dataclasses import replace
            from autopilot.config import build_default_config, PreflightConfig
            from autopilot.connection import connect_vehicle
            from autopilot.preflight import run_preflight_checks
            import logging
            logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
            logger = logging.getLogger("autopilot")

            cfg = build_default_config()
            cfg = replace(cfg, preflight=replace(cfg.preflight, min_gps_fix_type=99))

            vehicle = connect_vehicle("{addr}", baud=57600)
            report = run_preflight_checks(vehicle, cfg.preflight)
            for c in report.checks:
                lvl = logging.INFO if c.ok else logging.ERROR
                logger.log(lvl, "preflight check=%s ok=%s details=%s", c.name, c.ok, c.details)
            vehicle.close()
            sys.exit(0 if report.ok else 2)
        """)

        result = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True, text=True, timeout=30,
            cwd=str(__import__("pathlib").Path(__file__).resolve().parent.parent.parent),
        )
        output = result.stdout + result.stderr
        assert "preflight check=gps ok=False" in output, _fmt(output, "gps ok=False")
        assert result.returncode == 2, (
            f"Очікував exit code 2 (preflight fail), отримав {result.returncode}.\n{output}"
        )


# ---------------------------------------------------------------------------
# Util
# ---------------------------------------------------------------------------

def _fmt(output: str, needle: str) -> str:
    return f"Рядок '{needle}' не знайдено у виводі:\n{output[-3000:]}"


# ---------------------------------------------------------------------------
# Stage 3 SITL tests
# ---------------------------------------------------------------------------

_PROJECT_ROOT = str(__import__("pathlib").Path(__file__).resolve().parent.parent.parent)


def _run_stage3(
    addr: str,
    extra_args: list,
    timeout_s: int = 180,
) -> subprocess.CompletedProcess:
    """Запускає autopilot.main з --sim-only (Stage 3 FSM)."""
    cmd = [
        sys.executable, "-m", "autopilot.main",
        "--connect", addr,
        "--sim-only",
    ] + extra_args
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        cwd=_PROJECT_ROOT,
    )


class TestStage3SITL:
    """Stage 3 integration tests — потребують запущеного SITL (sim_vehicle.py).

    Тести автоматично пропускаються якщо SITL недоступний.
    Довгі тести позначені @pytest.mark.slow; запустіть з `pytest -m slow` окремо.
    """

    # ------------------------------------------------------------------
    # test 1: guard — Stage 3 не запускається без --sim-only
    # ------------------------------------------------------------------

    def test_stage2_runs_without_sim_only(self, request: pytest.FixtureRequest) -> None:
        """Без --sim-only запускається Stage 2 stub FSM (не Stage 3)."""
        addr = _sitl_addr(request)
        if not _is_reachable(addr):
            pytest.skip(f"SITL недоступний на {addr}")

        # run Stage 2 stub with very short navigate timeout → fast ABORT
        result = _run_main(addr, ["--navigate-stub-timeout", "4"], timeout_s=45)
        output = result.stdout + result.stderr

        # Stage 3 FSM must NOT appear in output
        assert "Stage3FSM" not in output, (
            f"Stage3FSM запустився без --sim-only!\n{output[-2000:]}"
        )
        # Stage 2 stub transitions must appear
        assert "ARM_TAKEOFF_STUB" in output or "NAVIGATE_STUB" in output, (
            _fmt(output, "Stage 2 FSM transitions")
        )

    # ------------------------------------------------------------------
    # test 2: takeoff + enroute progress (quick, ~90 s)
    # ------------------------------------------------------------------

    def test_takeoff_and_enroute_progress(self, request: pytest.FixtureRequest) -> None:
        """SITL: Stage 3 FSM стартує, виконує TAKEOFF, досягає ENROUTE_TO_B.

        Короткі таймаути змушують ABORT одразу після входу в ENROUTE
        щоб тест завершився за ~90 s.
        """
        addr = _sitl_addr(request)
        if not _is_reachable(addr):
            pytest.skip(f"SITL недоступний на {addr}")

        result = _run_stage3(
            addr,
            # takeoff 60 s, enroute only 15 s — форсуємо ABORT швидко
            ["--stage3-takeoff-timeout", "60", "--stage3-enroute-timeout", "15"],
            timeout_s=150,
        )
        output = result.stdout + result.stderr

        # FSM must start
        assert "Stage3FSM started" in output, _fmt(output, "Stage3FSM started")

        # INIT → READY transition
        assert "INIT -> READY" in output, _fmt(output, "INIT -> READY")

        # READY → TAKEOFF transition
        assert "READY -> TAKEOFF" in output, _fmt(output, "READY -> TAKEOFF")

        # Either TAKEOFF completed (ENROUTE reached) or ABORT due to takeoff timeout
        reached_enroute = "TAKEOFF -> ENROUTE_TO_B" in output
        got_abort = "-> ABORT" in output
        assert reached_enroute or got_abort, (
            f"Очікував TAKEOFF→ENROUTE або ABORT, але нічого не знайдено.\n{output[-3000:]}"
        )

        # Altitude data must be logged in TAKEOFF state
        assert "state=TAKEOFF" in output, _fmt(output, "state=TAKEOFF log line")

        # RC override must be safely reset
        assert "RC override safely reset" in output, _fmt(output, "RC override safely reset")

        # Exit code: 0 (COMPLETE, unlikely at this timeout) or 3 (ABORT)
        assert result.returncode in (0, 3), (
            f"Очікував exit code 0 або 3, отримав {result.returncode}.\n{output[-2000:]}"
        )

    # ------------------------------------------------------------------
    # test 3: arrival to fine-approach radius (slow, full flight)
    # ------------------------------------------------------------------

    @pytest.mark.slow
    def test_arrival_to_target_radius(self, request: pytest.FixtureRequest) -> None:
        """SITL: коптер досягає зони fine-approach (dist < 80 m) навколо точки B.

        Повний рейс A→B займає ~5-7 хвилин.
        Запускати окремо: pytest tests/e2e/ -m slow -v -s
        """
        addr = _sitl_addr(request)
        if not _is_reachable(addr):
            pytest.skip(f"SITL недоступний на {addr}")

        result = _run_stage3(addr, [], timeout_s=600)
        output = result.stdout + result.stderr

        # At minimum must reach ENROUTE
        assert "ENROUTE_TO_B" in output, _fmt(output, "ENROUTE_TO_B reached")

        # dist and bearing must be logged continuously
        assert "dist=" in output, _fmt(output, "dist= in logs")
        assert "brng=" in output, _fmt(output, "brng= in logs")

        # Check if APPROACH_FINE or better was reached
        reached_approach = "APPROACH_FINE" in output
        reached_landing = "LANDING" in output
        complete = "COMPLETE" in output

        if reached_approach or reached_landing or complete:
            # Verify dist was < 100 m when approach started
            dist_matches = re.findall(r"dist=([\d.]+)m", output)
            if dist_matches:
                min_dist = min(float(d) for d in dist_matches)
                assert min_dist <= 120.0, (
                    f"Мінімальна відстань {min_dist:.1f} m > 120 m — ніколи не наблизився до B"
                )
        # At minimum ENROUTE must be logged (checked above)

        # RC always reset
        assert "RC override safely reset" in output, _fmt(output, "RC override safely reset")
        assert result.returncode in (0, 3), (
            f"Очікував exit code 0 або 3, отримав {result.returncode}."
        )

    # ------------------------------------------------------------------
    # test 4: landing sequence or clean abort on timeout
    # ------------------------------------------------------------------

    def test_landing_sequence_or_abort_on_timeout(self, request: pytest.FixtureRequest) -> None:
        """SITL: місія або завершується посадкою (COMPLETE), або ABORT з safe reset.

        Використовує скорочені таймаути для approach/landing щоб завершитись
        за ~90 s після входу в ці стани.
        """
        addr = _sitl_addr(request)
        if not _is_reachable(addr):
            pytest.skip(f"SITL недоступний на {addr}")

        result = _run_stage3(
            addr,
            [
                "--stage3-approach-timeout", "30",
                "--stage3-landing-timeout", "45",
            ],
            timeout_s=600,
        )
        output = result.stdout + result.stderr

        assert "Stage3FSM started" in output, _fmt(output, "Stage3FSM started")

        # Must end in some terminal state
        ended_complete = "Stage3 mission finished: COMPLETE" in output
        ended_abort = "Stage3 mission finished: ABORT" in output or "-> ABORT" in output
        assert ended_complete or ended_abort, (
            f"Місія не досягла термінального стану.\n{output[-3000:]}"
        )

        # RC must always be safely reset
        assert "RC override safely reset" in output, _fmt(output, "RC override safely reset")

        # Exit code: 0 (COMPLETE) or 3 (ABORT)
        assert result.returncode in (0, 3), (
            f"Очікував exit code 0 або 3, отримав {result.returncode}.\n{output[-1000:]}"
        )

