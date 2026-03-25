"""E2E integration test — запускає autopilot.main як окремий subprocess
з реальним DroneKit підключенням до ArduPilot SITL.

Перед запуском тестів потрібен запущений SITL:
    sim_vehicle.py -v ArduCopter --console --map

Запуск тестів:
    pytest tests/e2e/ -v -s
    pytest tests/e2e/ -v -s --sitl-addr 192.168.1.10:14550   # інша адреса
"""

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
