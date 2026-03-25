import pytest

SITL_DEFAULT = "tcp:127.0.0.1:5762"


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--sitl-addr",
        default=SITL_DEFAULT,
        help=f"MAVLink address of running SITL (default: {SITL_DEFAULT})",
    )
