"""Logging setup for the BMS UART desktop tool."""

from __future__ import annotations

from datetime import datetime
import logging
import os
from pathlib import Path


_LOG_FILE: Path | None = None


def _default_log_file() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parents[1] / "logs" / f"bms_tool_{timestamp}.log"


def configure_logging(log_file: str | os.PathLike[str] | None = None) -> Path:
    """Configure process-wide file logging and return the active log path."""

    global _LOG_FILE
    if _LOG_FILE is not None:
        return _LOG_FILE

    configured_log_file = log_file or os.environ.get("BMS_TOOL_LOG_FILE")
    _LOG_FILE = Path(configured_log_file) if configured_log_file else _default_log_file()
    _LOG_FILE.parent.mkdir(parents=True, exist_ok=True)

    level_name = os.environ.get("BMS_TOOL_LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)

    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        handlers=[logging.FileHandler(_LOG_FILE, encoding="utf-8")],
    )
    logging.captureWarnings(True)
    logging.getLogger(__name__).info("Logging initialized: %s", _LOG_FILE)
    return _LOG_FILE
