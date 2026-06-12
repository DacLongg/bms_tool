from __future__ import annotations

import logging

from bms_tool_app.logging_config import configure_logging


logger = logging.getLogger(__name__)


def run() -> None:
    log_file = configure_logging()
    logger.info("Starting BMS UART Tool. Log file: %s", log_file)
    try:
        from bms_tool_app.gui import main

        main()
    except Exception:
        logger.exception("BMS UART Tool crashed")
        raise
    finally:
        logger.info("BMS UART Tool exited")


if __name__ == "__main__":
    run()
