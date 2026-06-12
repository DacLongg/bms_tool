# BMS UART Tool

Python desktop tool for the UART protocol in `reference/bms_uart.c`.

## Run

```powershell
python -m pip install -r requirements.txt
python run_bms_tool.py
```

Default baud rate is `115200`; change it in the UI if the firmware UART uses a
different speed.

Launcher and runtime logs are written to `logs\bms_tool_YYYYMMDD_HHMMSS.log`.
Set `BMS_TOOL_LOG_LEVEL=DEBUG` before running to include raw UART TX/RX frames.

## Features

- Connect to a COM port with pyserial.
- Read cells, faults, limits, and summary data.
- Auto-poll at a configurable interval.
- Send current calibration command (`CMD_CALIBRATE_CURRENT`, `0x30`).
- Run OTP check/read/write commands (`0x20`, `0x22`, `0x21`).
- Send a custom command byte with custom hex payload.

## Firmware note

The reference header sets `BMS_UART_MAX_PAYLOAD_SIZE` to `64`, while the current
`READ_SUMMARY` handler sends `sizeof(BMS_Tracking_t)`. That struct is larger
than 64 bytes on normal STM32/GCC layouts, so the firmware can reply
`INTERNAL_ERROR` for `READ_SUMMARY`. The app keeps working with `READ_CELLS`,
`READ_FAULTS`, and `READ_LIMITS`, and will parse summary payloads if the
firmware is changed to use the compact commented-out summary or a larger max
payload.
