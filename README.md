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

The reference header now sets `BMS_UART_MAX_PAYLOAD_SIZE` to `160`, and the
current `READ_SUMMARY` handler sends raw `sizeof(BMS_Tracking_t)`. The app
parses the current 152-byte short-enum layout and also keeps compatibility with
the 160-byte normal-enum layout if the firmware build options change.
