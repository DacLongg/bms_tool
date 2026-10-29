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
- Read current, compact status/SOC, pack voltage, cell count, TS1 temperature,
  and completed cycle count (`0x14`-`0x19`).
- Auto-poll at a configurable interval.
- Enable or disable the balancing algorithm (`0x31`).
- Send current calibration command (`CMD_CALIBRATE_CURRENT`, `0x30`).
- Run OTP check/read/write commands (`0x20`, `0x22`, `0x21`).
- Send a custom command byte with custom hex payload.

## Firmware note

The implementation follows `BMS_UART_Protocol_For_Partner.md` v1.3. The
current `READ_SUMMARY` response contains the documented 124-byte GCC/ARM
`BMS_Tracking_t` layout. Legacy 152/160-byte captures remain readable, but new
integrations should prefer the stable dedicated read commands (`0x11`-`0x19`).
