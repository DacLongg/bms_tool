"""Tkinter desktop application for the BMS UART protocol."""

from __future__ import annotations

import logging
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable, Dict, Optional

from . import protocol
from .serial_client import (
    BmsSerialClient,
    SerialDependencyError,
    list_serial_ports,
)
from .logging_config import configure_logging


logger = logging.getLogger(__name__)
SUMMARY_POLL_INTERVAL_MS = 5000


def _fmt_bool(value: object) -> str:
    if value is None:
        return "-"
    return "Yes" if bool(value) else "No"


def _fmt_mv(value: object) -> str:
    if value is None:
        return "-"
    return f"{int(value)} mV  ({int(value) / 1000:.3f} V)"


def _fmt_ma(value: object) -> str:
    if value is None:
        return "-"
    return f"{int(value)} mA  ({int(value) / 1000:.3f} A)"


def _fmt_hex(value: object, width: int = 4) -> str:
    if value is None:
        return "-"
    return f"0x{int(value):0{width}X}"


class BmsToolApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.log_file = configure_logging()
        self.title("BMS UART Tool")
        self.geometry("1120x760")
        self.minsize(980, 660)

        self.client = BmsSerialClient()
        self.events: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.worker_busy = False
        self.summary_poll_after_id: Optional[str] = None
        self.auto_poll = tk.BooleanVar(value=False)
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.interval_var = tk.IntVar(value=1000)
        self.status_var = tk.StringVar(value="Disconnected")
        self.summary_note_var = tk.StringVar(value="")
        self.last_summary: Dict[str, object] = {}
        self.last_limits: Optional[Dict[str, object]] = None
        self._fault_labels: Dict[str, ttk.Label] = {}
        self._overview_vars: Dict[str, tk.StringVar] = {}
        self._limit_vars: Dict[str, tk.StringVar] = {}
        self._otp_vars: Dict[str, tk.StringVar] = {}
        self._calibration_vars: Dict[str, tk.StringVar] = {}

        self._build_style()
        self._build_ui()
        self._log(f"Log file: {self.log_file}")
        self.refresh_ports()
        self.after(100, self._drain_events)
        self._schedule_summary_poll()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        logger.info("GUI initialized")

    def _build_style(self) -> None:
        style = ttk.Style(self)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Title.TLabel", font=("Segoe UI", 14, "bold"))
        style.configure("Metric.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("Danger.TLabel", foreground="#B00020")
        style.configure("Ok.TLabel", foreground="#0A7A33")
        style.configure("Warn.TLabel", foreground="#B35A00")
        style.configure("Mono.TLabel", font=("Consolas", 10))
        style.configure("Treeview", rowheight=26)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)

        self._build_toolbar(root)

        notebook = ttk.Notebook(root)
        notebook.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        self._build_overview_tab(notebook)
        self._build_cells_tab(notebook)
        self._build_faults_tab(notebook)
        self._build_controls_tab(notebook)
        self._build_log_tab(notebook)

        status_bar = ttk.Frame(root)
        status_bar.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        status_bar.columnconfigure(0, weight=1)
        ttk.Label(status_bar, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Label(status_bar, textvariable=self.summary_note_var, style="Warn.TLabel").grid(
            row=0, column=1, sticky="e"
        )

    def _build_toolbar(self, parent: ttk.Frame) -> None:
        bar = ttk.Frame(parent)
        bar.grid(row=0, column=0, sticky="ew")
        for col in (1, 3, 5):
            bar.columnconfigure(col, weight=1)

        ttk.Label(bar, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(6, 10))
        ttk.Button(bar, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=(0, 10)
        )

        ttk.Label(bar, text="Baud").grid(row=0, column=3, sticky="e")
        self.baud_combo = ttk.Combobox(
            bar,
            textvariable=self.baud_var,
            values=("2400", "9600", "19200", "38400", "57600", "115200", "230400"),
            width=10,
        )
        self.baud_combo.grid(row=0, column=4, padx=(6, 10))

        self.connect_button = ttk.Button(bar, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, sticky="e", padx=(0, 10))

        ttk.Button(bar, text="Ping", command=self.ping).grid(row=0, column=6, padx=(0, 6))
        ttk.Button(bar, text="Read Once", command=self.read_once).grid(
            row=0, column=7, padx=(0, 10)
        )
        ttk.Checkbutton(
            bar,
            text="Auto",
            variable=self.auto_poll,
            command=self._auto_poll_changed,
        ).grid(row=0, column=8, padx=(0, 6))
        ttk.Spinbox(
            bar,
            from_=200,
            to=10000,
            increment=100,
            width=7,
            textvariable=self.interval_var,
        ).grid(row=0, column=9)
        ttk.Label(bar, text="ms").grid(row=0, column=10, padx=(4, 0))

    def _build_overview_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="Overview")
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        left = ttk.LabelFrame(tab, text="Pack", padding=10)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        right = ttk.LabelFrame(tab, text="Runtime", padding=10)
        right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        pack_fields = [
            ("state_name", "State"),
            ("current_direction_name", "Current direction"),
            ("stack_voltage_mV", "Stack voltage"),
            ("pack_voltage_mV", "Pack voltage"),
            ("bat_adc_estimated_pack_mV", "BAT ADC estimate"),
            ("current_mA", "Current"),
            ("min_cell_voltage_mV", "Min cell"),
            ("max_cell_voltage_mV", "Max cell"),
            ("average_cell_voltage_mV", "Average cell"),
            ("delta_cell_voltage_mV", "Delta cell"),
            ("temperature_C", "Temperatures"),
        ]
        runtime_fields = [
            ("initialized", "Initialized"),
            ("connected", "BQ connected"),
            ("charging", "Charging"),
            ("discharging", "Discharging"),
            ("fets", "FET flags"),
            ("balance_required", "Balance required"),
            ("balance_mask", "Balance mask"),
            ("alert_active", "Alert active"),
            ("alert_counter", "Alert counter"),
            ("circle_counter", "Loop counter"),
            ("current_calibration_gain_ppm", "Current cal gain"),
        ]
        self._add_metric_grid(left, pack_fields)
        self._add_metric_grid(right, runtime_fields)

        self.raw_summary_text = tk.Text(tab, height=8, wrap="word")
        self.raw_summary_text.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(12, 0))
        tab.rowconfigure(1, weight=1)
        self.raw_summary_text.insert("end", "Raw or unknown summary payload will appear here.\n")
        self.raw_summary_text.configure(state="disabled")

    def _add_metric_grid(self, parent: ttk.Frame, fields: list[tuple[str, str]]) -> None:
        parent.columnconfigure(1, weight=1)
        for row, (key, label) in enumerate(fields):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=3)
            value_var = tk.StringVar(value="-")
            self._overview_vars[key] = value_var
            ttk.Label(parent, textvariable=value_var, style="Metric.TLabel").grid(
                row=row, column=1, sticky="w", padx=(16, 0), pady=3
            )

    def _build_cells_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="Cells")
        tab.rowconfigure(0, weight=1)
        tab.columnconfigure(0, weight=1)

        columns = ("cell", "mv", "v", "balance")
        self.cells_tree = ttk.Treeview(tab, columns=columns, show="headings", height=14)
        self.cells_tree.heading("cell", text="Cell")
        self.cells_tree.heading("mv", text="Voltage mV")
        self.cells_tree.heading("v", text="Voltage V")
        self.cells_tree.heading("balance", text="Balance")
        self.cells_tree.column("cell", width=80, anchor="center")
        self.cells_tree.column("mv", width=160, anchor="e")
        self.cells_tree.column("v", width=160, anchor="e")
        self.cells_tree.column("balance", width=120, anchor="center")
        self.cells_tree.grid(row=0, column=0, sticky="nsew")

        scroll = ttk.Scrollbar(tab, orient=tk.VERTICAL, command=self.cells_tree.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.cells_tree.configure(yscrollcommand=scroll.set)

        self.cells_summary_var = tk.StringVar(value="-")
        ttk.Label(tab, textvariable=self.cells_summary_var, style="Metric.TLabel").grid(
            row=1, column=0, sticky="w", pady=(10, 0)
        )

    def _build_faults_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="Faults")
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        fault_box = ttk.LabelFrame(tab, text="Protection flags", padding=10)
        fault_box.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        gate_box = ttk.LabelFrame(tab, text="Gate and alert signals", padding=10)
        gate_box.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        for row, name in enumerate(protocol.FAULT_NAMES):
            ttk.Label(fault_box, text=name).grid(row=row, column=0, sticky="w", pady=3)
            label = ttk.Label(fault_box, text="-", style="Mono.TLabel")
            label.grid(row=row, column=1, sticky="e", padx=(16, 0), pady=3)
            self._fault_labels[name] = label

        for row, name in enumerate(protocol.GATE_SIGNAL_NAMES):
            ttk.Label(gate_box, text=name).grid(row=row, column=0, sticky="w", pady=3)
            label = ttk.Label(gate_box, text="-", style="Mono.TLabel")
            label.grid(row=row, column=1, sticky="e", padx=(16, 0), pady=3)
            self._fault_labels[name] = label

        self.fault_meta_var = tk.StringVar(value="-")
        ttk.Label(gate_box, textvariable=self.fault_meta_var, style="Metric.TLabel").grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(16, 0)
        )

        limits = ttk.LabelFrame(tab, text="Firmware limits", padding=10)
        limits.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(12, 0))
        for index, key in enumerate(
            [
                "cell_count",
                "thermistor_count",
                "cell_ov_cutoff_mV",
                "cell_ov_recover_mV",
                "cell_uv_cutoff_mV",
                "cell_uv_recover_mV",
                "balance_delta_mV",
                "balance_min_cell_mV",
                "over_current_mA",
                "short_circuit_mA",
                "charge_ot_cutoff_C",
                "discharge_ot_cutoff_C",
                "undertemp_cutoff_C",
                "nominal_capacity_mAh",
            ]
        ):
            row = index // 2
            col = (index % 2) * 2
            ttk.Label(limits, text=key).grid(row=row, column=col, sticky="w", pady=3)
            var = tk.StringVar(value="-")
            self._limit_vars[key] = var
            ttk.Label(limits, textvariable=var, style="Metric.TLabel").grid(
                row=row, column=col + 1, sticky="w", padx=(12, 28), pady=3
            )

    def _build_controls_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="Controls")
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        calibration = ttk.LabelFrame(tab, text="Current calibration", padding=10)
        calibration.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        ttk.Label(calibration, text="Actual current (mA)").grid(row=0, column=0, sticky="w")
        self.actual_current_var = tk.StringVar(value="0")
        ttk.Entry(calibration, textvariable=self.actual_current_var, width=16).grid(
            row=0, column=1, sticky="w", padx=(10, 0)
        )
        ttk.Button(calibration, text="Send Calibration", command=self.calibrate_current).grid(
            row=0, column=2, padx=(10, 0)
        )
        for row, key in enumerate(
            ["status_name", "actual_mA", "measured_mA", "deviation_ppm", "old_gain_ppm", "new_gain_ppm"],
            start=1,
        ):
            ttk.Label(calibration, text=key).grid(row=row, column=0, sticky="w", pady=3)
            var = tk.StringVar(value="-")
            self._calibration_vars[key] = var
            ttk.Label(calibration, textvariable=var, style="Metric.TLabel").grid(
                row=row, column=1, columnspan=2, sticky="w", padx=(10, 0), pady=3
            )

        otp = ttk.LabelFrame(tab, text="OTP", padding=10)
        otp.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        ttk.Button(otp, text="Check", command=self.otp_check).grid(row=0, column=0, sticky="w")
        ttk.Button(otp, text="Read", command=self.otp_read).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Button(otp, text="Write OTP", command=self.otp_write).grid(
            row=0, column=2, sticky="w", padx=(8, 0)
        )
        for row, key in enumerate(
            [
                "flag_names",
                "security_state",
                "check_result",
                "write_result",
                "battery_status_raw",
                "static_config_signature",
                "stack_voltage_mV",
                "pack_voltage_mV",
                "internal_temp_C",
                "reg0_config",
                "reg12_control",
                "da_config",
                "vcell_mode",
                "dchg_pin_config",
                "ddsg_pin_config",
                "dfetoff_pin_config",
            ],
            start=1,
        ):
            ttk.Label(otp, text=key).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="-")
            self._otp_vars[key] = var
            ttk.Label(otp, textvariable=var, style="Metric.TLabel", wraplength=430).grid(
                row=row, column=1, columnspan=2, sticky="w", padx=(10, 0), pady=2
            )

        custom = ttk.LabelFrame(tab, text="Custom frame", padding=10)
        custom.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        custom.columnconfigure(3, weight=1)
        ttk.Label(custom, text="Command hex").grid(row=0, column=0, sticky="w")
        self.custom_command_var = tk.StringVar(value="01")
        ttk.Entry(custom, textvariable=self.custom_command_var, width=8).grid(
            row=0, column=1, padx=(8, 16)
        )
        ttk.Label(custom, text="Payload hex").grid(row=0, column=2, sticky="w")
        self.custom_payload_var = tk.StringVar(value="")
        ttk.Entry(custom, textvariable=self.custom_payload_var).grid(
            row=0, column=3, sticky="ew", padx=(8, 8)
        )
        ttk.Button(custom, text="Send", command=self.send_custom_frame).grid(row=0, column=4)
        self.custom_response_var = tk.StringVar(value="-")
        ttk.Label(custom, textvariable=self.custom_response_var, style="Mono.TLabel").grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(8, 0)
        )

    def _build_log_tab(self, notebook: ttk.Notebook) -> None:
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="Log")
        tab.rowconfigure(0, weight=1)
        tab.columnconfigure(0, weight=1)
        self.log_text = tk.Text(tab, height=20, wrap="word")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(tab, orient=tk.VERTICAL, command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)
        self._log("BMS UART Tool ready.")

    def refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo.configure(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if not ports:
            self._log("No serial ports found.")
        else:
            self._log("Serial ports: " + ", ".join(ports))
        logger.info("Serial ports refreshed: %s", ", ".join(ports) if ports else "none")

    def toggle_connection(self) -> None:
        if self.client.is_open:
            self.auto_poll.set(False)
            self.client.close()
            self.connect_button.configure(text="Connect")
            self.status_var.set("Disconnected")
            self._log("Disconnected.")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("BMS UART Tool", "Select a serial port first.")
            self._log("Connect skipped: no serial port selected.", logging.WARNING)
            return
        try:
            baud = int(self.baud_var.get())
            self._log(f"Connecting to {port} at {baud} baud...")
            self.client.open(port, baud)
        except SerialDependencyError as exc:
            messagebox.showerror("Missing dependency", str(exc))
            self._log(f"Missing dependency: {exc}", logging.ERROR)
            return
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            self._log(f"Connect failed: {exc}", logging.ERROR)
            return

        self.connect_button.configure(text="Disconnect")
        self.status_var.set(f"Connected to {port} at {self.client.baudrate} baud")
        self._log(f"Connected to {port} at {self.client.baudrate} baud.")
        self._reschedule_summary_poll(0)

    def ping(self) -> None:
        self._run_worker("ping", lambda: self.client.ping(b"BMS"))

    def read_once(self) -> None:
        self._run_worker("read_all", self.client.read_all)

    def read_summary(self) -> None:
        self._run_worker("summary", self.client.read_summary)

    def otp_check(self) -> None:
        self._run_worker("otp", self.client.otp_check)

    def otp_read(self) -> None:
        self._run_worker("otp", self.client.otp_read)

    def otp_write(self) -> None:
        ok = messagebox.askyesno(
            "Confirm OTP write",
            "OTP write is permanent on the BQ device. Continue?",
        )
        if ok:
            self._run_worker("otp", self.client.otp_write)

    def calibrate_current(self) -> None:
        try:
            actual = int(self.actual_current_var.get())
        except ValueError:
            messagebox.showwarning("Invalid value", "Actual current must be an integer mA value.")
            return
        self._run_worker("calibration", lambda: self.client.calibrate_current(actual))

    def send_custom_frame(self) -> None:
        try:
            command = int(self.custom_command_var.get().strip(), 16)
            payload = protocol.from_hex(self.custom_payload_var.get())
        except ValueError as exc:
            messagebox.showwarning("Invalid frame", str(exc))
            return
        self._run_worker("custom", lambda: (command, self.client.request(command, payload)))

    def _auto_poll_changed(self) -> None:
        if self.auto_poll.get():
            self._schedule_auto_poll(0)

    def _schedule_auto_poll(self, delay_ms: Optional[int] = None) -> None:
        if delay_ms is None:
            delay_ms = max(200, int(self.interval_var.get()))
        self.after(delay_ms, self._auto_poll_tick)

    def _auto_poll_tick(self) -> None:
        if not self.auto_poll.get():
            return
        if self.client.is_open and not self.worker_busy:
            self.read_once()
        self._schedule_auto_poll()

    def _schedule_summary_poll(self, delay_ms: int = SUMMARY_POLL_INTERVAL_MS) -> None:
        if self.summary_poll_after_id is None:
            self.summary_poll_after_id = self.after(delay_ms, self._summary_poll_tick)

    def _reschedule_summary_poll(self, delay_ms: int = SUMMARY_POLL_INTERVAL_MS) -> None:
        if self.summary_poll_after_id is not None:
            try:
                self.after_cancel(self.summary_poll_after_id)
            except tk.TclError:
                pass
            self.summary_poll_after_id = None
        self._schedule_summary_poll(delay_ms)

    def _summary_poll_tick(self) -> None:
        self.summary_poll_after_id = None
        if self.client.is_open and not self.worker_busy:
            self.read_summary()
        elif self.client.is_open:
            logger.info("Summary poll skipped because serial worker is busy")
        self._schedule_summary_poll()

    def _run_worker(self, kind: str, func: Callable[[], object]) -> None:
        if not self.client.is_open:
            messagebox.showwarning("Not connected", "Connect to the BMS UART port first.")
            self._log(f"{kind} skipped: serial port is not open.", logging.WARNING)
            return
        if self.worker_busy:
            self._log("Serial request already in progress.", logging.WARNING)
            return
        self.worker_busy = True
        self.status_var.set("Working...")
        self._log(f"Starting {kind} request.")

        def target() -> None:
            try:
                result = func()
                self.events.put((kind, result))
            except Exception as exc:
                logger.exception("%s request failed", kind)
                self.events.put(("error", f"{kind}: {exc}"))
            finally:
                self.events.put(("worker_done", None))

        threading.Thread(target=target, daemon=True).start()

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "worker_done":
                    self.worker_busy = False
                    if self.client.is_open:
                        self.status_var.set(
                            f"Connected to {self.client.port} at {self.client.baudrate} baud"
                        )
                    else:
                        self.status_var.set("Disconnected")
                elif kind == "error":
                    self._log(f"ERROR: {payload}", logging.ERROR)
                    self.status_var.set(str(payload))
                elif kind == "ping":
                    self._log(f"Ping response: {protocol.to_hex(bytes(payload))}")
                elif kind == "summary":
                    self._update_summary(payload)
                    self.summary_note_var.set("")
                    self._log_summary(payload)
                elif kind == "read_all":
                    self._update_read_all(payload)
                elif kind == "otp":
                    self._update_otp(payload)
                    self._log("OTP response received.")
                elif kind == "calibration":
                    self._update_calibration(payload)
                    self._log("Calibration response received.")
                elif kind == "custom":
                    command, data = payload
                    self.custom_response_var.set(
                        f"CMD 0x{int(command):02X} response: {protocol.to_hex(bytes(data))}"
                    )
                    self._log(self.custom_response_var.get())
        except queue.Empty:
            pass
        self.after(100, self._drain_events)

    def _log_summary(self, summary: object) -> None:
        if not isinstance(summary, dict) or not summary:
            self._log("Summary updated.")
            return
        self._log(
            "Summary updated: state={state} pack={pack} current={current}".format(
                state=summary.get("state_name", "-"),
                pack=_fmt_mv(summary.get("pack_voltage_mV")),
                current=_fmt_ma(summary.get("current_mA")),
            )
        )

    def _update_read_all(self, data: Dict[str, object]) -> None:
        errors = data.get("errors", {})
        if isinstance(errors, dict) and errors:
            self._log(
                "Read warnings: " + "; ".join(f"{k}: {v}" for k, v in errors.items()),
                logging.WARNING,
            )

        summary = data.get("summary")
        cells = data.get("cells")
        faults = data.get("faults")
        limits = data.get("limits")

        if isinstance(summary, dict):
            self._update_summary(summary)
            self.summary_note_var.set("")
        elif isinstance(errors, dict) and "summary" in errors:
            self.summary_note_var.set("READ_SUMMARY unavailable")
            self._update_summary({})

        if isinstance(cells, dict):
            self._update_cells(cells)
        if isinstance(faults, dict):
            self._update_faults(faults)
        if isinstance(limits, dict):
            self._update_limits(limits)

        details: list[str] = []
        if isinstance(summary, dict) and summary:
            details.append(
                "state={state} pack={pack} current={current}".format(
                    state=summary.get("state_name", "-"),
                    pack=_fmt_mv(summary.get("pack_voltage_mV")),
                    current=_fmt_ma(summary.get("current_mA")),
                )
            )
        if isinstance(cells, dict):
            details.append(
                "cells={count} min={minv} max={maxv} delta={delta}".format(
                    count=cells.get("cell_count", "-"),
                    minv=_fmt_mv(cells.get("min_cell_voltage_mV")),
                    maxv=_fmt_mv(cells.get("max_cell_voltage_mV")),
                    delta=_fmt_mv(cells.get("delta_cell_voltage_mV")),
                )
            )
        if isinstance(faults, dict):
            active_faults = faults.get("faults", [])
            active_count = len(active_faults) if isinstance(active_faults, list) else 0
            details.append(
                f"faults={active_count} bitmap={_fmt_hex(faults.get('fault_bitmap'))}"
            )
        self._log("Read complete: " + " | ".join(details) if details else "Read complete.")

    def _update_summary(self, summary: Dict[str, object]) -> None:
        self.last_summary = summary

        def value_for(key: str) -> str:
            value = summary.get(key)
            if key.endswith("_mV"):
                return _fmt_mv(value)
            if key == "current_mA":
                return _fmt_ma(value)
            if key in {"initialized", "connected", "charging", "discharging", "balance_required", "alert_active"}:
                return _fmt_bool(value)
            if key == "temperature_C" and isinstance(value, list):
                return ", ".join(f"{temp} C" for temp in value)
            if key == "fets" and isinstance(value, list):
                return ", ".join(value) if value else "None"
            if key == "balance_mask":
                return _fmt_hex(value)
            if value is None:
                return "-"
            return str(value)

        for key, var in self._overview_vars.items():
            var.set(value_for(key))

        self.raw_summary_text.configure(state="normal")
        self.raw_summary_text.delete("1.0", "end")
        if summary:
            for key in sorted(summary):
                self.raw_summary_text.insert("end", f"{key}: {summary[key]}\n")
        else:
            self.raw_summary_text.insert(
                "end",
                "READ_SUMMARY did not return data. The reference firmware sends a raw "
                "BMS_Tracking_t while MAX_PAYLOAD_SIZE is 64, so it can return "
                "INTERNAL_ERROR until firmware summary is compacted or max payload is raised.\n",
            )
        self.raw_summary_text.configure(state="disabled")

    def _update_cells(self, cells: Dict[str, object]) -> None:
        for item in self.cells_tree.get_children():
            self.cells_tree.delete(item)
        voltages = cells.get("cell_voltages_mV", [])
        balance_mask = int(self.last_summary.get("balance_mask", 0) or 0)
        if isinstance(voltages, list):
            for index, mv in enumerate(voltages, start=1):
                balancing = "On" if balance_mask & (1 << (index - 1)) else "-"
                self.cells_tree.insert(
                    "",
                    "end",
                    values=(index, mv, f"{int(mv) / 1000:.3f}", balancing),
                )
        self.cells_summary_var.set(
            "Min: {minv}   Max: {maxv}   Avg: {avg} mV   Delta: {delta} mV".format(
                minv=cells.get("min_cell_voltage_mV", "-"),
                maxv=cells.get("max_cell_voltage_mV", "-"),
                avg=cells.get("average_cell_voltage_mV", "-"),
                delta=cells.get("delta_cell_voltage_mV", "-"),
            )
        )

    def _update_faults(self, faults: Dict[str, object]) -> None:
        active_faults = set(faults.get("faults", []))
        active_gates = set(faults.get("gate_signals", []))
        for name in protocol.FAULT_NAMES:
            label = self._fault_labels[name]
            active = name in active_faults
            label.configure(text="ACTIVE" if active else "OK", style="Danger.TLabel" if active else "Ok.TLabel")
        for name in protocol.GATE_SIGNAL_NAMES:
            label = self._fault_labels[name]
            active = name in active_gates
            label.configure(text="ACTIVE" if active else "OK", style="Warn.TLabel" if active else "Ok.TLabel")

        self.fault_meta_var.set(
            f"Fault bitmap {_fmt_hex(faults.get('fault_bitmap'))}   "
            f"Gate bitmap {_fmt_hex(faults.get('gate_signal_bitmap'), 2)}   "
            f"Alert {_fmt_bool(faults.get('alert_active'))}   "
            f"Counter {faults.get('alert_counter', '-')}"
        )

    def _update_limits(self, limits: Dict[str, object]) -> None:
        self.last_limits = limits
        for key, var in self._limit_vars.items():
            value = limits.get(key)
            if key.endswith("_mV"):
                var.set(_fmt_mv(value))
            elif key.endswith("_mA"):
                var.set(_fmt_ma(value))
            elif key.endswith("_C"):
                var.set("-" if value is None else f"{value} C")
            else:
                var.set("-" if value is None else str(value))

    def _update_otp(self, otp: Dict[str, object]) -> None:
        for key, var in self._otp_vars.items():
            value = otp.get(key)
            if isinstance(value, list):
                var.set(", ".join(value) if value else "None")
            elif key.endswith("_mV"):
                var.set(_fmt_mv(value))
            elif key.endswith("_raw") or key.endswith("_signature") or key.endswith("_config") or key.endswith("_mode"):
                var.set(_fmt_hex(value))
            elif key.endswith("_C"):
                var.set("-" if value is None else f"{value} C")
            elif value is None:
                var.set("-")
            else:
                var.set(str(value))
        logger.info(
            "OTP updated: flags=%s check_result=%s write_result=%s",
            _fmt_hex(otp.get("flags")),
            otp.get("check_result", "-"),
            otp.get("write_result", "-"),
        )

    def _update_calibration(self, result: Dict[str, object]) -> None:
        for key, var in self._calibration_vars.items():
            value = result.get(key)
            var.set("-" if value is None else str(value))
        logger.info(
            "Calibration updated: status=%s actual=%s measured=%s new_gain=%s",
            result.get("status_name", "-"),
            result.get("actual_mA", "-"),
            result.get("measured_mA", "-"),
            result.get("new_gain_ppm", "-"),
        )

    def _log(self, message: str, level: int = logging.INFO) -> None:
        logger.log(level, message)
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _on_close(self) -> None:
        self._log("Closing BMS UART Tool.")
        if self.summary_poll_after_id is not None:
            try:
                self.after_cancel(self.summary_poll_after_id)
            except tk.TclError:
                pass
            self.summary_poll_after_id = None
        self.auto_poll.set(False)
        self.client.close()
        self.destroy()


def main() -> None:
    configure_logging()
    app = BmsToolApp()
    app.mainloop()


if __name__ == "__main__":
    main()
