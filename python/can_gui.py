#!/usr/bin/env python3
"""CAN1/CAN2 GUI sender/receiver for ZLGCAN-compatible libcontrolcan.so.

Features:
- Configure user-defined CAN frames (template list)
- Send selected frame via CAN1 or CAN2
- Display live packets received by CAN1 and CAN2
"""

from __future__ import annotations

import csv
import ctypes
import queue
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


VCI_USBCAN2 = 4
STATUS_OK = 1
MAX_CAN_DATA_LEN = 8
MAX_RX_HISTORY = 20000


class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint),
        ("AccMask", ctypes.c_uint),
        ("Reserved", ctypes.c_uint),
        ("Filter", ctypes.c_ubyte),
        ("Timing0", ctypes.c_ubyte),
        ("Timing1", ctypes.c_ubyte),
        ("Mode", ctypes.c_ubyte),
    ]


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * MAX_CAN_DATA_LEN),
        ("Reserved", ctypes.c_ubyte * 3),
    ]


BAUD_TO_TIMING = {
    "1000K": (0x00, 0x14),
    "800K": (0x00, 0x16),
    "500K": (0x00, 0x1C),
    "250K": (0x01, 0x1C),
    "125K": (0x03, 0x1C),
    "100K": (0x04, 0x1C),
    "50K": (0x09, 0x1C),
}


DECODE_ALIAS_TO_BASE = {
    "bool": "bool",
    "boolean": "bool",
    "byte": "u8",
    "u8": "u8",
    "uint8": "u8",
    "unsigned8": "u8",
    "i8": "i8",
    "int8": "i8",
    "u16": "u16",
    "uint16": "u16",
    "unsigned16": "u16",
    "i16": "i16",
    "int16": "i16",
    "u32": "u32",
    "uint32": "u32",
    "unsigned": "u32",
    "uint": "u32",
    "i32": "i32",
    "int32": "i32",
    "int": "i32",
    "u64": "u64",
    "uint64": "u64",
    "unsigned64": "u64",
    "i64": "i64",
    "int64": "i64",
    "float": "f32",
    "f32": "f32",
    "double": "f64",
    "f64": "f64",
}

DECODE_BASE_INFO = {
    "u8": ("B", 1),
    "i8": ("b", 1),
    "u16": ("H", 2),
    "i16": ("h", 2),
    "u32": ("I", 4),
    "i32": ("i", 4),
    "u64": ("Q", 8),
    "i64": ("q", 8),
    "f32": ("f", 4),
    "f64": ("d", 8),
}


@dataclass
class MessageTemplate:
    name: str
    channel: int  # 0 for CAN1, 1 for CAN2
    frame_id: int
    extended: bool
    remote: bool
    dlc: int
    data: list[int]
    decode_spec: str = ""
    period_ms: int = 0


class CanGuiApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("CAN1/CAN2 Message Console")
        self.geometry("1200x760")
        self.minsize(1000, 650)

        self.api_lock = threading.Lock()
        self.rx_queue: queue.Queue[tuple] = queue.Queue()
        self.ui_event_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.rx_thread: threading.Thread | None = None
        self.periodic_stop_event = threading.Event()
        self.periodic_thread: threading.Thread | None = None
        self.periodic_running = False

        self.self_test_waiters_lock = threading.Lock()
        self.self_test_waiters: list[dict[str, object]] = []
        self.self_test_running = False

        self.connected = False
        self.templates_by_item: dict[str, MessageTemplate] = {}
        self.decode_specs_by_id: dict[int, str] = {}
        self.grouped_packets_by_id: dict[int, dict[str, object]] = {}
        self.group_tree_items: dict[int, str] = {}
        self.rx_records_can1: deque[dict[str, object]] = deque(maxlen=MAX_RX_HISTORY)
        self.rx_records_can2: deque[dict[str, object]] = deque(maxlen=MAX_RX_HISTORY)

        self.lib_path_var = tk.StringVar(value=self._default_lib_path())
        self.baud_var = tk.StringVar(value="125K")
        self.status_var = tk.StringVar(value="Disconnected")

        self.name_var = tk.StringVar(value="Frame1")
        self.channel_var = tk.StringVar(value="CAN1")
        self.id_var = tk.StringVar(value="0x100")
        self.extended_var = tk.IntVar(value=0)
        self.remote_var = tk.IntVar(value=0)
        self.dlc_var = tk.IntVar(value=8)
        self.data_var = tk.StringVar(value="01 02 03 04 05 06 07 08")
        self.decode_var = tk.StringVar(value="")
        self.period_ms_var = tk.IntVar(value=0)

        self.dll = None

        self._build_ui()
        self._set_connected_ui(False)
        self._on_remote_toggle()
        self.after(50, self._process_rx_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _default_lib_path(self) -> str:
        for candidate in self._library_candidates(preferred_path=None):
            if candidate.exists():
                return str(candidate)

        script_dir = Path(__file__).resolve().parent
        return str(script_dir.parent / "c" / "libcontrolcan.so")

    def _library_candidates(self, preferred_path: str | None) -> list[Path]:
        script_dir = Path(__file__).resolve().parent
        raw_candidates: list[Path] = []

        if preferred_path:
            raw_candidates.append(Path(preferred_path).expanduser())

        raw_candidates.extend(
            [
                script_dir.parent / "c" / "libcontrolcan.so",
                script_dir / "libcontrolcan.so",
                Path.cwd() / "c" / "libcontrolcan.so",
                Path.cwd() / "libcontrolcan.so",
            ]
        )

        deduped: list[Path] = []
        seen: set[str] = set()
        for candidate in raw_candidates:
            key = str(candidate.resolve(strict=False))
            if key in seen:
                continue
            seen.add(key)
            deduped.append(candidate)

        return deduped

    def _build_ui(self) -> None:
        container = ttk.Frame(self, padding=10)
        container.pack(fill=tk.BOTH, expand=True)

        self._build_connection_panel(container)
        self._build_message_panel(container)
        self._build_rx_panel(container)

        status_label = ttk.Label(
            container,
            textvariable=self.status_var,
            anchor=tk.W,
            relief=tk.SUNKEN,
            padding=(8, 4),
        )
        status_label.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))

    def _build_connection_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Device Connection", padding=10)
        frame.pack(fill=tk.X)

        ttk.Label(frame, text="Library path:").grid(row=0, column=0, sticky=tk.W)
        lib_entry = ttk.Entry(frame, textvariable=self.lib_path_var)
        lib_entry.grid(row=0, column=1, sticky=tk.EW, padx=6)

        ttk.Label(frame, text="Baud:").grid(row=0, column=2, sticky=tk.W)
        baud_combo = ttk.Combobox(
            frame,
            textvariable=self.baud_var,
            values=list(BAUD_TO_TIMING.keys()),
            state="readonly",
            width=8,
        )
        baud_combo.grid(row=0, column=3, sticky=tk.W, padx=(6, 12))

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.connect_device)
        self.connect_btn.grid(row=0, column=4, sticky=tk.EW)

        self.disconnect_btn = ttk.Button(frame, text="Disconnect", command=self.disconnect_device)
        self.disconnect_btn.grid(row=0, column=5, sticky=tk.EW, padx=(6, 0))

        self.self_test_btn = ttk.Button(frame, text="Self Test", command=self.run_self_test)
        self.self_test_btn.grid(row=0, column=6, sticky=tk.EW, padx=(6, 0))

        frame.columnconfigure(1, weight=1)

    def _build_message_panel(self, parent: ttk.Frame) -> None:
        outer = ttk.LabelFrame(parent, text="Message Configuration", padding=10)
        outer.pack(fill=tk.BOTH, expand=False, pady=(10, 0))

        form = ttk.Frame(outer)
        form.pack(fill=tk.X)

        ttk.Label(form, text="Name:").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(form, textvariable=self.name_var, width=18).grid(row=0, column=1, sticky=tk.W, padx=(6, 12))

        ttk.Label(form, text="Interface:").grid(row=0, column=2, sticky=tk.W)
        ttk.Combobox(
            form,
            textvariable=self.channel_var,
            values=["CAN1", "CAN2"],
            state="readonly",
            width=8,
        ).grid(row=0, column=3, sticky=tk.W, padx=(6, 12))

        ttk.Label(form, text="ID:").grid(row=0, column=4, sticky=tk.W)
        ttk.Entry(form, textvariable=self.id_var, width=12).grid(row=0, column=5, sticky=tk.W, padx=(6, 12))

        ttk.Checkbutton(form, text="Extended", variable=self.extended_var).grid(row=0, column=6, sticky=tk.W)
        ttk.Checkbutton(
            form,
            text="Remote",
            variable=self.remote_var,
            command=self._on_remote_toggle,
        ).grid(row=0, column=7, sticky=tk.W, padx=(10, 0))

        ttk.Label(form, text="DLC:").grid(row=1, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Spinbox(
            form,
            from_=0,
            to=8,
            textvariable=self.dlc_var,
            width=5,
        ).grid(row=1, column=1, sticky=tk.W, padx=(6, 12), pady=(8, 0))

        ttk.Label(form, text="Data (hex bytes):").grid(row=1, column=2, sticky=tk.W, pady=(8, 0))
        self.data_entry = ttk.Entry(form, textvariable=self.data_var)
        self.data_entry.grid(row=1, column=3, columnspan=5, sticky=tk.EW, padx=(6, 12), pady=(8, 0))

        ttk.Label(form, text="Decode spec:").grid(row=2, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Entry(form, textvariable=self.decode_var).grid(
            row=2,
            column=1,
            columnspan=5,
            sticky=tk.EW,
            padx=(6, 12),
            pady=(8, 0),
        )
        ttk.Label(form, text="e.g. count:u16, ok:bool, temp:float").grid(
            row=2,
            column=6,
            columnspan=2,
            sticky=tk.W,
            pady=(8, 0),
        )

        ttk.Label(form, text="Period (ms):").grid(row=1, column=6, sticky=tk.W, pady=(8, 0))
        ttk.Spinbox(
            form,
            from_=0,
            to=600000,
            textvariable=self.period_ms_var,
            width=10,
        ).grid(row=1, column=7, sticky=tk.W, pady=(8, 0))

        button_row = ttk.Frame(outer)
        button_row.pack(fill=tk.X, pady=(10, 0))

        self.save_btn = ttk.Button(button_row, text="Save Template", command=self.save_template)
        self.save_btn.pack(side=tk.LEFT)

        self.delete_btn = ttk.Button(button_row, text="Delete Selected", command=self.delete_template)
        self.delete_btn.pack(side=tk.LEFT, padx=(6, 0))

        self.send_btn = ttk.Button(button_row, text="Send Current", command=self.send_current)
        self.send_btn.pack(side=tk.LEFT, padx=(6, 0))

        self.send_selected_btn = ttk.Button(
            button_row,
            text="Send Selected Templates",
            command=self.send_selected_templates,
        )
        self.send_selected_btn.pack(side=tk.LEFT, padx=(6, 0))

        self.start_periodic_btn = ttk.Button(
            button_row,
            text="Start Periodic",
            command=self.start_periodic_tx,
        )
        self.start_periodic_btn.pack(side=tk.LEFT, padx=(6, 0))

        self.stop_periodic_btn = ttk.Button(
            button_row,
            text="Stop Periodic",
            command=self.stop_periodic_tx,
        )
        self.stop_periodic_btn.pack(side=tk.LEFT, padx=(6, 0))

        columns = ("name", "if", "id", "format", "type", "dlc", "period", "decode", "data")
        self.template_tree = ttk.Treeview(outer, columns=columns, show="headings", height=7)
        self.template_tree.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        headings = {
            "name": "Name",
            "if": "Interface",
            "id": "ID",
            "format": "Format",
            "type": "Type",
            "dlc": "DLC",
            "period": "Period (ms)",
            "decode": "Decode Spec",
            "data": "Data",
        }
        widths = {
            "name": 140,
            "if": 90,
            "id": 90,
            "format": 90,
            "type": 80,
            "dlc": 60,
            "period": 100,
            "decode": 220,
            "data": 220,
        }

        for key in columns:
            self.template_tree.heading(key, text=headings[key])
            self.template_tree.column(key, width=widths[key], anchor=tk.W)

        self.template_tree.bind("<<TreeviewSelect>>", self._on_template_selected)

        form.columnconfigure(3, weight=1)

    def _build_rx_panel(self, parent: ttk.Frame) -> None:
        outer = ttk.LabelFrame(parent, text="Received Packets", padding=10)
        outer.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        row = ttk.Frame(outer)
        row.pack(fill=tk.X)
        ttk.Button(row, text="Clear CAN1", command=self.clear_can1_log).pack(side=tk.LEFT)
        ttk.Button(row, text="Clear CAN2", command=self.clear_can2_log).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(row, text="Export CAN1 CSV", command=lambda: self.export_rx_csv(0)).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(row, text="Export CAN2 CSV", command=lambda: self.export_rx_csv(1)).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(row, text="Clear Grouped", command=self.clear_grouped_view).pack(side=tk.LEFT, padx=(6, 0))

        notebook = ttk.Notebook(outer)
        notebook.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        raw_tab = ttk.Frame(notebook)
        grouped_tab = ttk.Frame(notebook)
        notebook.add(raw_tab, text="Raw RX")
        notebook.add(grouped_tab, text="Grouped by CAN ID")

        logs = ttk.Frame(raw_tab)
        logs.pack(fill=tk.BOTH, expand=True)

        can1_frame = ttk.LabelFrame(logs, text="CAN1 RX")
        can1_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))
        self.rx_text_can1 = tk.Text(can1_frame, height=14, wrap="none")
        self.rx_text_can1.pack(fill=tk.BOTH, expand=True)

        can2_frame = ttk.LabelFrame(logs, text="CAN2 RX")
        can2_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 0))
        self.rx_text_can2 = tk.Text(can2_frame, height=14, wrap="none")
        self.rx_text_can2.pack(fill=tk.BOTH, expand=True)

        grouped_columns = (
            "id",
            "format",
            "type",
            "tx_count",
            "rx_count",
            "last_dir",
            "last_if",
            "dlc",
            "raw",
            "decoded",
            "last_seen",
        )
        self.group_tree = ttk.Treeview(grouped_tab, columns=grouped_columns, show="headings", height=14)
        self.group_tree.pack(fill=tk.BOTH, expand=True)

        grouped_headings = {
            "id": "CAN ID",
            "format": "Format",
            "type": "Type",
            "tx_count": "TX",
            "rx_count": "RX",
            "last_dir": "Last Dir",
            "last_if": "Last IF",
            "dlc": "DLC",
            "raw": "Last Raw Data",
            "decoded": "Decoded",
            "last_seen": "Last Seen",
        }
        grouped_widths = {
            "id": 90,
            "format": 70,
            "type": 70,
            "tx_count": 55,
            "rx_count": 55,
            "last_dir": 70,
            "last_if": 70,
            "dlc": 50,
            "raw": 180,
            "decoded": 320,
            "last_seen": 115,
        }

        for key in grouped_columns:
            self.group_tree.heading(key, text=grouped_headings[key])
            self.group_tree.column(key, width=grouped_widths[key], anchor=tk.W)

    def _set_connected_ui(self, connected: bool) -> None:
        self.connect_btn.configure(state=tk.DISABLED if connected else tk.NORMAL)
        self.disconnect_btn.configure(state=tk.NORMAL if connected else tk.DISABLED)
        self.self_test_btn.configure(state=tk.NORMAL if connected and not self.self_test_running else tk.DISABLED)

        for btn in (self.send_btn, self.send_selected_btn):
            btn.configure(state=tk.NORMAL if connected else tk.DISABLED)

        self.start_periodic_btn.configure(state=tk.NORMAL if connected and not self.periodic_running else tk.DISABLED)
        self.stop_periodic_btn.configure(state=tk.NORMAL if connected and self.periodic_running else tk.DISABLED)

    def _on_remote_toggle(self) -> None:
        if self.remote_var.get():
            self.data_entry.configure(state=tk.DISABLED)
        else:
            self.data_entry.configure(state=tk.NORMAL)

    def _load_library(self) -> ctypes.CDLL:
        errors: list[str] = []
        selected = self.lib_path_var.get().strip()

        dll = None
        loaded_path = None
        for candidate in self._library_candidates(preferred_path=selected):
            lib_path = candidate.resolve(strict=False)
            if not lib_path.exists():
                errors.append(f"{lib_path}: not found")
                continue

            try:
                dll = ctypes.cdll.LoadLibrary(str(lib_path))
                loaded_path = lib_path
                break
            except OSError as exc:
                errors.append(f"{lib_path}: {exc}")

        if dll is None or loaded_path is None:
            detail = "\n".join(errors) if errors else "No candidate paths checked"
            raise OSError(f"Unable to load libcontrolcan.so. Tried:\n{detail}")

        self.lib_path_var.set(str(loaded_path))

        dll.VCI_OpenDevice.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        dll.VCI_OpenDevice.restype = ctypes.c_uint

        dll.VCI_CloseDevice.argtypes = [ctypes.c_uint, ctypes.c_uint]
        dll.VCI_CloseDevice.restype = ctypes.c_uint

        dll.VCI_InitCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_INIT_CONFIG)]
        dll.VCI_InitCAN.restype = ctypes.c_uint

        dll.VCI_StartCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        dll.VCI_StartCAN.restype = ctypes.c_uint

        dll.VCI_ResetCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        dll.VCI_ResetCAN.restype = ctypes.c_uint

        dll.VCI_Transmit.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_CAN_OBJ), ctypes.c_uint]
        dll.VCI_Transmit.restype = ctypes.c_uint

        dll.VCI_Receive.argtypes = [
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.POINTER(VCI_CAN_OBJ),
            ctypes.c_uint,
            ctypes.c_int,
        ]
        dll.VCI_Receive.restype = ctypes.c_uint

        return dll

    def connect_device(self) -> None:
        if self.connected:
            return

        try:
            self.dll = self._load_library()
            timing0, timing1 = BAUD_TO_TIMING[self.baud_var.get()]

            cfg = VCI_INIT_CONFIG(
                AccCode=0x80000008,
                AccMask=0xFFFFFFFF,
                Reserved=0,
                Filter=0,
                Timing0=timing0,
                Timing1=timing1,
                Mode=0,
            )

            with self.api_lock:
                if self.dll.VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK:
                    raise RuntimeError("VCI_OpenDevice failed")

                for channel in (0, 1):
                    if self.dll.VCI_InitCAN(VCI_USBCAN2, 0, channel, ctypes.byref(cfg)) != STATUS_OK:
                        raise RuntimeError(f"VCI_InitCAN failed for CAN{channel + 1}")
                    if self.dll.VCI_StartCAN(VCI_USBCAN2, 0, channel) != STATUS_OK:
                        raise RuntimeError(f"VCI_StartCAN failed for CAN{channel + 1}")

            self.stop_event.clear()
            self.rx_thread = threading.Thread(target=self._rx_loop, name="can-rx", daemon=True)
            self.rx_thread.start()

            self.connected = True
            self._set_connected_ui(True)
            self._set_status(f"Connected ({self.baud_var.get()})")
        except Exception as exc:
            self._set_status("Connect failed")
            self._safe_close_device()
            messagebox.showerror("Connection Error", str(exc))

    def disconnect_device(self) -> None:
        if not self.connected:
            return

        self.stop_periodic_tx(wait=True)

        self.stop_event.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.5)

        self._safe_close_device()

        self.connected = False
        self._set_connected_ui(False)
        self._set_status("Disconnected")

    def _safe_close_device(self) -> None:
        if not self.dll:
            return

        dll = self.dll
        with self.api_lock:
            try:
                dll.VCI_ResetCAN(VCI_USBCAN2, 0, 0)
            except Exception:
                pass
            try:
                dll.VCI_ResetCAN(VCI_USBCAN2, 0, 1)
            except Exception:
                pass
            try:
                dll.VCI_CloseDevice(VCI_USBCAN2, 0)
            except Exception:
                pass

        self.dll = None

    def _rx_loop(self) -> None:
        assert self.dll is not None

        rx_buffers = {
            0: (VCI_CAN_OBJ * 512)(),
            1: (VCI_CAN_OBJ * 512)(),
        }

        while not self.stop_event.is_set():
            for channel in (0, 1):
                if self.stop_event.is_set():
                    break

                with self.api_lock:
                    received = int(
                        self.dll.VCI_Receive(
                            VCI_USBCAN2,
                            0,
                            channel,
                            rx_buffers[channel],
                            512,
                            0,
                        )
                    )

                if received > 0:
                    now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    for i in range(received):
                        frame = rx_buffers[channel][i]
                        payload = [int(frame.Data[j]) for j in range(int(frame.DataLen))]
                        self.rx_queue.put(
                            (
                                now,
                                channel,
                                int(frame.ID),
                                int(frame.ExternFlag),
                                int(frame.RemoteFlag),
                                int(frame.DataLen),
                                payload,
                                int(frame.TimeStamp),
                            )
                        )

            time.sleep(0.01)

    def _process_rx_queue(self) -> None:
        max_batch = 400
        count = 0
        while count < max_batch:
            try:
                item = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            now, channel, frame_id, ext, remote, dlc, payload, timestamp = item
            frame_type = "EXT" if ext else "STD"
            data_kind = "RTR" if remote else "DATA"
            data_text = " ".join(f"{byte:02X}" for byte in payload)
            line = (
                f"[{now}] ID=0x{frame_id:X} {frame_type} {data_kind} "
                f"DLC={dlc} DATA={data_text:<23} TS=0x{timestamp:08X}\n"
            )

            target = self.rx_text_can1 if channel == 0 else self.rx_text_can2
            target.insert(tk.END, line)
            target.see(tk.END)

            record = {
                "host_time": now,
                "channel": channel + 1,
                "id_hex": f"0x{frame_id:X}",
                "id_dec": frame_id,
                "format": frame_type,
                "type": data_kind,
                "dlc": dlc,
                "data": data_text,
                "decoded": "",
                "timestamp_hex": f"0x{timestamp:08X}",
                "timestamp_dec": timestamp,
            }
            try:
                record["decoded"] = self._decode_payload(frame_id, payload)
            except Exception as exc:
                record["decoded"] = f"decode-error: {exc}"

            if channel == 0:
                self.rx_records_can1.append(record)
            else:
                self.rx_records_can2.append(record)

            self._record_grouped_packet(
                direction="RX",
                channel=channel,
                frame_id=frame_id,
                extended=bool(ext),
                remote=bool(remote),
                dlc=dlc,
                payload=payload,
            )
            self._notify_self_test_waiters(channel, frame_id, bool(ext), bool(remote), dlc, payload)
            count += 1

        self._drain_ui_event_queue()

        self.after(50, self._process_rx_queue)

    def _drain_ui_event_queue(self) -> None:
        max_batch = 100
        count = 0

        while count < max_batch:
            try:
                kind, payload = self.ui_event_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "error":
                text = str(payload)
                self._set_status(text)
                messagebox.showerror("CAN Error", text)
            elif kind == "periodic_done":
                text = str(payload)
                self.periodic_running = False
                self._set_connected_ui(self.connected)
                self._set_status(text)
            elif kind == "self_test_result":
                text = str(payload)
                self.self_test_running = False
                self._set_connected_ui(self.connected)
                self._set_status(text)
                if text.startswith("PASS"):
                    messagebox.showinfo("Self Test", text)
                else:
                    messagebox.showwarning("Self Test", text)
            elif kind == "group_event":
                event = payload
                if isinstance(event, dict):
                    self._record_grouped_packet(
                        direction=str(event.get("direction", "RX")),
                        channel=int(event.get("channel", 0)),
                        frame_id=int(event.get("frame_id", 0)),
                        extended=bool(event.get("extended", False)),
                        remote=bool(event.get("remote", False)),
                        dlc=int(event.get("dlc", 0)),
                        payload=list(event.get("payload", [])),
                        decode_spec=(str(event.get("decode_spec")) if event.get("decode_spec") is not None else None),
                    )

            count += 1

    def _notify_self_test_waiters(
        self,
        channel: int,
        frame_id: int,
        extended: bool,
        remote: bool,
        dlc: int,
        payload: list[int],
    ) -> None:
        with self.self_test_waiters_lock:
            for waiter in self.self_test_waiters:
                event = waiter["event"]
                if not isinstance(event, threading.Event) or event.is_set():
                    continue

                if channel != waiter["channel"]:
                    continue
                if frame_id != waiter["frame_id"]:
                    continue
                if extended != waiter["extended"]:
                    continue
                if remote != waiter["remote"]:
                    continue
                if dlc != waiter["dlc"]:
                    continue

                expected_payload = waiter["payload"]
                if isinstance(expected_payload, list) and payload != expected_payload:
                    continue

                event.set()

    def _split_decode_field(self, field_text: str, index: int) -> tuple[str, str]:
        text = field_text.strip()
        if not text:
            raise ValueError("decode field is empty")

        if ":" in text:
            left, right = text.split(":", 1)
            field_name = left.strip()
            field_type = right.strip()
            if not field_name:
                field_name = f"f{index}"
            if not field_type:
                raise ValueError(f"decode type missing for field '{field_name}'")
            return field_name, field_type

        return f"f{index}", text

    def _resolve_decode_type(self, type_token: str) -> tuple[str, str]:
        token = type_token.strip().lower().replace(" ", "").replace("-", "_")
        if not token:
            raise ValueError("empty decode type")

        if token in DECODE_ALIAS_TO_BASE:
            return DECODE_ALIAS_TO_BASE[token], "<"

        for suffix, endian in (("_le", "<"), ("_be", ">")):
            if token.endswith(suffix):
                base_token = token[: -len(suffix)]
                if base_token in DECODE_ALIAS_TO_BASE:
                    return DECODE_ALIAS_TO_BASE[base_token], endian

        for suffix, endian in (("le", "<"), ("be", ">")):
            if token.endswith(suffix):
                base_token = token[: -len(suffix)]
                if base_token in DECODE_ALIAS_TO_BASE:
                    return DECODE_ALIAS_TO_BASE[base_token], endian

        raise ValueError(f"unsupported decode type '{type_token}'")

    def _decode_payload(self, frame_id: int, payload: list[int], decode_spec: str | None = None) -> str:
        if not payload:
            return ""

        spec = (decode_spec if decode_spec is not None else self.decode_specs_by_id.get(frame_id, "")).strip()
        if not spec:
            return ""

        fields = [part.strip() for part in spec.split(",") if part.strip()]
        if not fields:
            return ""

        offset = 0
        parsed: list[str] = []

        for index, field_text in enumerate(fields, start=1):
            field_name, field_type = self._split_decode_field(field_text, index)
            base_type, endian = self._resolve_decode_type(field_type)

            if base_type == "bool":
                if offset + 1 > len(payload):
                    raise ValueError(f"{field_name} expects 1 byte")
                value = payload[offset] != 0
                offset += 1
                parsed.append(f"{field_name}={'true' if value else 'false'}")
                continue

            type_info = DECODE_BASE_INFO.get(base_type)
            if type_info is None:
                raise ValueError(f"unsupported decode base '{base_type}'")

            fmt_char, size = type_info
            if offset + size > len(payload):
                raise ValueError(f"{field_name} expects {size} bytes")

            raw = bytes(payload[offset : offset + size])
            value = struct.unpack(endian + fmt_char, raw)[0]
            offset += size

            if isinstance(value, float):
                parsed.append(f"{field_name}={value:.6g}")
            else:
                parsed.append(f"{field_name}={value}")

        if offset < len(payload):
            tail = " ".join(f"{byte:02X}" for byte in payload[offset:])
            parsed.append(f"raw_tail={tail}")

        return ", ".join(parsed)

    def _refresh_decode_specs_by_id(self) -> None:
        specs: dict[int, str] = {}
        for msg in self.templates_by_item.values():
            decode_spec = msg.decode_spec.strip()
            if decode_spec:
                specs[msg.frame_id] = decode_spec
        self.decode_specs_by_id = specs

        for can_id, entry in self.grouped_packets_by_id.items():
            payload = entry.get("last_payload")
            if isinstance(payload, list):
                try:
                    entry["decoded"] = self._decode_payload(can_id, payload)
                except Exception as exc:
                    entry["decoded"] = f"decode-error: {exc}"
            self._upsert_grouped_row(can_id, entry)

    def _grouped_row_values(self, can_id: int, entry: dict[str, object]) -> tuple[str, ...]:
        return (
            f"0x{can_id:X}",
            str(entry.get("format", "")),
            str(entry.get("type", "")),
            str(entry.get("tx_count", 0)),
            str(entry.get("rx_count", 0)),
            str(entry.get("last_dir", "")),
            str(entry.get("last_if", "")),
            str(entry.get("dlc", "")),
            str(entry.get("raw", "")),
            str(entry.get("decoded", "")),
            str(entry.get("last_seen", "")),
        )

    def _upsert_grouped_row(self, can_id: int, entry: dict[str, object]) -> None:
        values = self._grouped_row_values(can_id, entry)
        existing = self.group_tree_items.get(can_id)

        if existing and self.group_tree.exists(existing):
            self.group_tree.item(existing, values=values)
            return

        item = self.group_tree.insert("", tk.END, values=values)
        self.group_tree_items[can_id] = item

    def _record_grouped_packet(
        self,
        direction: str,
        channel: int,
        frame_id: int,
        extended: bool,
        remote: bool,
        dlc: int,
        payload: list[int],
        decode_spec: str | None = None,
    ) -> None:
        if decode_spec and decode_spec.strip():
            self.decode_specs_by_id[frame_id] = decode_spec.strip()

        entry = self.grouped_packets_by_id.get(frame_id)
        if entry is None:
            entry = {
                "format": "EXT" if extended else "STD",
                "type": "RTR" if remote else "DATA",
                "tx_count": 0,
                "rx_count": 0,
                "last_dir": "",
                "last_if": "",
                "dlc": 0,
                "raw": "",
                "decoded": "",
                "last_seen": "",
                "last_payload": [],
            }
            self.grouped_packets_by_id[frame_id] = entry

        if direction == "TX":
            entry["tx_count"] = int(entry.get("tx_count", 0)) + 1
        else:
            entry["rx_count"] = int(entry.get("rx_count", 0)) + 1

        raw_text = " ".join(f"{byte:02X}" for byte in payload)
        decoded = ""
        try:
            decoded = self._decode_payload(frame_id, payload, decode_spec=decode_spec)
        except Exception as exc:
            decoded = f"decode-error: {exc}"

        entry["format"] = "EXT" if extended else "STD"
        entry["type"] = "RTR" if remote else "DATA"
        entry["last_dir"] = direction
        entry["last_if"] = f"CAN{channel + 1}"
        entry["dlc"] = dlc
        entry["raw"] = raw_text
        entry["decoded"] = decoded
        entry["last_seen"] = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        entry["last_payload"] = list(payload)

        self._upsert_grouped_row(frame_id, entry)

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _parse_message_from_form(self) -> MessageTemplate:
        name = self.name_var.get().strip() or "Unnamed"
        channel = 0 if self.channel_var.get() == "CAN1" else 1
        frame_id = self._parse_can_id(self.id_var.get().strip(), bool(self.extended_var.get()))
        extended = bool(self.extended_var.get())
        remote = bool(self.remote_var.get())
        decode_spec = self.decode_var.get().strip()
        period_ms = int(self.period_ms_var.get())

        if period_ms < 0:
            raise ValueError("Period must be 0 or greater")

        dlc = int(self.dlc_var.get())
        if dlc < 0 or dlc > MAX_CAN_DATA_LEN:
            raise ValueError("DLC must be between 0 and 8")

        data = []
        if not remote:
            data = self._parse_data_bytes(self.data_var.get().strip())
            if len(data) != dlc:
                raise ValueError("For DATA frame, DLC must match number of data bytes")

        return MessageTemplate(
            name=name,
            channel=channel,
            frame_id=frame_id,
            extended=extended,
            remote=remote,
            dlc=dlc,
            data=data,
            decode_spec=decode_spec,
            period_ms=period_ms,
        )

    def _parse_can_id(self, text: str, extended: bool) -> int:
        if not text:
            raise ValueError("ID is required")

        try:
            value = int(text, 0)
        except ValueError as exc:
            raise ValueError("ID must be a valid decimal or hex value") from exc

        if value < 0:
            raise ValueError("ID must be non-negative")

        max_id = 0x1FFFFFFF if extended else 0x7FF
        if value > max_id:
            if extended:
                raise ValueError("Extended ID out of range (0x0..0x1FFFFFFF)")
            raise ValueError("Standard ID out of range (0x0..0x7FF)")

        return value

    def _parse_data_bytes(self, text: str) -> list[int]:
        if not text:
            return []

        tokens = text.replace(",", " ").split()
        if len(tokens) > MAX_CAN_DATA_LEN:
            raise ValueError("At most 8 data bytes are allowed")

        values: list[int] = []
        for token in tokens:
            try:
                value = int(token, 16)
            except ValueError as exc:
                raise ValueError(f"Invalid data byte: {token}") from exc

            if value < 0x00 or value > 0xFF:
                raise ValueError(f"Data byte out of range: {token}")

            values.append(value)

        return values

    def _template_values(self, msg: MessageTemplate) -> tuple[str, ...]:
        return (
            msg.name,
            f"CAN{msg.channel + 1}",
            f"0x{msg.frame_id:X}",
            "EXT" if msg.extended else "STD",
            "RTR" if msg.remote else "DATA",
            str(msg.dlc),
            str(msg.period_ms),
            msg.decode_spec,
            " ".join(f"{b:02X}" for b in msg.data),
        )

    def save_template(self) -> None:
        try:
            msg = self._parse_message_from_form()
        except Exception as exc:
            messagebox.showerror("Invalid Message", str(exc))
            return

        selected = self.template_tree.selection()
        if len(selected) == 1:
            item = selected[0]
            self.templates_by_item[item] = msg
            self.template_tree.item(item, values=self._template_values(msg))
            self._refresh_decode_specs_by_id()
            self._set_status(f"Updated template '{msg.name}'")
            return

        item = self.template_tree.insert("", tk.END, values=self._template_values(msg))
        self.templates_by_item[item] = msg
        self._refresh_decode_specs_by_id()
        self._set_status(f"Saved template '{msg.name}'")

    def delete_template(self) -> None:
        selected = self.template_tree.selection()
        if not selected:
            return

        for item in selected:
            self.template_tree.delete(item)
            self.templates_by_item.pop(item, None)

        self._refresh_decode_specs_by_id()
        self._set_status("Deleted selected template(s)")

    def _on_template_selected(self, _event: object) -> None:
        selected = self.template_tree.selection()
        if len(selected) != 1:
            return

        item = selected[0]
        msg = self.templates_by_item.get(item)
        if not msg:
            return

        self.name_var.set(msg.name)
        self.channel_var.set(f"CAN{msg.channel + 1}")
        self.id_var.set(hex(msg.frame_id))
        self.extended_var.set(1 if msg.extended else 0)
        self.remote_var.set(1 if msg.remote else 0)
        self.dlc_var.set(msg.dlc)
        self.decode_var.set(msg.decode_spec)
        self.period_ms_var.set(msg.period_ms)
        self.data_var.set(" ".join(f"{byte:02X}" for byte in msg.data))
        self._on_remote_toggle()

    def _to_can_object(self, msg: MessageTemplate) -> VCI_CAN_OBJ:
        frame = VCI_CAN_OBJ()
        frame.ID = msg.frame_id
        frame.TimeStamp = 0
        frame.TimeFlag = 0
        frame.SendType = 0
        frame.RemoteFlag = 1 if msg.remote else 0
        frame.ExternFlag = 1 if msg.extended else 0
        frame.DataLen = msg.dlc

        for i in range(MAX_CAN_DATA_LEN):
            frame.Data[i] = 0

        for i, value in enumerate(msg.data):
            frame.Data[i] = value

        return frame

    def _transmit_message(self, msg: MessageTemplate) -> None:
        if not self.connected or not self.dll:
            raise RuntimeError("Device is not connected")

        frame = self._to_can_object(msg)
        with self.api_lock:
            sent = int(self.dll.VCI_Transmit(VCI_USBCAN2, 0, msg.channel, ctypes.byref(frame), 1))

        if sent != 1:
            raise RuntimeError(f"VCI_Transmit failed on CAN{msg.channel + 1}")

        payload_for_group = list(msg.data[: msg.dlc]) if not msg.remote else []
        self.ui_event_queue.put(
            (
                "group_event",
                {
                    "direction": "TX",
                    "channel": msg.channel,
                    "frame_id": msg.frame_id,
                    "extended": msg.extended,
                    "remote": msg.remote,
                    "dlc": msg.dlc,
                    "payload": payload_for_group,
                    "decode_spec": msg.decode_spec,
                },
            )
        )

    def _send_message(self, msg: MessageTemplate) -> None:
        self._transmit_message(msg)

        channel_text = f"CAN{msg.channel + 1}"
        data_text = " ".join(f"{b:02X}" for b in msg.data)
        self._set_status(
            f"TX {channel_text} ID=0x{msg.frame_id:X} "
            f"{'EXT' if msg.extended else 'STD'} {'RTR' if msg.remote else 'DATA'} "
            f"DLC={msg.dlc} DATA={data_text}"
        )

    def send_current(self) -> None:
        try:
            msg = self._parse_message_from_form()
            self._send_message(msg)
        except Exception as exc:
            messagebox.showerror("Send Error", str(exc))

    def send_selected_templates(self) -> None:
        selected = self.template_tree.selection()
        if not selected:
            messagebox.showinfo("No Selection", "Select one or more templates to send")
            return

        sent = 0
        for item in selected:
            msg = self.templates_by_item.get(item)
            if not msg:
                continue
            try:
                self._send_message(msg)
                sent += 1
            except Exception as exc:
                messagebox.showerror("Send Error", str(exc))
                return

        self._set_status(f"Sent {sent} template(s)")

    def start_periodic_tx(self) -> None:
        if not self.connected:
            messagebox.showwarning("Periodic TX", "Connect the device before starting periodic transmit")
            return

        if self.periodic_running:
            messagebox.showinfo("Periodic TX", "Periodic transmit is already running")
            return

        selected = self.template_tree.selection()
        if not selected:
            messagebox.showinfo("Periodic TX", "Select one or more templates")
            return

        tasks: list[dict[str, object]] = []
        skipped_names: list[str] = []
        for item in selected:
            msg = self.templates_by_item.get(item)
            if not msg:
                continue
            if msg.period_ms <= 0:
                skipped_names.append(msg.name)
                continue

            tasks.append(
                {
                    "name": msg.name,
                    "msg": msg,
                    "period_ms": msg.period_ms,
                    "next_due": time.monotonic(),
                }
            )

        if not tasks:
            messagebox.showwarning("Periodic TX", "No selected templates have Period (ms) > 0")
            return

        self.periodic_stop_event.clear()
        self.periodic_running = True
        self._set_connected_ui(self.connected)
        self.periodic_thread = threading.Thread(
            target=self._periodic_loop,
            args=(tasks,),
            name="can-periodic",
            daemon=True,
        )
        self.periodic_thread.start()

        if skipped_names:
            self._set_status(
                f"Periodic TX started for {len(tasks)} template(s), skipped: {', '.join(skipped_names)}"
            )
        else:
            self._set_status(f"Periodic TX started for {len(tasks)} template(s)")

    def stop_periodic_tx(self, wait: bool = False) -> None:
        if not self.periodic_running:
            return

        self.periodic_stop_event.set()
        if wait and self.periodic_thread and self.periodic_thread.is_alive():
            self.periodic_thread.join(timeout=1.5)

        if not self.periodic_thread or not self.periodic_thread.is_alive():
            self.periodic_running = False
            self._set_connected_ui(self.connected)

    def _periodic_loop(self, tasks: list[dict[str, object]]) -> None:
        sent_count = 0

        while not self.periodic_stop_event.is_set():
            now = time.monotonic()
            next_due = None

            for task in tasks:
                task_next_due = float(task["next_due"])
                if now >= task_next_due:
                    try:
                        msg = task["msg"]
                        if not isinstance(msg, MessageTemplate):
                            continue
                        self._transmit_message(msg)
                        sent_count += 1
                    except Exception as exc:
                        self.ui_event_queue.put(("error", f"Periodic TX error: {exc}"))
                        self.periodic_stop_event.set()
                        break

                    period_ms = int(task["period_ms"])
                    task["next_due"] = now + (period_ms / 1000.0)

                if next_due is None or float(task["next_due"]) < next_due:
                    next_due = float(task["next_due"])

            if self.periodic_stop_event.is_set():
                break

            if next_due is None:
                break

            sleep_s = max(0.001, min(0.05, next_due - time.monotonic()))
            self.periodic_stop_event.wait(timeout=sleep_s)

        self.ui_event_queue.put(("periodic_done", f"Periodic TX stopped, sent {sent_count} frame(s)"))

    def run_self_test(self) -> None:
        if not self.connected:
            messagebox.showwarning("Self Test", "Connect the device before running self-test")
            return

        if self.self_test_running:
            messagebox.showinfo("Self Test", "Self-test is already running")
            return

        self.self_test_running = True
        self._set_connected_ui(self.connected)
        self._set_status("Self-test running...")

        threading.Thread(target=self._self_test_worker, name="can-self-test", daemon=True).start()

    def _self_test_worker(self) -> None:
        frame_can1 = MessageTemplate(
            name="SelfTest CAN1->CAN2",
            channel=0,
            frame_id=0x5A1,
            extended=False,
            remote=False,
            dlc=4,
            data=[0x11, 0x22, 0x33, 0x44],
            period_ms=0,
        )
        frame_can2 = MessageTemplate(
            name="SelfTest CAN2->CAN1",
            channel=1,
            frame_id=0x5A2,
            extended=False,
            remote=False,
            dlc=4,
            data=[0xAA, 0xBB, 0xCC, 0xDD],
            period_ms=0,
        )

        wait_can2 = threading.Event()
        wait_can1 = threading.Event()
        waiters = [
            {
                "event": wait_can2,
                "channel": 1,
                "frame_id": frame_can1.frame_id,
                "extended": frame_can1.extended,
                "remote": frame_can1.remote,
                "dlc": frame_can1.dlc,
                "payload": frame_can1.data,
            },
            {
                "event": wait_can1,
                "channel": 0,
                "frame_id": frame_can2.frame_id,
                "extended": frame_can2.extended,
                "remote": frame_can2.remote,
                "dlc": frame_can2.dlc,
                "payload": frame_can2.data,
            },
        ]

        with self.self_test_waiters_lock:
            self.self_test_waiters.extend(waiters)

        try:
            self._transmit_message(frame_can1)
            time.sleep(0.03)
            self._transmit_message(frame_can2)

            can2_ok = wait_can2.wait(timeout=2.0)
            can1_ok = wait_can1.wait(timeout=2.0)

            if can1_ok and can2_ok:
                self.ui_event_queue.put(("self_test_result", "PASS: CAN1<->CAN2 loopback verified"))
            else:
                missing = []
                if not can2_ok:
                    missing.append("CAN2 did not receive CAN1 self-test frame")
                if not can1_ok:
                    missing.append("CAN1 did not receive CAN2 self-test frame")
                self.ui_event_queue.put(("self_test_result", f"FAIL: {'; '.join(missing)}"))
        except Exception as exc:
            self.ui_event_queue.put(("self_test_result", f"FAIL: self-test transmit error: {exc}"))
        finally:
            with self.self_test_waiters_lock:
                for waiter in waiters:
                    if waiter in self.self_test_waiters:
                        self.self_test_waiters.remove(waiter)

    def export_rx_csv(self, channel: int) -> None:
        if channel == 0:
            records = list(self.rx_records_can1)
            channel_name = "CAN1"
        else:
            records = list(self.rx_records_can2)
            channel_name = "CAN2"

        if not records:
            messagebox.showinfo("Export CSV", f"No {channel_name} packets to export")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"{channel_name.lower()}_rx_{timestamp}.csv"
        target_path = filedialog.asksaveasfilename(
            title=f"Export {channel_name} RX to CSV",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )

        if not target_path:
            return

        with open(target_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "host_time",
                    "channel",
                    "id_hex",
                    "id_dec",
                    "format",
                    "type",
                    "dlc",
                    "data",
                    "decoded",
                    "timestamp_hex",
                    "timestamp_dec",
                ]
            )

            for record in records:
                writer.writerow(
                    [
                        record["host_time"],
                        record["channel"],
                        record["id_hex"],
                        record["id_dec"],
                        record["format"],
                        record["type"],
                        record["dlc"],
                        record["data"],
                        record.get("decoded", ""),
                        record["timestamp_hex"],
                        record["timestamp_dec"],
                    ]
                )

        self._set_status(f"Exported {len(records)} {channel_name} packet(s) to CSV")

    def clear_grouped_view(self) -> None:
        self.grouped_packets_by_id.clear()
        self.group_tree_items.clear()
        for item in self.group_tree.get_children():
            self.group_tree.delete(item)
        self._set_status("Cleared grouped-by-ID view")

    def clear_can1_log(self) -> None:
        self._clear_text(self.rx_text_can1)
        self.rx_records_can1.clear()
        self._set_status("Cleared CAN1 RX log")

    def clear_can2_log(self) -> None:
        self._clear_text(self.rx_text_can2)
        self.rx_records_can2.clear()
        self._set_status("Cleared CAN2 RX log")

    def _clear_text(self, widget: tk.Text) -> None:
        widget.delete("1.0", tk.END)

    def _on_close(self) -> None:
        try:
            self.disconnect_device()
        finally:
            self.destroy()


def main() -> None:
    app = CanGuiApp()
    app.mainloop()


if __name__ == "__main__":
    main()
