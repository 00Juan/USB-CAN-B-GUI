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
import json
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

RULE_TYPE_OPTIONS = ["bool", "u8", "i8", "u16", "i16", "u32", "i32", "u64", "i64", "f32", "f64"]
RULE_ENDIAN_OPTIONS = ["LE", "BE"]


@dataclass
class MessageTemplate:
    name: str
    channel: int  # 0 for CAN1, 1 for CAN2
    frame_id: int
    extended: bool
    remote: bool
    dlc: int
    data: list[int]
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
        self.custom_decode_fields_by_id: dict[int, list[dict[str, str]]] = {}
        self.custom_decode_specs_by_id: dict[int, str] = {}
        self.decode_specs_by_id: dict[int, str] = {}
        self.grouped_packets_by_id: dict[tuple[int, int], dict[str, object]] = {}
        self.group_tree_items: dict[tuple[int, int], str] = {}
        self.decoded_group_tree_items: dict[tuple[int, int], str] = {}
        self.database_tree_items: dict[tuple[int, int, str], str] = {}
        self.database_row_keys_by_group: dict[tuple[int, int], set[tuple[int, int, str]]] = {}
        self.decode_rule_tree_items: dict[int, str] = {}
        self.rx_records_can1: deque[dict[str, object]] = deque(maxlen=MAX_RX_HISTORY)
        self.rx_records_can2: deque[dict[str, object]] = deque(maxlen=MAX_RX_HISTORY)

        self.lib_path_var = tk.StringVar(value=self._default_lib_path())
        self.can1_baud_var = tk.StringVar(value="125K")
        self.can2_baud_var = tk.StringVar(value="125K")
        self.status_var = tk.StringVar(value="Disconnected")
        self.group_if_filter_var = tk.StringVar(value="All")
        self.group_dir_filter_var = tk.StringVar(value="All")

        self.name_var = tk.StringVar(value="Frame1")
        self.channel_var = tk.StringVar(value="CAN1")
        self.id_var = tk.StringVar(value="0x100")
        self.extended_var = tk.IntVar(value=0)
        self.remote_var = tk.IntVar(value=0)
        self.dlc_var = tk.IntVar(value=8)
        self.data_var = tk.StringVar(value="01 02 03 04 05 06 07 08")
        self.period_ms_var = tk.IntVar(value=0)
        self.rule_id_var = tk.StringVar(value="")
        self.rule_field_name_var = tk.StringVar(value="field1")
        self.rule_field_start_var = tk.StringVar(value="0")
        self.rule_field_type_var = tk.StringVar(value="u8")
        self.rule_field_endian_var = tk.StringVar(value="LE")

        self.dll = None
        self.rule_window: tk.Toplevel | None = None
        self.rule_fields_tree = None
        self.decode_rule_tree = None
        self.session_state_path = Path(__file__).resolve().parent / "can_gui_session.json"

        self._build_ui()
        self._load_session_state()
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

    def _template_to_state_dict(self, msg: MessageTemplate) -> dict[str, object]:
        return {
            "name": msg.name,
            "channel": int(msg.channel),
            "frame_id": int(msg.frame_id),
            "extended": bool(msg.extended),
            "remote": bool(msg.remote),
            "dlc": int(msg.dlc),
            "data": [int(byte) & 0xFF for byte in msg.data],
            "period_ms": int(msg.period_ms),
        }

    def _template_from_state_dict(self, raw: object) -> MessageTemplate | None:
        if not isinstance(raw, dict):
            return None

        try:
            name = str(raw.get("name", "")).strip() or "Unnamed"
            channel = int(raw.get("channel", 0))
            frame_id = int(raw.get("frame_id", 0))
            extended = bool(raw.get("extended", False))
            remote = bool(raw.get("remote", False))
            dlc = int(raw.get("dlc", 0))
            period_ms = int(raw.get("period_ms", 0))
        except Exception:
            return None

        if channel not in (0, 1):
            return None
        if frame_id < 0 or frame_id > 0x1FFFFFFF:
            return None
        if dlc < 0 or dlc > MAX_CAN_DATA_LEN:
            return None
        if period_ms < 0:
            period_ms = 0

        raw_data = raw.get("data", [])
        data: list[int] = []
        if isinstance(raw_data, list):
            for token in raw_data[:MAX_CAN_DATA_LEN]:
                try:
                    value = int(token)
                except Exception:
                    continue
                if 0 <= value <= 0xFF:
                    data.append(value)

        if remote:
            data = []
        if not remote:
            data = data[:dlc]
            if len(data) < dlc:
                data.extend([0] * (dlc - len(data)))

        return MessageTemplate(
            name=name,
            channel=channel,
            frame_id=frame_id,
            extended=extended,
            remote=remote,
            dlc=dlc,
            data=data,
            period_ms=period_ms,
        )

    def _normalize_grouped_entry(self, channel: int, entry: object) -> dict[str, object] | None:
        if not isinstance(entry, dict):
            return None

        try:
            tx_count = max(0, int(entry.get("tx_count", 0)))
            rx_count = max(0, int(entry.get("rx_count", 0)))
            dlc = int(entry.get("dlc", 0))
        except Exception:
            return None

        if dlc < 0:
            dlc = 0
        if dlc > MAX_CAN_DATA_LEN:
            dlc = MAX_CAN_DATA_LEN

        payload: list[int] = []
        raw_payload = entry.get("last_payload", [])
        if isinstance(raw_payload, list):
            for token in raw_payload[:MAX_CAN_DATA_LEN]:
                try:
                    value = int(token)
                except Exception:
                    continue
                if 0 <= value <= 0xFF:
                    payload.append(value)

        normalized = {
            "format": str(entry.get("format", "STD")),
            "type": str(entry.get("type", "DATA")),
            "tx_count": tx_count,
            "rx_count": rx_count,
            "last_dir": str(entry.get("last_dir", "")),
            "last_if": str(entry.get("last_if", f"CAN{channel + 1}")),
            "dlc": dlc,
            "raw": str(entry.get("raw", "")),
            "decoded": str(entry.get("decoded", "")),
            "last_seen": str(entry.get("last_seen", "")),
            "last_payload": payload,
        }
        return normalized

    def _save_session_state(self) -> None:
        try:
            templates_state: list[dict[str, object]] = []
            for item in self.template_tree.get_children():
                msg = self.templates_by_item.get(item)
                if not msg:
                    continue
                templates_state.append(self._template_to_state_dict(msg))

            grouped_state: list[dict[str, object]] = []
            sorted_groups = sorted(self.grouped_packets_by_id.items(), key=lambda entry: (entry[0][0], entry[0][1]))
            for (channel, frame_id), entry in sorted_groups:
                normalized_entry = self._normalize_grouped_entry(channel, entry)
                if normalized_entry is None:
                    continue
                grouped_state.append(
                    {
                        "channel": channel,
                        "frame_id": frame_id,
                        "entry": normalized_entry,
                    }
                )

            fields_state: dict[str, list[dict[str, str]]] = {}
            for can_id, field_defs in self.custom_decode_fields_by_id.items():
                normalized_fields: list[dict[str, str]] = []
                for field in field_defs:
                    if not isinstance(field, dict):
                        continue
                    field_name = str(field.get("name", "")).strip()
                    field_start = str(field.get("start", "0")).strip() or "0"
                    field_type = str(field.get("type", "")).strip().lower()
                    field_endian = str(field.get("endian", "LE")).strip().upper() or "LE"
                    if not field_name or field_type not in RULE_TYPE_OPTIONS:
                        continue
                    if field_endian not in RULE_ENDIAN_OPTIONS:
                        field_endian = "LE"
                    normalized_fields.append(
                        {
                            "name": field_name,
                            "start": field_start,
                            "type": field_type,
                            "endian": field_endian,
                        }
                    )
                if normalized_fields:
                    fields_state[str(can_id)] = normalized_fields

            state = {
                "version": 1,
                "can1_baud": self.can1_baud_var.get(),
                "can2_baud": self.can2_baud_var.get(),
                "group_if_filter": self.group_if_filter_var.get(),
                "group_dir_filter": self.group_dir_filter_var.get(),
                "templates": templates_state,
                "custom_decode_specs_by_id": {str(can_id): spec for can_id, spec in self.custom_decode_specs_by_id.items()},
                "custom_decode_fields_by_id": fields_state,
                "grouped_packets": grouped_state,
            }

            temp_path = self.session_state_path.with_suffix(self.session_state_path.suffix + ".tmp")
            with temp_path.open("w", encoding="utf-8") as state_file:
                json.dump(state, state_file, indent=2)
            temp_path.replace(self.session_state_path)
        except Exception as exc:
            self._set_status(f"State save warning: {exc}")

    def _load_session_state(self) -> None:
        if not self.session_state_path.exists():
            return

        try:
            with self.session_state_path.open("r", encoding="utf-8") as state_file:
                state = json.load(state_file)
        except Exception as exc:
            self._set_status(f"State load warning: {exc}")
            return

        if not isinstance(state, dict):
            return

        can1_baud = str(state.get("can1_baud", "")).strip().upper()
        can2_baud = str(state.get("can2_baud", "")).strip().upper()
        if can1_baud in BAUD_TO_TIMING:
            self.can1_baud_var.set(can1_baud)
        if can2_baud in BAUD_TO_TIMING:
            self.can2_baud_var.set(can2_baud)

        if_filter = str(state.get("group_if_filter", "All")).strip().upper()
        dir_filter = str(state.get("group_dir_filter", "All")).strip().upper()
        self.group_if_filter_var.set({"ALL": "All", "CAN1": "CAN1", "CAN2": "CAN2"}.get(if_filter, "All"))
        self.group_dir_filter_var.set({"ALL": "All", "RX": "RX", "TX": "TX"}.get(dir_filter, "All"))

        for item in self.template_tree.get_children():
            self.template_tree.delete(item)
        self.templates_by_item.clear()

        templates = state.get("templates", [])
        if isinstance(templates, list):
            for raw_template in templates:
                msg = self._template_from_state_dict(raw_template)
                if msg is None:
                    continue
                item = self.template_tree.insert("", tk.END, values=self._template_values(msg))
                self.templates_by_item[item] = msg

        self.custom_decode_specs_by_id.clear()
        raw_specs = state.get("custom_decode_specs_by_id", {})
        if isinstance(raw_specs, dict):
            for raw_id, raw_spec in raw_specs.items():
                try:
                    can_id = int(str(raw_id), 0)
                except Exception:
                    continue
                if can_id < 0 or can_id > 0x1FFFFFFF:
                    continue
                if isinstance(raw_spec, str):
                    self.custom_decode_specs_by_id[can_id] = raw_spec

        self.custom_decode_fields_by_id.clear()
        raw_fields_map = state.get("custom_decode_fields_by_id", {})
        if isinstance(raw_fields_map, dict):
            for raw_id, raw_fields in raw_fields_map.items():
                try:
                    can_id = int(str(raw_id), 0)
                except Exception:
                    continue
                if can_id < 0 or can_id > 0x1FFFFFFF:
                    continue
                if not isinstance(raw_fields, list):
                    continue

                parsed_fields: list[dict[str, str]] = []
                for raw_field in raw_fields:
                    if not isinstance(raw_field, dict):
                        continue
                    field_name = str(raw_field.get("name", "")).strip()
                    field_start = str(raw_field.get("start", "0")).strip() or "0"
                    field_type = str(raw_field.get("type", "")).strip().lower()
                    field_endian = str(raw_field.get("endian", "LE")).strip().upper() or "LE"
                    if not field_name or field_type not in RULE_TYPE_OPTIONS:
                        continue
                    if field_endian not in RULE_ENDIAN_OPTIONS:
                        field_endian = "LE"
                    parsed_fields.append(
                        {
                            "name": field_name,
                            "start": field_start,
                            "type": field_type,
                            "endian": field_endian,
                        }
                    )

                if parsed_fields:
                    self.custom_decode_fields_by_id[can_id] = parsed_fields
                    if can_id not in self.custom_decode_specs_by_id:
                        self.custom_decode_specs_by_id[can_id] = self._field_defs_to_spec(parsed_fields)

        self.grouped_packets_by_id.clear()
        self.group_tree_items.clear()
        self.decoded_group_tree_items.clear()
        self.database_tree_items.clear()
        self.database_row_keys_by_group.clear()
        for item in self.group_tree.get_children():
            self.group_tree.delete(item)
        for item in self.decoded_group_tree.get_children():
            self.decoded_group_tree.delete(item)
        for item in self.database_tree.get_children():
            self.database_tree.delete(item)

        grouped_packets = state.get("grouped_packets", [])
        if isinstance(grouped_packets, list):
            for raw_packet in grouped_packets:
                if not isinstance(raw_packet, dict):
                    continue
                try:
                    channel = int(raw_packet.get("channel", 0))
                    frame_id = int(raw_packet.get("frame_id", 0))
                except Exception:
                    continue
                if channel not in (0, 1):
                    continue
                if frame_id < 0 or frame_id > 0x1FFFFFFF:
                    continue

                normalized_entry = self._normalize_grouped_entry(channel, raw_packet.get("entry", {}))
                if normalized_entry is None:
                    continue
                self.grouped_packets_by_id[(channel, frame_id)] = normalized_entry

        self._refresh_decode_specs_by_id()
        self._refresh_grouped_rows_by_filters()

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

        ttk.Label(frame, text="CAN1 baud:").grid(row=0, column=2, sticky=tk.W)
        ttk.Combobox(
            frame,
            textvariable=self.can1_baud_var,
            values=list(BAUD_TO_TIMING.keys()),
            state="readonly",
            width=8,
        ).grid(row=0, column=3, sticky=tk.W, padx=(6, 12))

        ttk.Label(frame, text="CAN2 baud:").grid(row=0, column=4, sticky=tk.W)
        ttk.Combobox(
            frame,
            textvariable=self.can2_baud_var,
            values=list(BAUD_TO_TIMING.keys()),
            state="readonly",
            width=8,
        ).grid(row=0, column=5, sticky=tk.W, padx=(6, 12))

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.connect_device)
        self.connect_btn.grid(row=0, column=6, sticky=tk.EW)

        self.disconnect_btn = ttk.Button(frame, text="Disconnect", command=self.disconnect_device)
        self.disconnect_btn.grid(row=0, column=7, sticky=tk.EW, padx=(6, 0))

        self.self_test_btn = ttk.Button(frame, text="Self Test", command=self.run_self_test)
        self.self_test_btn.grid(row=0, column=8, sticky=tk.EW, padx=(6, 0))

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

        columns = ("name", "if", "id", "format", "type", "dlc", "period", "data")
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
            "data": 320,
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
        ttk.Button(row, text="Configure Decode Rules", command=self.open_decode_rule_window).pack(side=tk.LEFT, padx=(6, 0))

        filter_row = ttk.Frame(outer)
        filter_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(filter_row, text="Grouped Filters:").pack(side=tk.LEFT)
        ttk.Label(filter_row, text="Interface").pack(side=tk.LEFT, padx=(8, 4))
        interface_filter = ttk.Combobox(
            filter_row,
            textvariable=self.group_if_filter_var,
            values=["All", "CAN1", "CAN2"],
            state="readonly",
            width=6,
        )
        interface_filter.pack(side=tk.LEFT)
        interface_filter.bind("<<ComboboxSelected>>", self._on_group_filter_changed)

        ttk.Label(filter_row, text="Direction").pack(side=tk.LEFT, padx=(10, 4))
        direction_filter = ttk.Combobox(
            filter_row,
            textvariable=self.group_dir_filter_var,
            values=["All", "RX", "TX"],
            state="readonly",
            width=5,
        )
        direction_filter.pack(side=tk.LEFT)
        direction_filter.bind("<<ComboboxSelected>>", self._on_group_filter_changed)

        ttk.Button(filter_row, text="Reset Filters", command=self.reset_group_filters).pack(side=tk.LEFT, padx=(10, 0))

        notebook = ttk.Notebook(outer)
        notebook.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        raw_group_tab = ttk.Frame(notebook)
        decoded_group_tab = ttk.Frame(notebook)
        database_tab = ttk.Frame(notebook)
        notebook.add(raw_group_tab, text="Grouped by CAN ID")
        notebook.add(decoded_group_tab, text="Grouped by CAN ID (Decoded)")
        notebook.add(database_tab, text="CAN Database")

        raw_columns = (
            "id",
            "last_if",
            "format",
            "type",
            "tx_count",
            "rx_count",
            "last_dir",
            "dlc",
            "raw",
            "last_seen",
        )
        raw_headings = {
            "id": "CAN ID",
            "last_if": "Interface",
            "format": "Format",
            "type": "Type",
            "tx_count": "TX",
            "rx_count": "RX",
            "last_dir": "Last Dir",
            "dlc": "DLC",
            "raw": "Last Raw Data",
            "last_seen": "Last Seen",
        }
        raw_widths = {
            "id": 90,
            "last_if": 85,
            "format": 70,
            "type": 70,
            "tx_count": 55,
            "rx_count": 55,
            "last_dir": 70,
            "dlc": 50,
            "raw": 300,
            "last_seen": 115,
        }

        self.group_tree = ttk.Treeview(raw_group_tab, columns=raw_columns, show="headings", height=14)
        self.group_tree.pack(fill=tk.BOTH, expand=True)
        for key in raw_columns:
            self.group_tree.heading(key, text=raw_headings[key])
            self.group_tree.column(key, width=raw_widths[key], anchor=tk.W)
        self.group_tree.bind("<<TreeviewSelect>>", self._on_group_row_selected)

        ttk.Label(
            decoded_group_tab,
            text="Decoded values are generated using rules from 'Configure Decode Rules'.",
            anchor=tk.W,
        ).pack(fill=tk.X, pady=(0, 6))

        decoded_columns = (
            "id",
            "last_if",
            "format",
            "type",
            "tx_count",
            "rx_count",
            "last_dir",
            "dlc",
            "decoded",
            "last_seen",
        )
        decoded_headings = {
            "id": "CAN ID",
            "last_if": "Interface",
            "format": "Format",
            "type": "Type",
            "tx_count": "TX",
            "rx_count": "RX",
            "last_dir": "Last Dir",
            "dlc": "DLC",
            "decoded": "Decoded Data",
            "last_seen": "Last Seen",
        }
        decoded_widths = {
            "id": 90,
            "last_if": 85,
            "format": 70,
            "type": 70,
            "tx_count": 55,
            "rx_count": 55,
            "last_dir": 70,
            "dlc": 50,
            "decoded": 360,
            "last_seen": 115,
        }

        self.decoded_group_tree = ttk.Treeview(decoded_group_tab, columns=decoded_columns, show="headings", height=14)
        self.decoded_group_tree.pack(fill=tk.BOTH, expand=True)
        for key in decoded_columns:
            self.decoded_group_tree.heading(key, text=decoded_headings[key])
            self.decoded_group_tree.column(key, width=decoded_widths[key], anchor=tk.W)
        self.decoded_group_tree.bind("<<TreeviewSelect>>", self._on_group_row_selected)

        ttk.Label(
            database_tab,
            text="High-level message/signal view derived from grouped packets and decode rules.",
            anchor=tk.W,
        ).pack(fill=tk.X, pady=(0, 6))

        db_columns = (
            "signal",
            "value",
            "rx_count",
            "tx_count",
            "last_seen",
        )
        db_headings = {
            "signal": "Signal",
            "value": "Latest Value",
            "rx_count": "RX",
            "tx_count": "TX",
            "last_seen": "Last Update",
        }
        db_widths = {
            "signal": 220,
            "value": 470,
            "rx_count": 55,
            "tx_count": 55,
            "last_seen": 115,
        }

        self.database_tree = ttk.Treeview(database_tab, columns=db_columns, show="headings", height=14)
        self.database_tree.pack(fill=tk.BOTH, expand=True)
        for key in db_columns:
            self.database_tree.heading(key, text=db_headings[key])
            self.database_tree.column(key, width=db_widths[key], anchor=tk.W)

    def open_decode_rule_window(self) -> None:
        if self.rule_window is not None and self.rule_window.winfo_exists():
            self.rule_window.deiconify()
            self.rule_window.lift()
            self.rule_window.focus_force()
            return

        self.rule_window = tk.Toplevel(self)
        self.rule_window.title("Decode Rule Configuration")
        self.rule_window.geometry("980x560")
        self.rule_window.minsize(860, 460)
        self.rule_window.transient(self)
        self.rule_window.protocol("WM_DELETE_WINDOW", self._on_rule_window_close)

        container = ttk.Frame(self.rule_window, padding=10)
        container.pack(fill=tk.BOTH, expand=True)

        top_row = ttk.Frame(container)
        top_row.pack(fill=tk.X)
        ttk.Label(top_row, text="CAN ID:").pack(side=tk.LEFT)
        ttk.Entry(top_row, textvariable=self.rule_id_var, width=14).pack(side=tk.LEFT, padx=(6, 12))
        ttk.Button(top_row, text="Use Selected ID", command=self.use_selected_group_id).pack(side=tk.LEFT)
        ttk.Button(top_row, text="Set/Update Rule", command=self.set_custom_decode_rule).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(top_row, text="Delete Rule", command=self.delete_custom_decode_rule).pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(
            container,
            text="Define each field by byte-group (start byte + datatype + endianness).",
            anchor=tk.W,
        ).pack(fill=tk.X, pady=(6, 0))

        field_controls = ttk.Frame(container)
        field_controls.pack(fill=tk.X, pady=(8, 6))
        ttk.Label(field_controls, text="Field name:").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(field_controls, textvariable=self.rule_field_name_var, width=16).grid(row=0, column=1, sticky=tk.W, padx=(6, 12))

        ttk.Label(field_controls, text="Start byte:").grid(row=0, column=2, sticky=tk.W)
        ttk.Spinbox(field_controls, from_=0, to=63, textvariable=self.rule_field_start_var, width=6).grid(
            row=0,
            column=3,
            sticky=tk.W,
            padx=(6, 12),
        )

        ttk.Label(field_controls, text="Type:").grid(row=0, column=4, sticky=tk.W)
        ttk.Combobox(
            field_controls,
            textvariable=self.rule_field_type_var,
            values=RULE_TYPE_OPTIONS,
            state="readonly",
            width=8,
        ).grid(row=0, column=5, sticky=tk.W, padx=(6, 12))

        ttk.Label(field_controls, text="Endian:").grid(row=0, column=6, sticky=tk.W)
        ttk.Combobox(
            field_controls,
            textvariable=self.rule_field_endian_var,
            values=RULE_ENDIAN_OPTIONS,
            state="readonly",
            width=6,
        ).grid(row=0, column=7, sticky=tk.W, padx=(6, 12))

        ttk.Button(field_controls, text="Add Field", command=self.add_rule_field).grid(row=0, column=8, sticky=tk.W, padx=(0, 6))
        ttk.Button(field_controls, text="Update Field", command=self.update_rule_field).grid(row=0, column=9, sticky=tk.W, padx=(0, 6))
        ttk.Button(field_controls, text="Remove Field", command=self.remove_rule_field).grid(row=0, column=10, sticky=tk.W, padx=(0, 6))
        ttk.Button(field_controls, text="Clear Fields", command=self.clear_rule_fields).grid(row=0, column=11, sticky=tk.W)

        fields_frame = ttk.LabelFrame(container, text="Rule Fields")
        fields_frame.pack(fill=tk.BOTH, expand=False)
        self.rule_fields_tree = ttk.Treeview(
            fields_frame,
            columns=("name", "start", "type", "endian"),
            show="headings",
            height=6,
        )
        self.rule_fields_tree.pack(fill=tk.BOTH, expand=True)
        self.rule_fields_tree.heading("name", text="Field")
        self.rule_fields_tree.heading("start", text="Start Byte")
        self.rule_fields_tree.heading("type", text="Type")
        self.rule_fields_tree.heading("endian", text="Endian")
        self.rule_fields_tree.column("name", width=240, anchor=tk.W)
        self.rule_fields_tree.column("start", width=110, anchor=tk.W)
        self.rule_fields_tree.column("type", width=120, anchor=tk.W)
        self.rule_fields_tree.column("endian", width=110, anchor=tk.W)
        self.rule_fields_tree.bind("<<TreeviewSelect>>", self._on_rule_field_selected)

        rules_frame = ttk.LabelFrame(container, text="Saved Rules")
        rules_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.decode_rule_tree = ttk.Treeview(rules_frame, columns=("id", "summary"), show="headings", height=6)
        self.decode_rule_tree.pack(fill=tk.BOTH, expand=True)
        self.decode_rule_tree.heading("id", text="CAN ID")
        self.decode_rule_tree.heading("summary", text="Rule Summary")
        self.decode_rule_tree.column("id", width=120, anchor=tk.W)
        self.decode_rule_tree.column("summary", width=700, anchor=tk.W)
        self.decode_rule_tree.bind("<<TreeviewSelect>>", self._on_decode_rule_selected)

        self._populate_decode_rule_tree()

    def _on_rule_window_close(self) -> None:
        if self.rule_window is not None and self.rule_window.winfo_exists():
            self.rule_window.destroy()
        self.rule_window = None
        self.rule_fields_tree = None
        self.decode_rule_tree = None
        self.decode_rule_tree_items = {}

    def _populate_decode_rule_tree(self) -> None:
        if self.decode_rule_tree is None:
            return

        for item in self.decode_rule_tree.get_children():
            self.decode_rule_tree.delete(item)
        self.decode_rule_tree_items = {}

        sorted_ids = sorted(set(self.custom_decode_fields_by_id.keys()) | set(self.custom_decode_specs_by_id.keys()))
        for can_id in sorted_ids:
            field_defs = self.custom_decode_fields_by_id.get(can_id)
            if field_defs is None:
                decode_spec = self.custom_decode_specs_by_id.get(can_id, "")
                field_defs = self._spec_to_field_defs(decode_spec) if decode_spec else []
            self._upsert_decode_rule_row(can_id, field_defs)

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
            channel_baud = {
                0: self.can1_baud_var.get(),
                1: self.can2_baud_var.get(),
            }

            with self.api_lock:
                if self.dll.VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK:
                    raise RuntimeError("VCI_OpenDevice failed")

                for channel in (0, 1):
                    timing0, timing1 = BAUD_TO_TIMING[channel_baud[channel]]
                    cfg = VCI_INIT_CONFIG(
                        AccCode=0x80000008,
                        AccMask=0xFFFFFFFF,
                        Reserved=0,
                        Filter=0,
                        Timing0=timing0,
                        Timing1=timing1,
                        Mode=0,
                    )
                    if self.dll.VCI_InitCAN(VCI_USBCAN2, 0, channel, ctypes.byref(cfg)) != STATUS_OK:
                        raise RuntimeError(f"VCI_InitCAN failed for CAN{channel + 1}")
                    if self.dll.VCI_StartCAN(VCI_USBCAN2, 0, channel) != STATUS_OK:
                        raise RuntimeError(f"VCI_StartCAN failed for CAN{channel + 1}")

            self.stop_event.clear()
            self.rx_thread = threading.Thread(target=self._rx_loop, name="can-rx", daemon=True)
            self.rx_thread.start()

            self.connected = True
            self._set_connected_ui(True)
            self._set_status(
                f"Connected (CAN1={self.can1_baud_var.get()}, CAN2={self.can2_baud_var.get()})"
            )
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

        custom_fields = self.custom_decode_fields_by_id.get(frame_id)
        if custom_fields:
            return self._decode_with_field_defs(payload, custom_fields)

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

    def _parse_can_id_any(self, text: str) -> int:
        if not text:
            raise ValueError("CAN ID is required")

        try:
            value = int(text, 0)
        except ValueError as exc:
            raise ValueError("CAN ID must be decimal or hex") from exc

        if value < 0:
            raise ValueError("CAN ID must be non-negative")

        if value > 0x1FFFFFFF:
            raise ValueError("CAN ID out of range (0x0..0x1FFFFFFF)")

        return value

    def _field_def_to_token(self, field: dict[str, str]) -> str:
        base_type = field["type"].strip().lower()
        endian = field.get("endian", "LE").strip().upper()

        if base_type in ("bool", "u8", "i8"):
            return base_type

        suffix = "_be" if endian == "BE" else "_le"
        return f"{base_type}{suffix}"

    def _field_size(self, base_type: str) -> int:
        if base_type == "bool":
            return 1

        type_info = DECODE_BASE_INFO.get(base_type)
        if type_info is None:
            raise ValueError(f"unsupported decode base '{base_type}'")

        return int(type_info[1])

    def _decode_with_field_defs(self, payload: list[int], field_defs: list[dict[str, str]]) -> str:
        if not field_defs:
            return ""

        parsed: list[str] = []
        used_ranges: list[tuple[int, int]] = []

        for field in field_defs:
            field_name = str(field.get("name", "")).strip()
            if not field_name:
                raise ValueError("field name is required")

            base_type = str(field.get("type", "")).strip().lower()
            if not base_type:
                raise ValueError(f"decode type missing for field '{field_name}'")

            start_text = str(field.get("start", "0")).strip() or "0"
            try:
                start = int(start_text, 0)
            except ValueError as exc:
                raise ValueError(f"invalid start byte '{start_text}' for field '{field_name}'") from exc

            if start < 0:
                raise ValueError(f"start byte must be >= 0 for field '{field_name}'")

            endian_text = str(field.get("endian", "LE")).strip().upper()
            endian = ">" if endian_text == "BE" else "<"

            size = self._field_size(base_type)
            end = start + size
            if end > len(payload):
                raise ValueError(f"{field_name} expects bytes [{start}..{end - 1}]")

            used_ranges.append((start, end))

            if base_type == "bool":
                value = payload[start] != 0
                parsed.append(f"{field_name}={'true' if value else 'false'}")
                continue

            type_info = DECODE_BASE_INFO.get(base_type)
            if type_info is None:
                raise ValueError(f"unsupported decode base '{base_type}'")

            fmt_char, _ = type_info
            raw = bytes(payload[start:end])
            value = struct.unpack(endian + fmt_char, raw)[0]
            if isinstance(value, float):
                parsed.append(f"{field_name}={value:.6g}")
            else:
                parsed.append(f"{field_name}={value}")

        consumed = [False] * len(payload)
        for start, end in used_ranges:
            for idx in range(start, end):
                if 0 <= idx < len(consumed):
                    consumed[idx] = True

        tail = [payload[idx] for idx, is_used in enumerate(consumed) if not is_used]
        if tail:
            parsed.append(f"raw_tail={' '.join(f'{byte:02X}' for byte in tail)}")

        return ", ".join(parsed)

    def _field_defs_to_spec(self, field_defs: list[dict[str, str]]) -> str:
        parts: list[str] = []
        for field in field_defs:
            token = self._field_def_to_token(field)
            start_text = str(field.get("start", "0")).strip() or "0"
            parts.append(f"{field['name']}@{start_text}:{token}")
        return ", ".join(parts)

    def _spec_to_field_defs(self, decode_spec: str) -> list[dict[str, str]]:
        fields = [part.strip() for part in decode_spec.split(",") if part.strip()]
        parsed: list[dict[str, str]] = []
        running_offset = 0

        for index, field_text in enumerate(fields, start=1):
            field_name, field_type = self._split_decode_field(field_text, index)
            field_start = running_offset
            if "@" in field_name:
                raw_name, raw_start = field_name.rsplit("@", 1)
                raw_name = raw_name.strip()
                if raw_name:
                    field_name = raw_name
                try:
                    field_start = int(raw_start.strip(), 0)
                except ValueError as exc:
                    raise ValueError(f"invalid start byte '{raw_start}' for field '{field_name}'") from exc

            base_type, endian_symbol = self._resolve_decode_type(field_type)
            running_offset = field_start + self._field_size(base_type)
            parsed.append(
                {
                    "name": field_name,
                    "start": str(field_start),
                    "type": base_type,
                    "endian": "BE" if endian_symbol == ">" else "LE",
                }
            )

        return parsed

    def _get_editor_field_defs(self) -> list[dict[str, str]]:
        if self.rule_fields_tree is None:
            return []

        field_defs: list[dict[str, str]] = []
        for item in self.rule_fields_tree.get_children():
            values = self.rule_fields_tree.item(item, "values")
            if len(values) < 4:
                continue
            field_defs.append(
                {
                    "name": str(values[0]).strip(),
                    "start": str(values[1]).strip() or "0",
                    "type": str(values[2]).strip().lower(),
                    "endian": str(values[3]).strip().upper() or "LE",
                }
            )

        return field_defs

    def _load_editor_field_defs(self, field_defs: list[dict[str, str]]) -> None:
        if self.rule_fields_tree is None:
            return

        for item in self.rule_fields_tree.get_children():
            self.rule_fields_tree.delete(item)

        for field in field_defs:
            self.rule_fields_tree.insert(
                "",
                tk.END,
                values=(
                    field.get("name", ""),
                    str(field.get("start", "0")),
                    field.get("type", "u8"),
                    field.get("endian", "LE"),
                ),
            )

        next_index = len(field_defs) + 1
        self.rule_field_name_var.set(f"field{next_index}")
        self.rule_field_start_var.set("0")

    def _rule_summary(self, field_defs: list[dict[str, str]]) -> str:
        return self._field_defs_to_spec(field_defs)

    def _on_rule_field_selected(self, _event: object) -> None:
        if self.rule_fields_tree is None:
            return

        selected = self.rule_fields_tree.selection()
        if len(selected) != 1:
            return

        values = self.rule_fields_tree.item(selected[0], "values")
        if len(values) < 4:
            return

        self.rule_field_name_var.set(str(values[0]))
        self.rule_field_start_var.set(str(values[1]))
        self.rule_field_type_var.set(str(values[2]))
        self.rule_field_endian_var.set(str(values[3]))

    def add_rule_field(self) -> None:
        if self.rule_fields_tree is None:
            return

        field_name = self.rule_field_name_var.get().strip()
        field_start_text = self.rule_field_start_var.get().strip() or "0"
        field_type = self.rule_field_type_var.get().strip().lower()
        field_endian = self.rule_field_endian_var.get().strip().upper() or "LE"

        if not field_name:
            messagebox.showerror("Decoder Rule Error", "Field name is required")
            return

        try:
            field_start = int(field_start_text, 0)
        except ValueError:
            messagebox.showerror("Decoder Rule Error", "Start byte must be an integer")
            return

        if field_start < 0:
            messagebox.showerror("Decoder Rule Error", "Start byte must be >= 0")
            return

        if field_type not in RULE_TYPE_OPTIONS:
            messagebox.showerror("Decoder Rule Error", f"Unsupported type '{field_type}'")
            return

        if field_endian not in RULE_ENDIAN_OPTIONS:
            messagebox.showerror("Decoder Rule Error", "Endian must be LE or BE")
            return

        existing_names = {str(self.rule_fields_tree.item(item, "values")[0]) for item in self.rule_fields_tree.get_children()}
        existing_starts: set[int] = set()
        for item in self.rule_fields_tree.get_children():
            values = self.rule_fields_tree.item(item, "values")
            if len(values) < 2:
                continue
            try:
                existing_starts.add(int(str(values[1]), 0))
            except ValueError:
                continue
        if field_name in existing_names:
            messagebox.showerror("Decoder Rule Error", f"Field '{field_name}' already exists")
            return
        if field_start in existing_starts:
            messagebox.showerror("Decoder Rule Error", f"Start byte {field_start} already exists")
            return

        self.rule_fields_tree.insert("", tk.END, values=(field_name, str(field_start), field_type, field_endian))
        self.rule_field_name_var.set(f"field{len(self.rule_fields_tree.get_children()) + 1}")
        self.rule_field_start_var.set("0")

    def update_rule_field(self) -> None:
        if self.rule_fields_tree is None:
            return

        selected = self.rule_fields_tree.selection()
        if len(selected) != 1:
            messagebox.showinfo("Decoder Rule", "Select one field row to update")
            return

        field_name = self.rule_field_name_var.get().strip()
        field_start_text = self.rule_field_start_var.get().strip() or "0"
        field_type = self.rule_field_type_var.get().strip().lower()
        field_endian = self.rule_field_endian_var.get().strip().upper() or "LE"

        if not field_name:
            messagebox.showerror("Decoder Rule Error", "Field name is required")
            return

        try:
            field_start = int(field_start_text, 0)
        except ValueError:
            messagebox.showerror("Decoder Rule Error", "Start byte must be an integer")
            return

        if field_start < 0:
            messagebox.showerror("Decoder Rule Error", "Start byte must be >= 0")
            return

        if field_type not in RULE_TYPE_OPTIONS:
            messagebox.showerror("Decoder Rule Error", f"Unsupported type '{field_type}'")
            return

        if field_endian not in RULE_ENDIAN_OPTIONS:
            messagebox.showerror("Decoder Rule Error", "Endian must be LE or BE")
            return

        target = selected[0]
        for item in self.rule_fields_tree.get_children():
            if item == target:
                continue
            values = self.rule_fields_tree.item(item, "values")
            if not values:
                continue
            if str(values[0]) == field_name:
                messagebox.showerror("Decoder Rule Error", f"Field '{field_name}' already exists")
                return
            try:
                existing_start = int(str(values[1]), 0)
            except ValueError:
                existing_start = None
            if existing_start == field_start:
                messagebox.showerror("Decoder Rule Error", f"Start byte {field_start} already exists")
                return

        self.rule_fields_tree.item(target, values=(field_name, str(field_start), field_type, field_endian))

    def remove_rule_field(self) -> None:
        if self.rule_fields_tree is None:
            return

        selected = self.rule_fields_tree.selection()
        if not selected:
            return

        for item in selected:
            self.rule_fields_tree.delete(item)

    def clear_rule_fields(self) -> None:
        if self.rule_fields_tree is None:
            return

        for item in self.rule_fields_tree.get_children():
            self.rule_fields_tree.delete(item)
        self.rule_field_name_var.set("field1")
        self.rule_field_start_var.set("0")

    def _upsert_decode_rule_row(self, can_id: int, field_defs: list[dict[str, str]]) -> None:
        if self.decode_rule_tree is None:
            return

        values = (f"0x{can_id:X}", self._rule_summary(field_defs))
        existing = self.decode_rule_tree_items.get(can_id)

        if existing and self.decode_rule_tree.exists(existing):
            self.decode_rule_tree.item(existing, values=values)
            return

        item = self.decode_rule_tree.insert("", tk.END, values=values)
        self.decode_rule_tree_items[can_id] = item

    def _remove_decode_rule_row(self, can_id: int) -> None:
        if self.decode_rule_tree is None:
            self.decode_rule_tree_items.pop(can_id, None)
            return

        item = self.decode_rule_tree_items.pop(can_id, None)
        if item and self.decode_rule_tree.exists(item):
            self.decode_rule_tree.delete(item)

    def _on_decode_rule_selected(self, _event: object) -> None:
        if self.decode_rule_tree is None:
            return

        selected = self.decode_rule_tree.selection()
        if len(selected) != 1:
            return

        item = selected[0]
        values = self.decode_rule_tree.item(item, "values")
        if not values:
            return

        can_id = self._parse_can_id_any(str(values[0]))
        self.rule_id_var.set(f"0x{can_id:X}")

        field_defs = self.custom_decode_fields_by_id.get(can_id)
        if field_defs is None:
            decode_spec = self.custom_decode_specs_by_id.get(can_id, "")
            field_defs = self._spec_to_field_defs(decode_spec) if decode_spec else []

        self._load_editor_field_defs(field_defs)

    def _on_group_row_selected(self, _event: object) -> None:
        tree = _event.widget if isinstance(_event.widget, ttk.Treeview) else self.group_tree
        selected = tree.selection()
        if len(selected) != 1:
            return

        values = tree.item(selected[0], "values")
        if not values:
            return

        can_id = self._parse_can_id_any(str(values[0]))
        self.rule_id_var.set(f"0x{can_id:X}")

        field_defs = self.custom_decode_fields_by_id.get(can_id)
        if field_defs is None:
            decode_spec = self.custom_decode_specs_by_id.get(can_id, "")
            field_defs = self._spec_to_field_defs(decode_spec) if decode_spec else []
        self._load_editor_field_defs(field_defs)

    def use_selected_group_id(self) -> None:
        selected = self.group_tree.selection()
        source = self.group_tree
        if len(selected) != 1:
            selected = self.decoded_group_tree.selection()
            source = self.decoded_group_tree

        if len(selected) != 1:
            messagebox.showinfo("Decoder Rule", "Select one row in Grouped by CAN ID first")
            return

        values = source.item(selected[0], "values")
        if not values:
            return

        can_id = self._parse_can_id_any(str(values[0]))
        self.rule_id_var.set(f"0x{can_id:X}")

        field_defs = self.custom_decode_fields_by_id.get(can_id)
        if field_defs is None:
            decode_spec = self.custom_decode_specs_by_id.get(can_id, "")
            field_defs = self._spec_to_field_defs(decode_spec) if decode_spec else []
        self._load_editor_field_defs(field_defs)

    def set_custom_decode_rule(self) -> None:
        try:
            can_id = self._parse_can_id_any(self.rule_id_var.get().strip())
            field_defs = self._get_editor_field_defs()
            if not field_defs:
                raise ValueError("Add at least one field before saving the rule")

            normalized: list[dict[str, str]] = []
            used_ranges: list[tuple[int, int, str]] = []
            for field in field_defs:
                field_name = str(field.get("name", "")).strip()
                if not field_name:
                    raise ValueError("Field name is required")

                start_text = str(field.get("start", "0")).strip() or "0"
                try:
                    start = int(start_text, 0)
                except ValueError as exc:
                    raise ValueError(f"Invalid start byte '{start_text}' for field '{field_name}'") from exc

                if start < 0:
                    raise ValueError(f"Start byte must be >= 0 for field '{field_name}'")

                field_type = str(field.get("type", "")).strip().lower()
                if field_type not in RULE_TYPE_OPTIONS:
                    raise ValueError(f"Unsupported type '{field_type}' for field '{field_name}'")

                field_endian = str(field.get("endian", "LE")).strip().upper() or "LE"
                if field_endian not in RULE_ENDIAN_OPTIONS:
                    raise ValueError(f"Endian must be LE or BE for field '{field_name}'")

                size = self._field_size(field_type)
                end = start + size
                if end > MAX_CAN_DATA_LEN:
                    raise ValueError(
                        f"Field '{field_name}' needs bytes [{start}..{end - 1}] but CAN payload is only 0..{MAX_CAN_DATA_LEN - 1}"
                    )

                for existing_start, existing_end, existing_name in used_ranges:
                    if max(start, existing_start) < min(end, existing_end):
                        raise ValueError(
                            f"Field '{field_name}' overlaps with '{existing_name}'"
                        )

                used_ranges.append((start, end, field_name))
                normalized.append(
                    {
                        "name": field_name,
                        "start": str(start),
                        "type": field_type,
                        "endian": field_endian,
                    }
                )

            field_defs = sorted(normalized, key=lambda field: int(str(field.get("start", "0")), 0))
            decode_spec = self._field_defs_to_spec(field_defs)
            self._spec_to_field_defs(decode_spec)
        except Exception as exc:
            messagebox.showerror("Decoder Rule Error", str(exc))
            return

        self.custom_decode_specs_by_id[can_id] = decode_spec
        self.custom_decode_fields_by_id[can_id] = field_defs
        self._load_editor_field_defs(field_defs)
        self._upsert_decode_rule_row(can_id, field_defs)
        self._refresh_decode_specs_by_id()
        self._save_session_state()
        self._set_status(f"Decoder rule set for ID 0x{can_id:X}")

    def delete_custom_decode_rule(self) -> None:
        can_id_text = self.rule_id_var.get().strip()
        if not can_id_text and self.decode_rule_tree is not None:
            selected = self.decode_rule_tree.selection()
            if len(selected) == 1:
                values = self.decode_rule_tree.item(selected[0], "values")
                if values:
                    can_id_text = str(values[0])

        try:
            can_id = self._parse_can_id_any(can_id_text)
        except Exception as exc:
            messagebox.showerror("Decoder Rule Error", str(exc))
            return

        if can_id not in self.custom_decode_specs_by_id:
            messagebox.showinfo("Decoder Rule", f"No custom rule exists for ID 0x{can_id:X}")
            return

        del self.custom_decode_specs_by_id[can_id]
        self.custom_decode_fields_by_id.pop(can_id, None)
        self._remove_decode_rule_row(can_id)
        self._refresh_decode_specs_by_id()
        self._save_session_state()
        self._set_status(f"Decoder rule removed for ID 0x{can_id:X}")

    def _refresh_decode_specs_by_id(self) -> None:
        self.decode_specs_by_id = dict(self.custom_decode_specs_by_id)

        for group_key, entry in self.grouped_packets_by_id.items():
            can_id = group_key[1]
            payload = entry.get("last_payload")
            if isinstance(payload, list):
                try:
                    entry["decoded"] = self._decode_payload(can_id, payload)
                except Exception as exc:
                    entry["decoded"] = f"decode-error: {exc}"
            self._upsert_grouped_row(group_key, entry)
            self._upsert_database_rows_for_group(group_key, entry)

    def _group_entry_matches_filters(self, group_key: tuple[int, int], entry: dict[str, object]) -> bool:
        channel, _ = group_key

        interface_filter = self.group_if_filter_var.get().strip().upper()
        if interface_filter == "CAN1" and channel != 0:
            return False
        if interface_filter == "CAN2" and channel != 1:
            return False

        direction_filter = self.group_dir_filter_var.get().strip().upper()
        tx_count = int(entry.get("tx_count", 0))
        rx_count = int(entry.get("rx_count", 0))

        if direction_filter == "RX" and rx_count <= 0:
            return False
        if direction_filter == "TX" and tx_count <= 0:
            return False

        return True

    def _on_group_filter_changed(self, _event: object) -> None:
        self._refresh_grouped_rows_by_filters()

    def reset_group_filters(self) -> None:
        self.group_if_filter_var.set("All")
        self.group_dir_filter_var.set("All")
        self._refresh_grouped_rows_by_filters()

    def _refresh_grouped_rows_by_filters(self) -> None:
        for group_key, entry in self.grouped_packets_by_id.items():
            self._upsert_grouped_row(group_key, entry)

        self._refresh_database_view()

    def _message_label_for_group(self, channel: int, frame_id: int) -> str:
        exact_matches: list[MessageTemplate] = []
        id_matches: list[MessageTemplate] = []

        for msg in self.templates_by_item.values():
            if msg.frame_id == frame_id:
                id_matches.append(msg)
                if msg.channel == channel:
                    exact_matches.append(msg)

        if exact_matches:
            return exact_matches[-1].name
        if id_matches:
            return id_matches[-1].name
        return "-"

    def _parse_decoded_pairs(self, decoded_text: str) -> dict[str, str]:
        parsed: dict[str, str] = {}
        for token in decoded_text.split(","):
            part = token.strip()
            if not part or "=" not in part:
                continue
            name, value = part.split("=", 1)
            key = name.strip()
            if not key:
                continue
            parsed[key] = value.strip()
        return parsed

    def _database_rows_for_group(
        self,
        group_key: tuple[int, int],
        entry: dict[str, object],
    ) -> list[tuple[tuple[int, int, str], tuple[str, ...]]]:
        if not self._group_entry_matches_filters(group_key, entry):
            return []

        channel, frame_id = group_key
        rx_count = str(entry.get("rx_count", 0))
        tx_count = str(entry.get("tx_count", 0))
        last_seen = str(entry.get("last_seen", ""))
        decoded_text = str(entry.get("decoded", ""))
        raw_text = str(entry.get("raw", ""))
        decoded_pairs = self._parse_decoded_pairs(decoded_text)

        rows: list[tuple[tuple[int, int, str], tuple[str, ...]]] = []
        field_defs = self.custom_decode_fields_by_id.get(frame_id, [])
        if field_defs:
            for field in field_defs:
                signal_name = str(field.get("name", "")).strip() or "field"
                value_text = decoded_pairs.get(signal_name, "")
                if not value_text:
                    value_text = decoded_text if decoded_text.startswith("decode-error") else "-"

                row_key = (channel, frame_id, signal_name)
                row_values = (
                    signal_name,
                    value_text,
                    rx_count,
                    tx_count,
                    last_seen,
                )
                rows.append((row_key, row_values))
            return rows

        if decoded_pairs:
            for signal_name, value_text in decoded_pairs.items():
                row_key = (channel, frame_id, signal_name)
                row_values = (
                    signal_name,
                    value_text,
                    rx_count,
                    tx_count,
                    last_seen,
                )
                rows.append((row_key, row_values))
            return rows

        fallback_value = decoded_text if decoded_text else (raw_text or "-")
        fallback_signal = "frame"
        row_key = (channel, frame_id, fallback_signal)
        row_values = (
            fallback_signal,
            fallback_value,
            rx_count,
            tx_count,
            last_seen,
        )
        rows.append((row_key, row_values))
        return rows

    def _delete_database_row(self, row_key: tuple[int, int, str]) -> None:
        item = self.database_tree_items.pop(row_key, None)
        if item and self.database_tree.exists(item):
            self.database_tree.delete(item)

    def _upsert_database_rows_for_group(self, group_key: tuple[int, int], entry: dict[str, object]) -> None:
        rows = self._database_rows_for_group(group_key, entry)
        new_keys = {row_key for row_key, _ in rows}
        current_keys = self.database_row_keys_by_group.get(group_key, set())

        for stale_key in current_keys - new_keys:
            self._delete_database_row(stale_key)

        for row_key, row_values in rows:
            existing = self.database_tree_items.get(row_key)
            if existing and self.database_tree.exists(existing):
                self.database_tree.item(existing, values=row_values)
            else:
                item = self.database_tree.insert("", tk.END, values=row_values)
                self.database_tree_items[row_key] = item

        if new_keys:
            self.database_row_keys_by_group[group_key] = new_keys
        else:
            self.database_row_keys_by_group.pop(group_key, None)

    def _refresh_database_view(self) -> None:
        for group_key in list(self.database_row_keys_by_group.keys()):
            if group_key not in self.grouped_packets_by_id:
                for row_key in self.database_row_keys_by_group[group_key]:
                    self._delete_database_row(row_key)
                self.database_row_keys_by_group.pop(group_key, None)

        for group_key, entry in self.grouped_packets_by_id.items():
            self._upsert_database_rows_for_group(group_key, entry)

    def _grouped_raw_row_values(self, group_key: tuple[int, int], entry: dict[str, object]) -> tuple[str, ...]:
        channel, can_id = group_key
        return (
            f"0x{can_id:X}",
            str(entry.get("last_if", f"CAN{channel + 1}")),
            str(entry.get("format", "")),
            str(entry.get("type", "")),
            str(entry.get("tx_count", 0)),
            str(entry.get("rx_count", 0)),
            str(entry.get("last_dir", "")),
            str(entry.get("dlc", "")),
            str(entry.get("raw", "")),
            str(entry.get("last_seen", "")),
        )

    def _grouped_decoded_row_values(self, group_key: tuple[int, int], entry: dict[str, object]) -> tuple[str, ...]:
        channel, can_id = group_key
        return (
            f"0x{can_id:X}",
            str(entry.get("last_if", f"CAN{channel + 1}")),
            str(entry.get("format", "")),
            str(entry.get("type", "")),
            str(entry.get("tx_count", 0)),
            str(entry.get("rx_count", 0)),
            str(entry.get("last_dir", "")),
            str(entry.get("dlc", "")),
            str(entry.get("decoded", "")),
            str(entry.get("last_seen", "")),
        )

    def _upsert_grouped_row(self, group_key: tuple[int, int], entry: dict[str, object]) -> None:
        raw_existing = self.group_tree_items.get(group_key)
        decoded_existing = self.decoded_group_tree_items.get(group_key)

        if not self._group_entry_matches_filters(group_key, entry):
            if raw_existing and self.group_tree.exists(raw_existing):
                self.group_tree.delete(raw_existing)
            self.group_tree_items.pop(group_key, None)

            if decoded_existing and self.decoded_group_tree.exists(decoded_existing):
                self.decoded_group_tree.delete(decoded_existing)
            self.decoded_group_tree_items.pop(group_key, None)
            return

        raw_values = self._grouped_raw_row_values(group_key, entry)

        if raw_existing and self.group_tree.exists(raw_existing):
            self.group_tree.item(raw_existing, values=raw_values)
        else:
            raw_item = self.group_tree.insert("", tk.END, values=raw_values)
            self.group_tree_items[group_key] = raw_item

        decoded_values = self._grouped_decoded_row_values(group_key, entry)

        if decoded_existing and self.decoded_group_tree.exists(decoded_existing):
            self.decoded_group_tree.item(decoded_existing, values=decoded_values)
        else:
            decoded_item = self.decoded_group_tree.insert("", tk.END, values=decoded_values)
            self.decoded_group_tree_items[group_key] = decoded_item

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
        active_decode_spec = self.custom_decode_specs_by_id.get(frame_id)
        if not active_decode_spec and decode_spec and decode_spec.strip():
            active_decode_spec = decode_spec.strip()
            self.decode_specs_by_id[frame_id] = active_decode_spec

        group_key = (channel, frame_id)
        entry = self.grouped_packets_by_id.get(group_key)
        if entry is None:
            entry = {
                "format": "EXT" if extended else "STD",
                "type": "RTR" if remote else "DATA",
                "tx_count": 0,
                "rx_count": 0,
                "last_dir": "",
                "last_if": f"CAN{channel + 1}",
                "dlc": 0,
                "raw": "",
                "decoded": "",
                "last_seen": "",
                "last_payload": [],
            }
            self.grouped_packets_by_id[group_key] = entry

        if direction == "TX":
            entry["tx_count"] = int(entry.get("tx_count", 0)) + 1
        else:
            entry["rx_count"] = int(entry.get("rx_count", 0)) + 1

        raw_text = " ".join(f"{byte:02X}" for byte in payload)
        decoded = ""
        try:
            decoded = self._decode_payload(frame_id, payload, decode_spec=active_decode_spec)
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

        self._upsert_grouped_row(group_key, entry)
        self._upsert_database_rows_for_group(group_key, entry)

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _parse_message_from_form(self) -> MessageTemplate:
        name = self.name_var.get().strip() or "Unnamed"
        channel = 0 if self.channel_var.get() == "CAN1" else 1
        frame_id = self._parse_can_id(self.id_var.get().strip(), bool(self.extended_var.get()))
        extended = bool(self.extended_var.get())
        remote = bool(self.remote_var.get())
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
            self._save_session_state()
            self._set_status(f"Updated template '{msg.name}'")
            return

        item = self.template_tree.insert("", tk.END, values=self._template_values(msg))
        self.templates_by_item[item] = msg
        self._refresh_decode_specs_by_id()
        self._save_session_state()
        self._set_status(f"Saved template '{msg.name}'")

    def delete_template(self) -> None:
        selected = self.template_tree.selection()
        if not selected:
            return

        for item in selected:
            self.template_tree.delete(item)
            self.templates_by_item.pop(item, None)

        self._refresh_decode_specs_by_id()
        self._save_session_state()
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
        self.decoded_group_tree_items.clear()
        self.database_tree_items.clear()
        self.database_row_keys_by_group.clear()
        for item in self.group_tree.get_children():
            self.group_tree.delete(item)
        for item in self.decoded_group_tree.get_children():
            self.decoded_group_tree.delete(item)
        for item in self.database_tree.get_children():
            self.database_tree.delete(item)
        self._save_session_state()
        self._set_status("Cleared grouped-by-ID view")

    def clear_can1_log(self) -> None:
        self.rx_records_can1.clear()
        self._set_status("Cleared CAN1 RX log")

    def clear_can2_log(self) -> None:
        self.rx_records_can2.clear()
        self._set_status("Cleared CAN2 RX log")

    def _on_close(self) -> None:
        try:
            self._save_session_state()
        except Exception:
            pass

        try:
            self.disconnect_device()
        finally:
            self.destroy()


def main() -> None:
    app = CanGuiApp()
    app.mainloop()


if __name__ == "__main__":
    main()
