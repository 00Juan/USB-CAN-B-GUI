#!/usr/bin/env python3
"""CAN1/CAN2 GUI sender/receiver for ZLGCAN-compatible libcontrolcan.so.

Features:
- Configure user-defined CAN frames (template list)
- Send selected frame via CAN1 or CAN2
- Display live packets received by CAN1 and CAN2
"""

from __future__ import annotations

import ctypes
import queue
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk


VCI_USBCAN2 = 4
STATUS_OK = 1
MAX_CAN_DATA_LEN = 8


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


@dataclass
class MessageTemplate:
    name: str
    channel: int  # 0 for CAN1, 1 for CAN2
    frame_id: int
    extended: bool
    remote: bool
    dlc: int
    data: list[int]


class CanGuiApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("CAN1/CAN2 Message Console")
        self.geometry("1200x760")
        self.minsize(1000, 650)

        self.api_lock = threading.Lock()
        self.rx_queue: queue.Queue[tuple] = queue.Queue()
        self.stop_event = threading.Event()
        self.rx_thread: threading.Thread | None = None

        self.connected = False
        self.templates_by_item: dict[str, MessageTemplate] = {}

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

        columns = ("name", "if", "id", "format", "type", "dlc", "data")
        self.template_tree = ttk.Treeview(outer, columns=columns, show="headings", height=7)
        self.template_tree.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        headings = {
            "name": "Name",
            "if": "Interface",
            "id": "ID",
            "format": "Format",
            "type": "Type",
            "dlc": "DLC",
            "data": "Data",
        }
        widths = {
            "name": 140,
            "if": 90,
            "id": 90,
            "format": 90,
            "type": 80,
            "dlc": 60,
            "data": 360,
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
        ttk.Button(row, text="Clear CAN1", command=lambda: self._clear_text(self.rx_text_can1)).pack(side=tk.LEFT)
        ttk.Button(row, text="Clear CAN2", command=lambda: self._clear_text(self.rx_text_can2)).pack(side=tk.LEFT, padx=(6, 0))

        logs = ttk.Frame(outer)
        logs.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        can1_frame = ttk.LabelFrame(logs, text="CAN1 RX")
        can1_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))
        self.rx_text_can1 = tk.Text(can1_frame, height=14, wrap="none")
        self.rx_text_can1.pack(fill=tk.BOTH, expand=True)

        can2_frame = ttk.LabelFrame(logs, text="CAN2 RX")
        can2_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 0))
        self.rx_text_can2 = tk.Text(can2_frame, height=14, wrap="none")
        self.rx_text_can2.pack(fill=tk.BOTH, expand=True)

    def _set_connected_ui(self, connected: bool) -> None:
        self.connect_btn.configure(state=tk.DISABLED if connected else tk.NORMAL)
        self.disconnect_btn.configure(state=tk.NORMAL if connected else tk.DISABLED)

        for btn in (self.send_btn, self.send_selected_btn):
            btn.configure(state=tk.NORMAL if connected else tk.DISABLED)

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
            count += 1

        self.after(50, self._process_rx_queue)

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _parse_message_from_form(self) -> MessageTemplate:
        name = self.name_var.get().strip() or "Unnamed"
        channel = 0 if self.channel_var.get() == "CAN1" else 1
        frame_id = self._parse_can_id(self.id_var.get().strip(), bool(self.extended_var.get()))
        extended = bool(self.extended_var.get())
        remote = bool(self.remote_var.get())

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
            self._set_status(f"Updated template '{msg.name}'")
            return

        item = self.template_tree.insert("", tk.END, values=self._template_values(msg))
        self.templates_by_item[item] = msg
        self._set_status(f"Saved template '{msg.name}'")

    def delete_template(self) -> None:
        selected = self.template_tree.selection()
        if not selected:
            return

        for item in selected:
            self.template_tree.delete(item)
            self.templates_by_item.pop(item, None)

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

    def _send_message(self, msg: MessageTemplate) -> None:
        if not self.connected or not self.dll:
            raise RuntimeError("Device is not connected")

        frame = self._to_can_object(msg)
        with self.api_lock:
            sent = int(self.dll.VCI_Transmit(VCI_USBCAN2, 0, msg.channel, ctypes.byref(frame), 1))

        if sent != 1:
            raise RuntimeError(f"VCI_Transmit failed on CAN{msg.channel + 1}")

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
