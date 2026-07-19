#!/usr/bin/env python3
"""Small Tkinter UI for scripts/ble_ota_loader.py."""

from __future__ import annotations

import asyncio
import queue
import subprocess
import sys
import threading
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import ble_ota_loader


APP_MODE = "Application reboot + upload"
LOADER_MODE = "Loader recovery only"


@dataclass(frozen=True)
class DeviceRow:
    name: str
    address: str
    rssi: int | None
    services: str

    @property
    def label(self) -> str:
        rssi = "?" if self.rssi is None else str(self.rssi)
        return f"{self.name} [{self.address}] RSSI={rssi}"


class BleOtaGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("BLE OTA Loader")
        self.geometry("860x600")
        self.minsize(760, 500)

        self.devices: list[DeviceRow] = []
        self.log_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self.worker: threading.Thread | None = None
        self.process: subprocess.Popen[str] | None = None

        self.file_var = tk.StringVar()
        self.mode_var = tk.StringVar(value=APP_MODE)
        self.status_var = tk.StringVar(value="Idle")
        self.scan_timeout_var = tk.StringVar(value="5")

        self._build_ui()
        self.after(100, self._drain_log_queue)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)

        file_row = ttk.Frame(outer)
        file_row.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        file_row.columnconfigure(1, weight=1)
        ttk.Label(file_row, text="Firmware").grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Entry(file_row, textvariable=self.file_var).grid(row=0, column=1, sticky="ew", padx=(0, 8))
        ttk.Button(file_row, text="Browse...", command=self._browse_firmware).grid(row=0, column=2)

        controls = ttk.Frame(outer)
        controls.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        controls.columnconfigure(1, weight=1)
        ttk.Label(controls, text="Mode").grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Combobox(
            controls,
            textvariable=self.mode_var,
            values=(APP_MODE, LOADER_MODE),
            state="readonly",
            width=28,
        ).grid(row=0, column=1, sticky="w")
        ttk.Label(controls, text="Scan seconds").grid(row=0, column=2, sticky="e", padx=(20, 8))
        ttk.Entry(controls, textvariable=self.scan_timeout_var, width=6).grid(row=0, column=3, sticky="w")
        ttk.Button(controls, text="Scan", command=self._scan_devices).grid(row=0, column=4, padx=(12, 0))
        ttk.Button(controls, text="Upload", command=self._upload).grid(row=0, column=5, padx=(8, 0))

        body = ttk.PanedWindow(outer, orient=tk.HORIZONTAL)
        body.grid(row=2, column=0, sticky="nsew")

        left = ttk.Frame(body, padding=(0, 0, 8, 0))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        ttk.Label(left, text="BLE Devices").grid(row=0, column=0, sticky="w")
        self.device_list = tk.Listbox(left, exportselection=False, height=14)
        self.device_list.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        self.device_list.bind("<<ListboxSelect>>", lambda _event: self._show_selected_device())
        self.device_detail = tk.Text(left, height=6, wrap="word", state="disabled")
        self.device_detail.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        body.add(left, weight=1)

        right = ttk.Frame(body)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        ttk.Label(right, text="Log").grid(row=0, column=0, sticky="w")
        self.log = tk.Text(right, height=18, wrap="word", state="disabled")
        self.log.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        body.add(right, weight=2)

        status = ttk.Frame(outer)
        status.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        status.columnconfigure(0, weight=1)
        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=0, sticky="w")

    def _browse_firmware(self) -> None:
        path = filedialog.askopenfilename(
            title="Select firmware .bin",
            filetypes=(("Firmware binaries", "*.bin"), ("All files", "*.*")),
        )
        if path:
            self.file_var.set(path)

    def _selected_device(self) -> DeviceRow | None:
        selection = self.device_list.curselection()
        if not selection:
            return None
        return self.devices[selection[0]]

    def _show_selected_device(self) -> None:
        device = self._selected_device()
        self.device_detail.configure(state="normal")
        self.device_detail.delete("1.0", tk.END)
        if device:
            self.device_detail.insert(
                tk.END,
                f"Name: {device.name}\nAddress: {device.address}\nRSSI: {device.rssi}\nServices: {device.services}",
            )
        self.device_detail.configure(state="disabled")

    def _set_busy(self, busy: bool, status: str) -> None:
        self.status_var.set(status)
        cursor = "watch" if busy else ""
        self.configure(cursor=cursor)

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert(tk.END, text)
        self.log.see(tk.END)
        self.log.configure(state="disabled")

    def _drain_log_queue(self) -> None:
        try:
            while True:
                kind, text = self.log_queue.get_nowait()
                if kind == "log":
                    self._append_log(text)
                elif kind == "status":
                    self.status_var.set(text)
                elif kind == "scan_done":
                    self._load_devices(text)
                    self._set_busy(False, f"Found {len(self.devices)} BLE device(s)")
                elif kind == "done":
                    self._set_busy(False, text)
        except queue.Empty:
            pass
        self.after(100, self._drain_log_queue)

    def _load_devices(self, encoded: str) -> None:
        self.devices.clear()
        self.device_list.delete(0, tk.END)
        for line in encoded.splitlines():
            name, address, rssi, services = line.split("\t", 3)
            self.devices.append(DeviceRow(name, address, None if rssi == "" else int(rssi), services))
        for index, device in enumerate(self.devices):
            self.device_list.insert(tk.END, device.label)
            if device.name.startswith((ble_ota_loader.APP_NAME_PREFIX, ble_ota_loader.LOADER_NAME_PREFIX)):
                self.device_list.selection_clear(0, tk.END)
                self.device_list.selection_set(index)
                self.device_list.activate(index)
        self._show_selected_device()

    def _run_worker(self, target, status: str) -> None:
        if self.worker and self.worker.is_alive():
            messagebox.showwarning("Busy", "An operation is already running.")
            return
        self._set_busy(True, status)
        self.worker = threading.Thread(target=target, daemon=True)
        self.worker.start()

    def _scan_devices(self) -> None:
        try:
            timeout = float(self.scan_timeout_var.get())
        except ValueError:
            messagebox.showerror("Invalid scan timeout", "Scan seconds must be a number.")
            return
        if timeout <= 0:
            messagebox.showerror("Invalid scan timeout", "Scan seconds must be greater than zero.")
            return

        def worker() -> None:
            try:
                rows = asyncio.run(self._scan_async(timeout))
                encoded = "\n".join(
                    f"{row.name}\t{row.address}\t{'' if row.rssi is None else row.rssi}\t{row.services}"
                    for row in rows
                )
                self.log_queue.put(("scan_done", encoded))
            except Exception as exc:
                self.log_queue.put(("log", f"Scan failed: {exc}\n"))
                self.log_queue.put(("done", "Scan failed"))

        self.log_queue.put(("log", "Scanning for BLE devices...\n"))
        self._run_worker(worker, "Scanning...")

    async def _scan_async(self, timeout: float) -> list[DeviceRow]:
        discovered = await ble_ota_loader.discover(timeout)
        rows: list[DeviceRow] = []
        for device, adv in discovered.values():
            name = device.name or adv.local_name or "<unnamed>"
            services = ", ".join(adv.service_uuids or []) or "none advertised"
            rows.append(DeviceRow(name, device.address, adv.rssi, services))
        rows.sort(key=lambda row: (not row.name.startswith((ble_ota_loader.APP_NAME_PREFIX, ble_ota_loader.LOADER_NAME_PREFIX)), row.name))
        return rows

    def _validate_firmware(self) -> Path | None:
        raw = self.file_var.get().strip()
        if not raw:
            messagebox.showerror("Missing firmware", "Select a firmware .bin file.")
            return None
        path = Path(raw)
        if not path.is_file():
            messagebox.showerror("Missing firmware", f"Firmware file does not exist:\n{path}")
            return None
        if path.suffix.lower() != ".bin":
            messagebox.showerror("Wrong file type", f"Firmware file must be a .bin file:\n{path}")
            return None
        return path

    def _upload(self) -> None:
        firmware = self._validate_firmware()
        if firmware is None:
            return

        selected = self._selected_device()
        mode = self.mode_var.get()
        cmd = [sys.executable, str(Path(__file__).with_name("ble_ota_loader.py")), "--file", str(firmware)]
        if mode == LOADER_MODE:
            cmd.append("--loader-only")
            if selected:
                cmd.extend(["--loader-address", selected.address])
        elif selected:
            cmd.extend(["--address", selected.address])

        def worker() -> None:
            self.log_queue.put(("log", "\n$ " + " ".join(cmd) + "\n"))
            try:
                self.process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                assert self.process.stdout is not None
                for line in self.process.stdout:
                    self.log_queue.put(("log", line))
                return_code = self.process.wait()
                status = "Upload complete" if return_code == 0 else f"Upload failed ({return_code})"
                self.log_queue.put(("done", status))
            except Exception as exc:
                self.log_queue.put(("log", f"Upload failed: {exc}\n"))
                self.log_queue.put(("done", "Upload failed"))
            finally:
                self.process = None

        self._run_worker(worker, "Uploading...")


def main() -> int:
    app = BleOtaGui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
