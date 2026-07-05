#!/usr/bin/env python3
"""BLE OTA loader matching ../web/webui.html.

Protocol mirrored from the WebUI:
- app reboot characteristic: 0000fe11-8e22-4541-9d4c-21edae82ed19
- reboot payload: [0x01, start_sector, num_sectors]
- OTA loader service: 0000fe20-cc7a-482a-984a-7f2ed5b3e58f
- base/control characteristic: 0000fe22-8e22-4541-9d4c-21edae82ed19
- confirmation characteristic: 0000fe23-8e22-4541-9d4c-21edae82ed19
- data characteristic: 0000fe24-8e22-4541-9d4c-21edae82ed19
- upload data chunks: 20-byte write-without-response chunks
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

try:
    from bleak import BleakClient, BleakError, BleakScanner
    BLEAK_IMPORT_ERROR = None
except ImportError as exc:
    BleakClient = None
    BleakScanner = None
    BleakError = Exception
    BLEAK_IMPORT_ERROR = exc


APP_NAME_PREFIX = "3000GT"
LOADER_NAME_PREFIX = "STM_OTA"
APP_BASE_OFFSET = 0x0007000
PAGE_SIZE = 4096
DATA_CHUNK_SIZE = 20

REBOOT_SERVICE_UUID = "00000a00-cc7a-482a-984a-7f2ed5b3e58f"
REBOOT_CHAR_UUID = "0000fe11-8e22-4541-9d4c-21edae82ed19"

OTA_SERVICE_UUID = "0000fe20-cc7a-482a-984a-7f2ed5b3e58f"
OTA_BASE_CHAR_UUID = "0000fe22-8e22-4541-9d4c-21edae82ed19"
OTA_CONF_CHAR_UUID = "0000fe23-8e22-4541-9d4c-21edae82ed19"
OTA_DATA_CHAR_UUID = "0000fe24-8e22-4541-9d4c-21edae82ed19"
OTA_EVT_CHAR_UUID = "0000fe25-8e22-4541-9d4c-21edae82ed19"

START_APPLICATION_UPLOAD = 0x02
UPLOAD_FINISHED = 0x07


class OtaError(RuntimeError):
    pass


def require_bleak() -> None:
    if BLEAK_IMPORT_ERROR is not None:
        raise OtaError(
            "missing BLE dependency; install it with: "
            "python -m pip install -r scripts/requirements-ble-ota.txt"
        )


@dataclass(frozen=True)
class Target:
    address: str | None
    name_prefix: str | None


def parse_hex_bytes(value: str) -> bytes:
    compact = value.strip().lower().removeprefix("0x")
    compact = compact.replace(" ", "").replace(":", "").replace("-", "")
    if len(compact) % 2:
        compact = "0" + compact
    try:
        return bytes.fromhex(compact)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid hex payload: {value!r}") from exc


def chunked(data: bytes, size: int) -> Iterable[bytes]:
    for offset in range(0, len(data), size):
        yield data[offset : offset + size]


def base_command(action: int, base_offset: int) -> bytes:
    base_offset &= 0xFFFFFF
    return bytes(
        [
            action,
            base_offset & 0xFF,
            (base_offset >> 8) & 0xFF,
            (base_offset >> 16) & 0xFF,
        ]
    )


def default_reboot_payload(bin_size: int, base_offset: int, page_size: int) -> bytes:
    start_sector = base_offset // page_size
    num_sectors = math.ceil(bin_size / page_size) if bin_size else 0
    if start_sector > 0xFF or num_sectors > 0xFF:
        raise OtaError(
            f"reboot payload exceeds one-byte sector fields: start={start_sector}, count={num_sectors}"
        )
    return bytes([0x01, start_sector & 0xFF, num_sectors & 0xFF])


def device_name(device, adv=None) -> str:
    name = device.name or (adv.local_name if adv is not None else None) or "<unnamed>"
    return f"{name} [{device.address}]"


def adv_services(adv) -> set[str]:
    return {uuid.lower() for uuid in (adv.service_uuids or [])}


def properties_string(char) -> str:
    return ",".join(char.properties) if char.properties else "none"


async def discover(timeout: float):
    require_bleak()
    try:
        return await BleakScanner.discover(timeout=timeout, return_adv=True)
    except Exception as exc:
        raise OtaError(f"BLE scan failed: {exc}") from exc


def target_matches(device, adv, target: Target, service_uuid: str | None = None) -> bool:
    if target.address and device.address.lower() == target.address.lower():
        return True
    name = device.name or adv.local_name or ""
    if target.name_prefix and name.startswith(target.name_prefix):
        return True
    if service_uuid and service_uuid.lower() in adv_services(adv):
        return True
    return False


async def find_device(args, target: Target, *, phase: str, service_uuid: str | None = None):
    deadline = time.monotonic() + args.reappear_timeout
    while True:
        matches = []
        for device, adv in (await discover(args.scan_timeout)).values():
            if target_matches(device, adv, target, service_uuid):
                matches.append((device, adv))
        if matches:
            if len(matches) > 1 and not target.address:
                print(f"Found {len(matches)} matches for {phase}; using first:")
                for device, adv in matches:
                    print(f"  {device_name(device, adv)}")
            return matches[0][0]
        if time.monotonic() >= deadline:
            raise OtaError(f"no BLE device found for {phase}")
        await asyncio.sleep(1.0)


async def inspect(args) -> None:
    discovered = await discover(args.scan_timeout)
    rows = []
    for device, adv in discovered.values():
        if args.list_all or target_matches(device, adv, Target(args.address, args.name_prefix), args.ota_service_uuid):
            rows.append((device, adv))

    if not rows:
        print("No matching BLE devices found.")
        return

    for device, adv in rows:
        services = ", ".join(adv.service_uuids or []) or "none advertised"
        print(f"{device_name(device, adv)} RSSI={adv.rssi} services={services}")

    if not args.inspect_gatt:
        return

    for device, adv in rows:
        print(f"\nGATT for {device_name(device, adv)}")
        try:
            async with BleakClient(device, timeout=args.connect_timeout) as client:
                for service in client.services:
                    print(f"  service {service.uuid}")
                    for char in service.characteristics:
                        print(f"    char {char.uuid} props={properties_string(char)}")
        except Exception as exc:
            print(f"  failed to inspect: {exc}")


async def write_char(client: BleakClient, uuid: str, payload: bytes, *, response: bool | None) -> None:
    await client.write_gatt_char(uuid, payload, response=response)


def is_reboot_disconnect_error(exc: BaseException) -> bool:
    text = str(exc).lower()
    return "operation aborted" in text or "not connected" in text or "device is unreachable" in text


async def reboot_to_loader(args, firmware_size: int) -> None:
    target = Target(args.address, args.name_prefix)
    device = await find_device(args, target, phase="application reboot", service_uuid=REBOOT_SERVICE_UUID)
    payload = args.reboot_payload or default_reboot_payload(firmware_size, args.base_offset, args.page_size)
    print(f"Connecting to app {device_name(device)}")
    print(f"Reboot payload: {' '.join(f'{b:02x}' for b in payload)}")
    try:
        async with BleakClient(device, timeout=args.connect_timeout) as client:
            await write_char(client, args.reboot_char_uuid, payload, response=args.reboot_response)
    except (BleakError, OSError) as exc:
        if is_reboot_disconnect_error(exc):
            print("Reboot request likely accepted; device disconnected during reset.")
            return
        raise OtaError(f"failed to request OTA reboot: {exc}") from exc


async def upload(args, firmware: bytes) -> None:
    target = Target(args.loader_address or args.address, args.loader_prefix)
    device = await find_device(args, target, phase="OTA loader", service_uuid=args.ota_service_uuid)
    total_chunks = math.ceil(len(firmware) / args.chunk_size)
    confirmation = asyncio.Event()

    def on_conf(_sender: int, data: bytearray) -> None:
        print("Confirmation:", " ".join(f"{b:02x}" for b in data))
        confirmation.set()

    print(f"Connecting to OTA loader {device_name(device)}")
    try:
        async with BleakClient(device, timeout=args.connect_timeout) as client:
            if args.inspect_gatt:
                for service in client.services:
                    print(f"  service {service.uuid}")
                    for char in service.characteristics:
                        print(f"    char {char.uuid} props={properties_string(char)}")

            with contextlib.suppress(Exception):
                await client.start_notify(args.ota_conf_char_uuid, on_conf)

            start = base_command(START_APPLICATION_UPLOAD, args.base_offset)
            finish = base_command(UPLOAD_FINISHED, args.base_offset)
            end_offset = args.base_offset + math.ceil(len(firmware) / args.page_size) * args.page_size
            print(
                f"Upload base=0x{args.base_offset:06x} end=0x{end_offset:06x} "
                f"size={len(firmware)} chunks={total_chunks} chunk_size={args.chunk_size}"
            )
            await write_char(client, args.ota_base_char_uuid, start, response=None)

            started = time.monotonic()
            for index, chunk in enumerate(chunked(firmware, args.chunk_size), start=1):
                await write_char(client, args.ota_data_char_uuid, chunk, response=False)
                if args.chunk_delay:
                    await asyncio.sleep(args.chunk_delay)
                if index == 1 or index == total_chunks or index % args.progress_every == 0:
                    print(f"  chunk {index}/{total_chunks} ({index * 100.0 / total_chunks:.1f}%)")

            await write_char(client, args.ota_base_char_uuid, finish, response=None)
            print(f"Finish command sent after {time.monotonic() - started:.1f}s")

            if args.wait_confirmation:
                try:
                    await asyncio.wait_for(confirmation.wait(), timeout=args.confirm_timeout)
                except asyncio.TimeoutError:
                    print("No confirmation received before timeout.")

            with contextlib.suppress(Exception):
                await client.stop_notify(args.ota_conf_char_uuid)
    except BleakError as exc:
        raise OtaError(f"OTA upload failed: {exc}") from exc


async def run(args) -> None:
    if args.scan:
        await inspect(args)
        return

    firmware = b""
    if args.file is not None:
        firmware = args.file.read_bytes()
        if not firmware:
            raise OtaError(f"firmware file is empty: {args.file}")
    elif not args.reboot_only:
        raise OtaError("--file is required unless --reboot-only is used")

    if args.loader_only:
        if args.reboot_only:
            raise OtaError("--loader-only cannot be combined with --reboot-only")
        await upload(args, firmware)
        return

    await reboot_to_loader(args, len(firmware))
    if args.reboot_only:
        return

    print("Waiting for OTA loader advertisement...")
    await asyncio.sleep(args.post_reboot_delay)
    await upload(args, firmware)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Upload STM32WB firmware over BLE OTA.")
    parser.add_argument("--file", type=Path, help="firmware .bin to upload")
    parser.add_argument("--name-prefix", default=APP_NAME_PREFIX, help="application BLE name prefix")
    parser.add_argument("--address", help="application BLE address/id")
    parser.add_argument("--loader-prefix", default=LOADER_NAME_PREFIX, help="OTA loader BLE name prefix")
    parser.add_argument("--loader-address", help="OTA loader BLE address/id")
    parser.add_argument("--loader-only", action="store_true", help="skip app reboot and upload to an already-advertising OTA loader")
    parser.add_argument("--reboot-only", action="store_true", help="only write the OTA reboot request")
    parser.add_argument(
        "--reboot-response",
        action="store_true",
        default=None,
        help="force write-with-response for the reboot request; default auto-selects from characteristic properties like the WebUI",
    )
    parser.add_argument("--reboot-payload", type=parse_hex_bytes, help="override auto [01,start_sector,num_sectors]")
    parser.add_argument("--scan", action="store_true", help="list matching BLE devices and exit")
    parser.add_argument("--list-all", action="store_true", help="with --scan, list all BLE devices")
    parser.add_argument("--inspect-gatt", action="store_true", help="print GATT services/characteristics")
    parser.add_argument("--base-offset", type=lambda value: int(value, 0), default=APP_BASE_OFFSET)
    parser.add_argument("--page-size", type=int, default=PAGE_SIZE)
    parser.add_argument("--chunk-size", type=int, default=DATA_CHUNK_SIZE)
    parser.add_argument("--chunk-delay", type=float, default=0.0)
    parser.add_argument("--reboot-char-uuid", default=REBOOT_CHAR_UUID)
    parser.add_argument("--ota-service-uuid", default=OTA_SERVICE_UUID)
    parser.add_argument("--ota-base-char-uuid", default=OTA_BASE_CHAR_UUID)
    parser.add_argument("--ota-conf-char-uuid", default=OTA_CONF_CHAR_UUID)
    parser.add_argument("--ota-data-char-uuid", default=OTA_DATA_CHAR_UUID)
    parser.add_argument("--ota-evt-char-uuid", default=OTA_EVT_CHAR_UUID)
    parser.add_argument("--scan-timeout", type=float, default=8.0)
    parser.add_argument("--connect-timeout", type=float, default=15.0)
    parser.add_argument("--reappear-timeout", type=float, default=25.0)
    parser.add_argument("--post-reboot-delay", type=float, default=2.0)
    parser.add_argument("--confirm-timeout", type=float, default=20.0)
    parser.add_argument("--progress-every", type=int, default=25)
    parser.add_argument("--no-wait-confirmation", dest="wait_confirmation", action="store_false")
    parser.set_defaults(wait_confirmation=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.file is not None and not args.file.is_file():
        parser.error(f"firmware file does not exist: {args.file}")
    if args.file is not None and args.file.suffix.lower() != ".bin":
        parser.error(f"firmware file must be a .bin file: {args.file}")
    if args.chunk_size < 1:
        parser.error("--chunk-size must be >= 1")
    if args.progress_every < 1:
        parser.error("--progress-every must be >= 1")
    if args.page_size < 1:
        parser.error("--page-size must be >= 1")

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except OtaError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())