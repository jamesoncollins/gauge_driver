# BLE Command Protocol

This documents the application command protocol carried by the `buttonPress`
BLE characteristic.

OTA is intentionally separate. Firmware update reboot requests still use the
existing `BM_REQ_CHAR` / `FE11` characteristic documented in `ota_protocol.md`.

## GATT

| Item | UUID | Payload |
| --- | --- | --- |
| Service | `00000000-cc7a-482a-984a-7f2ed5b3e58f` | UserButtonService |
| Command characteristic | `00000001-8e22-4541-9d4c-21edae82ed19` | 20-byte write |
| Response/telemetry characteristic | `00000003-8e22-4541-9d4c-21edae82ed19` | 64-byte read/notify |

## Command Frame

All multi-byte fields are little-endian. Version 1 frames are capped at 20 bytes.

```text
byte 0   magic       0x47
byte 1   version     0x01
byte 2   seq         client-chosen sequence number
byte 3   opcode
byte 4   flags       bit0 = ack requested
byte 5   payload_len
byte 6..N payload
next     crc8        over bytes 0 through payload end
rest     padding     ignored when a fixed 20-byte write is used
```

CRC is CRC-8/ATM: polynomial `0x07`, initial value `0x00`, no reflection, no
final xor.

Clients may write exactly the frame length, or write the full 20-byte characteristic value with padding bytes after the CRC. Invalid frames are rejected. There is no legacy one-byte button fallback.

## Opcodes

| Opcode | Name | Payload | Current behavior |
| --- | --- | --- | --- |
| `0x01` | `BUTTON` | `u8 button_id` | Sets the active button command. |
| `0x02` | `SET_MODE` | `u8 mode_id` | Reserved; returns `DENIED`. |
| `0x03` | `SET_BRIGHTNESS` | `u8 percent_0_100` | Reserved; returns `DENIED`. |
| `0x04` | `REQUEST_SNAPSHOT` | none | Reserved; returns `DENIED`. |
| `0x10` | `CONFIG_GET` | `u16 key` | Reserved; returns `DENIED`. |
| `0x11` | `CONFIG_SET` | `u16 key + value` | Reserved; returns `DENIED`. |
| `0x20` | `DIAG_PING` | optional bytes | Echoes payload in the response detail. |
| `0x21` | `DIAG_RESET_STATS` | none | Clears loop diagnostic counters. |
| `0x22` | `DIAG_SQUARE` | `u8 enabled` | Shows the on-screen diagnostic square when nonzero; hides it when zero. |

Button IDs:

```text
0 = OK
1 = UP
2 = DOWN
3 = LEFT
4 = RIGHT
```

## Responses

Responses are sent through the existing `readNext` notification stream as a
reserved `BTBufferData` item:

```text
id1 = 0x434D
id2 = seq
data[0] = status
data[1] = opcode
data[2] = detail_len
data[3..] = optional detail
```

Status codes:

```text
0x00 OK
0x01 BAD_FRAME
0x02 BAD_CRC
0x03 UNSUPPORTED_VERSION
0x04 UNKNOWN_OPCODE
0x05 BAD_LENGTH
0x06 BAD_VALUE
0x07 BUSY
0x08 DENIED
0x09 INTERNAL_ERROR
```

If the command frame has `flags.bit0` clear, successful commands may be silent.
Errors always produce a response when the response buffer has been initialized.
