# BLE OTA Update Protocol

This documents the BLE OTA protocol used by the gauge firmware and WebUI. It is protocol reference material, not usage documentation for any particular uploader.

## Device States

The device has two BLE-visible states during an update:

| State | Advertised name | Purpose |
| --- | --- | --- |
| Application | `3000GT__` / prefix `3000GT` | Normal firmware. Accepts a reboot request into the OTA loader. |
| OTA loader | `STM_OTA` | ST BLE OTA loader. Accepts the firmware image stream. |

The update flow is:

1. Connect to the running application.
2. Write the OTA reboot request.
3. Wait for the application BLE connection to drop.
4. Scan for the OTA loader advertisement, normally `STM_OTA`.
5. Connect to the OTA loader.
6. Send the application base-address command.
7. Stream the `.bin` firmware bytes.
8. Send the upload-finished command.
9. Wait for the loader confirmation/indication where available.
10. Wait for the loader to reboot back into the application advertisement.

If the board is already advertising as `STM_OTA`, skip steps 1 through 4 and start at the OTA-loader upload phase.

## Application Reboot Request

The running application exposes a reboot request characteristic:

| Item | UUID |
| --- | --- |
| Service | `00000a00-cc7a-482a-984a-7f2ed5b3e58f` |
| Characteristic | `0000fe11-8e22-4541-9d4c-21edae82ed19` |

The reboot request payload is three bytes:

```text
[0x01, start_sector, num_sectors]
```

Fields:

| Byte | Meaning |
| --- | --- |
| `0x01` | Request reboot into BLE OTA loader. |
| `start_sector` | Flash sector index for the application slot. |
| `num_sectors` | Number of 4096-byte sectors required for the selected `.bin`. |

For the current application slot:

```text
application_base_offset = 0x0007000
page_size = 4096
start_sector = floor(0x0007000 / 4096) = 7
num_sectors = ceil(firmware_bin_size / 4096)
```

Example for a `200765` byte firmware image:

```text
num_sectors = ceil(200765 / 4096) = 50
payload = 01 07 32
```

The WebUI writes this request without response. During a successful reboot the peripheral may disconnect before the host stack reports write completion; treat an immediate disconnect or "operation aborted" during this write as likely success if the loader then advertises.

## OTA Loader GATT

The OTA loader exposes the ST OTA service:

| Item | UUID | Observed properties |
| --- | --- | --- |
| OTA service | `0000fe20-cc7a-482a-984a-7f2ed5b3e58f` | service |
| Base/control characteristic | `0000fe22-8e22-4541-9d4c-21edae82ed19` | write-without-response |
| Confirmation characteristic | `0000fe23-8e22-4541-9d4c-21edae82ed19` | indicate |
| Data characteristic | `0000fe24-8e22-4541-9d4c-21edae82ed19` | write-without-response |
| Event characteristic placeholder | `0000fe25-8e22-4541-9d4c-21edae82ed19` | not used by this flow |

On the tested board, the base/control and data characteristics are write-without-response only. A write-with-response to `FE22` fails with "Write Not Permitted".

## OTA Loader Commands

Multi-byte address fields are sent little-endian as a 24-bit offset, not as a full 32-bit address.

The current application base offset is:

```text
0x0007000
```

Encoded as three little-endian bytes:

```text
00 70 00
```

### Start Application Upload

Write this four-byte command to the base/control characteristic `FE22` using write-without-response:

```text
[0x02, addr0, addr1, addr2]
```

For the current application slot:

```text
02 00 70 00
```

### Firmware Data

Write the raw firmware `.bin` bytes to the data characteristic `FE24` using write-without-response.

Although the WebUI configuration contains a larger nominal chunk size, the implemented and verified OTA transfer uses 20-byte chunks:

```text
chunk_size = 20
```

The data stream is exactly the contents of the `.bin` file. Do not wrap chunks in extra headers, lengths, offsets, checksums, or packet numbers.

### Upload Finished

After the final data chunk, write this four-byte command to the base/control characteristic `FE22` using write-without-response:

```text
[0x07, addr0, addr1, addr2]
```

For the current application slot:

```text
07 00 70 00
```

## Confirmation

Enable indications on the confirmation characteristic `FE23` before or during the upload when the BLE stack allows it.

The tested loader returned:

```text
01
```

after the upload-finished command. After confirmation, the loader reboots and the application should reappear as `3000GT__`.

Absence of a confirmation indication should be treated as suspicious but not always fatal; the loader may disconnect or reboot before the host receives the indication. The practical success condition is that the application advertisement returns and the new firmware runs.

## File Format

The OTA payload is the raw application `.bin` image for the application slot. The update flow expects a `.bin` file, not `.elf`, `.hex`, or a container format.

The file size is used only to compute `num_sectors` for the application reboot request and progress reporting. The loader data phase sends the exact file bytes.

## Recovery Case

If an update is interrupted after the reboot request, the board can remain in OTA-loader mode advertising as `STM_OTA`.

Recovery procedure at the protocol level:

1. Scan for `STM_OTA`.
2. Connect directly to the OTA loader.
3. Send start upload command `02 00 70 00`.
4. Stream the `.bin` in 20-byte write-without-response chunks to `FE24`.
5. Send finish command `07 00 70 00`.
6. Wait for confirmation/reboot.

No application reboot request is needed in this state.
