import serial
import serial.tools.list_ports
import subprocess
import time
import sys
import os

from pathlib import Path

# STM32 USB IDs
STM32_VID = 0x0483
CDC_PIDS = [0x5740]      # Virtual COM Port PID
DFU_PIDS = [0xDF11]      # DFU Bootloader PID

# Paths relative to this script
SCRIPT_DIR = Path(__file__).parent.resolve()
ELF_FILE = SCRIPT_DIR.parent / "Debug" / "stm32ide_proj.elf"
BIN_FILE = SCRIPT_DIR / "stm32ide_proj.bin"

# STM32CubeProgrammer CLI tool
CUBEPROG_PATH = "STM32_Programmer_CLI"  # Must be in PATH or provide full path

def find_device(vid, pid_list):
    """
    Search for a USB device with specified VID and any of the PIDs.
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == vid and port.pid in pid_list:
            return port
    return None

def trigger_dfu(port_name):
    """
    Open CDC COM port and send DFU trigger.
    """
    try:
        print(f"Opening {port_name}...")
        ser = serial.Serial(port_name, 115200, timeout=1)
        time.sleep(0.1)  # Give USB CDC stack time to settle
        print("Sending DFU trigger...")
        ser.write(b'DFU\n')
        ser.flush()
        ser.close()
    except Exception as e:
        print(f"Error sending DFU trigger: {e}")
        sys.exit(1)

def wait_for_dfu(timeout=10):
    """
    Wait for STM32 DFU bootloader to enumerate.
    """
    print("Waiting for DFU device...")
    start = time.time()
    while time.time() - start < timeout:
        dfu_device = find_device(STM32_VID, DFU_PIDS)
        if dfu_device:
            print(f"✅ DFU device detected: {dfu_device.device}")
            return True
        time.sleep(0.25)
    print("❌ Timed out waiting for DFU device.")
    return False

def convert_elf_to_bin():
    """
    Convert ELF file to BIN using CubeProgrammer CLI or arm-none-eabi-objcopy.
    """
    if not ELF_FILE.exists():
        print(f"❌ ELF file not found: {ELF_FILE}")
        sys.exit(1)

    print(f"Converting ELF to BIN: {ELF_FILE.name} -> {BIN_FILE.name}")
    try:
        # Use STM32_Programmer_CLI to convert
        subprocess.run([
            CUBEPROG_PATH,
            "-ms", str(ELF_FILE),
            str(BIN_FILE)
        ], check=True)
        print("✅ Conversion complete.")
    except FileNotFoundError:
        print(f"❌ {CUBEPROG_PATH} not found. Is STM32CubeProgrammer installed?")
        sys.exit(1)
    except subprocess.CalledProcessError:
        print("❌ Failed to convert ELF to BIN.")
        sys.exit(1)

def flash_firmware():
    """
    Call STM32CubeProgrammer CLI to flash firmware.
    """
    if not BIN_FILE.exists():
        print(f"❌ Binary file not found: {BIN_FILE}")
        sys.exit(1)

    print(f"Flashing {BIN_FILE.name} using CubeProgrammer...")
    cmd = [
        CUBEPROG_PATH,
        "-c", "port=USB1",  # Adjust if you have multiple USB DFU devices
        "-w", str(BIN_FILE),
        "-v",  # Verify after write
        "-rst" # Reset MCU after flashing
    ]

    try:
        subprocess.run(cmd, check=True)
        print("✅ Firmware flashed successfully.")
    except subprocess.CalledProcessError as e:
        print("❌ Firmware flashing failed.")
        sys.exit(1)

def main():
    print("🔎 Searching for STM32 CDC device...")
    cdc_device = find_device(STM32_VID, CDC_PIDS)
    if not cdc_device:
        print("❌ STM32 CDC device not found. Is the board connected and running your firmware?")
        sys.exit(1)

    print(f"✅ Found CDC device: {cdc_device.device}")

    trigger_dfu(cdc_device.device)

    print("⏳ Waiting for STM32 to switch into DFU mode...")
    if not wait_for_dfu():
        print("❌ DFU device did not appear. Check your firmware and USB connection.")
        sys.exit(1)

    convert_elf_to_bin()
    flash_firmware()

if __name__ == "__main__":
    main()
