import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
import time

# Screen parameters
WIDTH = 240
HEIGHT = 255
PIXEL_SIZE = 2  # RGB565: 2 bytes per pixel
FRAME_SIZE = WIDTH * HEIGHT * PIXEL_SIZE

# Serial port configuration
SERIAL_PORT = "COM4"  # <-- Change this to your COM port
BAUDRATE = 921600  # Increased baudrate for faster transfer (requires matching MCU config)

# Open serial port
ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)

# Ensure buffers are clean before starting
ser.reset_input_buffer()
ser.reset_output_buffer()

def read_exact(ser, size):
    """Read exactly `size` bytes from serial, no more, no less."""
    buf = bytearray()
    while len(buf) < size:
        chunk = ser.read(size - len(buf))
        if not chunk:
            raise RuntimeError("Timeout waiting for data")
        buf += chunk
    return buf

def rgb565_to_rgb888_corrected(data):
    """Convert RGB565 bytes to RGB888 numpy array with byte swap correction."""
    swapped = np.frombuffer(data, dtype='>u2').reshape((HEIGHT, WIDTH))

    r = ((swapped >> 11) & 0x1F) << 3
    g = ((swapped >> 5) & 0x3F) << 2
    b = (swapped & 0x1F) << 3

    corrected = np.stack((r, g, b), axis=-1).astype(np.uint8)
    return corrected

# Setup live plot
plt.ion()
fig, ax = plt.subplots()
im = ax.imshow(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8))
plt.title("Live Framebuffer View (Corrected Colors)")
plt.axis("off")

try:
    while True:
        # Flush buffers before requesting frame to avoid misalignment
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Request frame
        ser.write(b'FRAM')

        # Read exactly FRAME_SIZE bytes using larger read size for performance
        frame_data = read_exact(ser, FRAME_SIZE)

        # Convert to RGB888 with byte swap correction and update plot
        rgb = rgb565_to_rgb888_corrected(frame_data)
        im.set_data(rgb)
        plt.pause(0.01)  # Minimal pause for fastest refresh

        # Flush any remaining bytes after plotting
        ser.reset_input_buffer()

except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()
