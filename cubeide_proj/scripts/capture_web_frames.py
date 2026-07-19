#!/usr/bin/env python3
"""Capture animated gauge simulator frames from the Emscripten canvas."""

from __future__ import annotations

import argparse
import base64
import contextlib
import http.server
import json
import os
import pathlib
import socket
import socketserver
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import zlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_BUILD_DIR = ROOT / "build" / "web-debug"
DEFAULT_OUTPUT_DIR = DEFAULT_BUILD_DIR / "analysis"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture PNG frames from the web-debug gauge simulator canvas."
    )
    parser.add_argument("--build-dir", type=pathlib.Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--output-dir", type=pathlib.Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument("--interval", type=float, default=0.75)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--chrome", type=pathlib.Path, default=None)
    parser.add_argument("--keep-profile", action="store_true")
    return parser.parse_args()


def find_chrome(explicit: pathlib.Path | None) -> pathlib.Path:
    candidates = []
    if explicit is not None:
        candidates.append(explicit)
    env_chrome = os.environ.get("CHROME") or os.environ.get("CHROME_PATH")
    if env_chrome:
        candidates.append(pathlib.Path(env_chrome))
    candidates.extend(
        [
            pathlib.Path(r"C:\Program Files\Google\Chrome\Application\chrome.exe"),
            pathlib.Path(r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe"),
            pathlib.Path(r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"),
            pathlib.Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"),
        ]
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise SystemExit("Could not find Chrome or Edge. Pass --chrome or set CHROME.")


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt: str, *args: object) -> None:
        pass


class ReusableTcpServer(socketserver.TCPServer):
    allow_reuse_address = True


@contextlib.contextmanager
def serve_directory(directory: pathlib.Path, port: int):
    handler = lambda *args, **kwargs: QuietHandler(  # noqa: E731
        *args, directory=str(directory), **kwargs
    )
    httpd = ReusableTcpServer(("127.0.0.1", port), handler)
    actual_port = httpd.server_address[1]
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{actual_port}/index.html"
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=5)


class CdpClient:
    def __init__(self, wsurl: str):
        hostport, path = wsurl[5:].split("/", 1)
        host, ws_port = hostport.split(":")
        self.sock = socket.create_connection((host, int(ws_port)), timeout=5)
        self.next_id = 1
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET /{path} HTTP/1.1\r\n"
            f"Host: {hostport}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(request.encode("ascii"))
        response = self.sock.recv(4096)
        if b"101" not in response.split(b"\r\n", 1)[0]:
            raise RuntimeError(response.decode(errors="replace"))

    def close(self) -> None:
        self.sock.close()

    def send(self, obj: dict) -> None:
        payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
        header = bytearray([0x81])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header += bytes([0x80 | 126]) + struct.pack(">H", length)
        else:
            header += bytes([0x80 | 127]) + struct.pack(">Q", length)
        mask = os.urandom(4)
        header += mask
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        self.sock.sendall(header + masked)

    def recv_msg(self, timeout: float = 10) -> dict:
        self.sock.settimeout(timeout)
        chunks = []
        while True:
            header = self.sock.recv(2)
            if not header:
                raise EOFError
            first, second = header
            length = second & 127
            if length == 126:
                length = struct.unpack(">H", self.sock.recv(2))[0]
            elif length == 127:
                length = struct.unpack(">Q", self.sock.recv(8))[0]
            mask = self.sock.recv(4) if second & 128 else None
            data = b""
            while len(data) < length:
                data += self.sock.recv(length - len(data))
            if mask:
                data = bytes(byte ^ mask[index % 4] for index, byte in enumerate(data))
            if first & 0x0F == 8:
                raise EOFError
            chunks.append(data)
            if first & 0x80:
                return json.loads(b"".join(chunks).decode("utf-8"))

    def command(self, method: str, params: dict | None = None, timeout: float = 10) -> dict:
        command_id = self.next_id
        self.next_id += 1
        self.send({"id": command_id, "method": method, "params": params or {}})
        while True:
            message = self.recv_msg(timeout)
            if message.get("id") == command_id:
                if "error" in message:
                    raise RuntimeError(f"{method}: {message['error']}")
                return message


def http_json(url: str) -> list[dict] | dict:
    with urllib.request.urlopen(url, timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def open_page(devtools_port: str, url: str) -> str:
    endpoint = f"http://127.0.0.1:{devtools_port}/json/new?{urllib.parse.quote(url, safe=':/?=&')}"
    try:
        urllib.request.urlopen(endpoint, timeout=5).read()
    except urllib.error.HTTPError:
        request = urllib.request.Request(endpoint, method="PUT")
        urllib.request.urlopen(request, timeout=5).read()
    tabs = http_json(f"http://127.0.0.1:{devtools_port}/json")
    for tab in tabs:
        if url in tab.get("url", ""):
            return tab["webSocketDebuggerUrl"]
    raise RuntimeError(f"Could not find opened tab for {url}")


def start_chrome(chrome: pathlib.Path, profile: pathlib.Path) -> tuple[subprocess.Popen, str]:
    port_file = profile / "DevToolsActivePort"
    if port_file.exists():
        port_file.unlink()
    proc = subprocess.Popen(
        [
            str(chrome),
            "--headless=new",
            "--disable-gpu",
            "--no-first-run",
            "--no-default-browser-check",
            "--disable-extensions",
            "--remote-debugging-port=0",
            f"--user-data-dir={profile}",
            "about:blank",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    for _ in range(100):
        if port_file.exists():
            return proc, port_file.read_text(encoding="utf-8").splitlines()[0].strip()
        time.sleep(0.1)
    proc.kill()
    _, err = proc.communicate(timeout=5)
    raise RuntimeError(f"Chrome did not create DevToolsActivePort:\n{err[-2000:]}")


CAPTURE_EXPR = r"""(() => {
  const c = document.getElementById('canvas');
  if (!c) return {error: 'no canvas'};
  const sample = document.createElement('canvas');
  sample.width = c.width;
  sample.height = c.height;
  const ctx = sample.getContext('2d');
  ctx.drawImage(c, 0, 0);
  const d = ctx.getImageData(0, 0, sample.width, sample.height).data;
  let nonBlack = 0, bright = 0, red = 0, yellow = 0, green = 0;
  for (let y = 0; y < c.height; y++) {
    for (let x = 0; x < c.width; x++) {
      const i = (y * c.width + x) * 4;
      const r = d[i], g = d[i + 1], b = d[i + 2];
      if (r || g || b) nonBlack++;
      if (Math.max(r, g, b) > 80) bright++;
      if (r > 180 && g < 80 && b < 80) red++;
      if (r > 180 && g > 120 && b < 80) yellow++;
      if (g > 100 && r < 120 && b < 80) green++;
    }
  }
  return {
    width: c.width,
    height: c.height,
    nonBlack,
    bright,
    red,
    yellow,
    green,
    dataUrl: sample.toDataURL('image/png')
  };
})()"""


def paeth_predictor(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def png_pixel_metrics(png_data: bytes) -> dict:
    if not png_data.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError("capture is not a PNG")
    offset = 8
    width = height = color_type = bit_depth = None
    idat = bytearray()
    while offset < len(png_data):
        length = int.from_bytes(png_data[offset:offset + 4], "big")
        chunk_type = png_data[offset + 4:offset + 8]
        chunk_data = png_data[offset + 8:offset + 8 + length]
        offset += 12 + length
        if chunk_type == b"IHDR":
            width = int.from_bytes(chunk_data[0:4], "big")
            height = int.from_bytes(chunk_data[4:8], "big")
            bit_depth = chunk_data[8]
            color_type = chunk_data[9]
        elif chunk_type == b"IDAT":
            idat.extend(chunk_data)
        elif chunk_type == b"IEND":
            break
    if width is None or height is None or bit_depth != 8 or color_type not in (2, 6):
        raise RuntimeError(f"unsupported PNG format: {width}x{height} depth={bit_depth} color={color_type}")
    channels = 4 if color_type == 6 else 3
    stride = width * channels
    raw = zlib.decompress(bytes(idat))
    rows = []
    source = 0
    prev = bytearray(stride)
    for _ in range(height):
        filter_type = raw[source]
        source += 1
        row = bytearray(raw[source:source + stride])
        source += stride
        for i in range(stride):
            left = row[i - channels] if i >= channels else 0
            up = prev[i]
            up_left = prev[i - channels] if i >= channels else 0
            if filter_type == 1:
                row[i] = (row[i] + left) & 0xFF
            elif filter_type == 2:
                row[i] = (row[i] + up) & 0xFF
            elif filter_type == 3:
                row[i] = (row[i] + ((left + up) // 2)) & 0xFF
            elif filter_type == 4:
                row[i] = (row[i] + paeth_predictor(left, up, up_left)) & 0xFF
            elif filter_type != 0:
                raise RuntimeError(f"unsupported PNG filter: {filter_type}")
        rows.append(row)
        prev = row
    non_black = bright = red = yellow = green = 0
    for row in rows:
        for i in range(0, stride, channels):
            r, g, b = row[i], row[i + 1], row[i + 2]
            if r or g or b:
                non_black += 1
            if max(r, g, b) > 80:
                bright += 1
            if r > 180 and g < 80 and b < 80:
                red += 1
            if r > 180 and g > 120 and b < 80:
                yellow += 1
            if g > 100 and r < 120 and b < 80:
                green += 1
    return {"width": width, "height": height, "nonBlack": non_black, "bright": bright, "red": red, "yellow": yellow, "green": green}


def get_canvas_clip(client: CdpClient) -> dict:
    expr = r"""(() => {
  const c = document.getElementById('canvas');
  if (!c) return {error: 'no canvas'};
  c.style.width = '240px';
  c.style.height = '320px';
  c.style.imageRendering = 'pixelated';
  const r = c.getBoundingClientRect();
  return {x: r.x, y: r.y, width: r.width, height: r.height, canvasWidth: c.width, canvasHeight: c.height};
})()"""
    result = client.command("Runtime.evaluate", {"expression": expr, "returnByValue": True}, timeout=5)
    value = result.get("result", {}).get("result", {}).get("value", {})
    if value.get("error"):
        raise RuntimeError(value["error"])
    return value


def capture_canvas_png(client: CdpClient) -> bytes:
    clip = get_canvas_clip(client)
    screenshot = client.command(
        "Page.captureScreenshot",
        {
            "format": "png",
            "clip": {
                "x": clip["x"],
                "y": clip["y"],
                "width": clip["width"],
                "height": clip["height"],
                "scale": 1,
            },
        },
        timeout=10,
    )
    return base64.b64decode(screenshot["result"]["data"])


def wait_for_canvas(client: CdpClient) -> None:
    deadline = time.time() + 15.0
    last_metrics = {}
    while time.time() < deadline:
        try:
            png_data = capture_canvas_png(client)
            last_metrics = png_pixel_metrics(png_data)
        except RuntimeError as exc:
            if "Execution context was destroyed" not in str(exc) and "no canvas" not in str(exc):
                raise
            time.sleep(0.1)
            continue
        if last_metrics.get("nonBlack", 0) > 0:
            return
        time.sleep(0.1)
    raise RuntimeError(f"Canvas did not render non-black pixels: {last_metrics}")


def capture_frames(client: CdpClient, output_dir: pathlib.Path, count: int, interval: float) -> list[dict]:
    frames = []
    data_urls = []
    for index in range(count):
        if index > 0:
            time.sleep(interval)
        png_data = capture_canvas_png(client)
        metrics = png_pixel_metrics(png_data)
        frame_path = output_dir / f"frame-{index:02d}.png"
        frame_path.write_bytes(png_data)
        metrics.update({"index": index, "file": frame_path.name, "captured_at_s": time.time()})
        frames.append(metrics)
        data_urls.append("data:image/png;base64," + base64.b64encode(png_data).decode("ascii"))
        print(index, json.dumps({k: metrics[k] for k in ("width", "height", "nonBlack", "bright", "red", "yellow", "green")}))
    write_contact_sheet(client, output_dir / "contact-sheet.png", data_urls)
    return frames

def write_contact_sheet(client: CdpClient, output_path: pathlib.Path, data_urls: list[str]) -> None:
    columns = 4
    cell_w = 240
    cell_h = 344
    rows = (len(data_urls) + columns - 1) // columns
    width = columns * cell_w
    height = max(1, rows) * cell_h
    client.command(
        "Emulation.setDeviceMetricsOverride",
        {"width": width, "height": height, "deviceScaleFactor": 1, "mobile": False},
    )
    expr = f"""(() => {{
  const urls = {json.dumps(data_urls)};
  document.body.innerHTML = '';
  document.body.style.margin = '0';
  document.body.style.background = '#111';
  document.body.style.display = 'grid';
  document.body.style.gridTemplateColumns = 'repeat({columns}, 240px)';
  document.body.style.width = '{width}px';
  const style = document.createElement('style');
  style.textContent = '.cell{{width:240px;height:344px;color:#ddd;font:14px sans-serif;background:#111}} img{{width:240px;height:320px;display:block;image-rendering:pixelated}} .label{{height:24px;line-height:24px;padding-left:6px;box-sizing:border-box}}';
  document.head.appendChild(style);
  for (let i = 0; i < urls.length; i++) {{
    const cell = document.createElement('div');
    cell.className = 'cell';
    const img = document.createElement('img');
    img.src = urls[i];
    const label = document.createElement('div');
    label.className = 'label';
    label.textContent = `frame-${{String(i).padStart(2, '0')}}`;
    cell.appendChild(img);
    cell.appendChild(label);
    document.body.appendChild(cell);
  }}
}})()"""
    client.command("Runtime.evaluate", {"expression": expr}, timeout=10)
    time.sleep(0.2)
    screenshot = client.command("Page.captureScreenshot", {"format": "png"}, timeout=10)
    output_path.write_bytes(base64.b64decode(screenshot["result"]["data"]))


def main() -> int:
    args = parse_args()
    build_dir = args.build_dir.resolve()
    output_dir = args.output_dir.resolve()
    if not (build_dir / "index.html").exists():
        raise SystemExit(f"Missing {build_dir / 'index.html'}; build web-debug first.")
    if not (build_dir / "gauge_driver.js").exists():
        raise SystemExit(f"Missing {build_dir / 'gauge_driver.js'}; build web-debug first.")
    output_dir.mkdir(parents=True, exist_ok=True)
    chrome = find_chrome(args.chrome)
    profile_cm = (
        contextlib.nullcontext(output_dir / "chrome-profile")
        if args.keep_profile
        else tempfile.TemporaryDirectory(prefix="gauge-capture-")
    )
    with profile_cm as profile_value:
        profile = pathlib.Path(profile_value)
        profile.mkdir(parents=True, exist_ok=True)
        with serve_directory(build_dir, args.port) as url:
            proc, devtools_port = start_chrome(chrome, profile)
            client = None
            try:
                wsurl = open_page(devtools_port, url)
                client = CdpClient(wsurl)
                client.command("Runtime.enable")
                client.command("Page.enable")
                wait_for_canvas(client)
                time.sleep(args.warmup)
                frames = capture_frames(client, output_dir, args.frames, args.interval)
                (output_dir / "frames.json").write_text(
                    json.dumps({"url": url, "frames": frames}, indent=2),
                    encoding="utf-8",
                )
            finally:
                if client is not None:
                    client.close()
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
    print(f"Wrote capture artifacts to {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
