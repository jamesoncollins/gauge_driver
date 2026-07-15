# Simulator Screen Capture

Use this procedure when visual inspection of the gauge screen is needed. The
Emscripten path is the preferred route because it exposes the display as a fixed
`240x320` browser canvas that can be captured exactly.

## Preferred Emscripten Workflow

Build the web simulator:

```sh
./scripts/build-preset.sh web-debug
```

Then capture frames:

```sh
python scripts/capture_web_frames.py
```

The helper serves `build/web-debug` locally, opens `index.html` in headless
Chrome through the Chrome DevTools Protocol, waits for the simulator canvas to
render, and captures the canvas directly with `canvas.toDataURL("image/png")`.

Default capture settings:

- Warmup: 2 seconds.
- Frame count: 12.
- Frame interval: 750 ms.
- Canvas size: `240x320`.

Outputs are written to `build/web-debug/analysis/`:

- `frame-00.png` through `frame-11.png`
- `contact-sheet.png`
- `frames.json`

`frames.json` includes per-frame timestamps, canvas dimensions, and simple pixel
metrics such as non-black, bright, red, yellow, and green pixel counts. Use these
metrics to detect blank captures or unexpectedly static output before detailed
visual inspection.

Useful options:

```sh
python scripts/capture_web_frames.py --frames 40 --interval 0.75
python scripts/capture_web_frames.py --warmup 2 --port 8765
python scripts/capture_web_frames.py --chrome "C:\Program Files\Google\Chrome\Application\chrome.exe"
```

For full synthetic drive-cycle coverage, capture at least 28 seconds of frames.
For routine layout checks, the default 12 frames over roughly 9 seconds is
usually enough.

## x86 Fallback

If the Emscripten path is unavailable, build and run the native simulator:

```sh
./scripts/build-preset.sh x86-debug
build/x86-debug/gauge_driver.exe
```

Capture the uGFX Win32 window client area at the same cadence as the web path:
warm up, then capture multiple frames. Crop out window chrome and normalize each
image to `240x320` before comparing with web captures.

Use the x86 path mainly to confirm that the Win32 backend matches the browser
backend. For automated analysis, prefer the web canvas capture because it avoids
native-window cropping and scaling issues.
