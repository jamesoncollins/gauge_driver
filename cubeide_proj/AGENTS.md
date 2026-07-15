# Codex Notes

## Gauge Screen Capture

When asked to inspect or analyze gauge screen visuals, prefer the Emscripten
`web-debug` simulator path. The screen is animated, so capture multiple exact
`240x320` canvas PNG frames instead of relying on a single screenshot.

Use `docs/simulator_screen_capture.md` for the full procedure. The stable helper
script is `scripts/capture_web_frames.py`; it writes analysis artifacts under
`build/web-debug/analysis/`.

During visual review, treat fully or mostly blank captured frames as a known PC
rendering artifact from display flushes unless the same blanking is observed on
hardware. The simulator also draws the plastic overlay/mask; the dark lower
area and edge mask are intentional. Do flag content that is clipped by that
overlay aperture, especially near the lower warning-light/gimbal area.

When reporting issues, separate transient shift-indicator coverage from layout
problems. The large shift indicator circle is allowed to cover the screen.
Persistent/constant element overlap is not: check the ECU meters, button text,
gimbal, line plots, and warning indicators against each other across multiple
frames.



