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

## Routine Validation

After shared application changes, build the simulator and hardware debug
presets when feasible:

- `cmake --build --preset x86-debug`
- `cmake --build --preset web-debug`
- `cmake --build --preset st7789vi-debug`
- `cmake --build --preset s6e63d6-debug`

For gauge layout or rendering changes, capture `web-debug` frames with:

```sh
python scripts/capture_web_frames.py --frames 12 --interval 0.75
```

Then inspect `build/web-debug/analysis/contact-sheet.png` for persistent layout
overlap or clipping.


## Windows Sandbox Command Notes

The Windows ACL sandbox can fail before a command starts with:

```text
windows sandbox: helper_unknown_error: apply deny-read ACLs
```

Known-good first attempts:

- Run simple commands one at a time, especially `git status --short`,
  `git diff`, and `cmake --build --preset x86-debug`.
- Prefer direct approved command forms before wrapping them in PowerShell.

Avoid as a first attempt:

- Parallel shell calls for filesystem-heavy reads.
- Complex PowerShell pipelines, variables, globs, or nested quoting.

If a needed command fails with the ACL error, rerun the simplest equivalent
command outside the sandbox with escalation instead of spending time trying
several quoting variants.
