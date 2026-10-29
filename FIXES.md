# Headless PyBullet Rendering on WSL2 — Fix Summary

Debugging the empty/black video output from the batch navigation-recording pipeline
running under WSL2 + the PyBullet `eglRendererPlugin`.

## Symptom

`record_navigation()` ran to completion without errors and produced a valid, playable
`.mp4` — but the video was either solid black or a flat light-blue-grey tile with no
visible geometry. File size was suspiciously small (~9KB for several seconds), which
is the signature of a static/unchanging frame buffer being h.264-compressed.

## Root causes (two separate bugs)

### 1. EGL platform selection on WSL2

**Problem:** PyBullet's `eglRendererPlugin` initializes EGL via `libEGL`, which probes
platforms in order (GBM → Wayland/surfaceless → X11) until one succeeds. On this WSL2
setup:
- `gbm` and `surfaceless` platforms both failed to initialize (`eglInitialize failed`) —
  WSL2 exposes GPU access through `/dev/dxg`, not the `/dev/dri` render nodes these
  platforms expect.
- Only the `x11` platform succeeded, routing through Xwayland (WSLg's persistent
  background X server) to Mesa's **d3d12 Gallium driver**, which forwards OpenGL calls
  through DirectX 12 to the real GPU (confirmed via `glxinfo -B`:
  `OpenGL renderer string: D3D12 (NVIDIA Quadro P4000)`).

When the script was launched from a context where `DISPLAY` wasn't set (e.g. via the
original PowerShell wrapper script), EGL had no working platform to fall back to and
silently produced blank/garbage framebuffers — no crash, no error, just empty frames.

**Fix:** Force the working platform explicitly, and guarantee `DISPLAY` is set, at the
very top of `trial_worker.py`, before `pybullet` is imported:

```python
import os
os.environ.setdefault("DISPLAY", ":0")
os.environ.setdefault("EGL_PLATFORM", "x11")
```

This works under `p.DIRECT` with **no visible window and no GUI event loop** — Xwayland
under WSLg runs as a persistent background service regardless of whether a terminal or
window is open, so this is still fully headless/scriptable (via cron, CI, detached
SSH, etc.), as long as these two env vars are set before the EGL context is created.

Since `run_batch.py` spawns workers via `ProcessPoolExecutor(mp_context=mp.get_context
('spawn'))`, each worker inherits the parent process's environment — so exporting
`DISPLAY`/`EGL_PLATFORM` in the launching shell is also sufficient, with the `os.environ`
guard in code as a belt-and-braces fallback for less predictable launch contexts.

### 2. Degenerate projection matrix in `recording.py`

**Problem:** `record_navigation()` built its projection matrix like this:

```python
camera = p.getDebugVisualizerCamera(physicsClientId=client)
view = camera[2]

if camera is not None and len(camera) >= 4:
    projection = list(camera[3])
    projection[0] = projection[5] / (width / height)
else:
    projection = p.computeProjectionMatrixFOV(...)   # never reached
```

`p.getDebugVisualizerCamera()` never returns `None` under `p.DIRECT`, so the `if`
branch always ran — the `else` fallback (the only branch that built a known-good
projection matrix) was dead code. The EGL renderer plugin maintains its own separate
camera state (`SimpleCamera m_camera` in `eglRendererVisualShapeConverter.cpp`),
independent of the debug-visualizer camera the code was reading from. That camera's
`projectionMatrix` could be stale or built against mismatched dimensions, and the
`projection[0] = projection[5] / (width / height)` patch would then silently produce
a degenerate matrix (e.g. all zeros) — pushing the entire scene outside the clip volume.
Rendering "succeeded" at the API level (correct buffer shape, no exception) but drew
nothing, producing a flat frame equal to the renderer's clear color (`glClearColor
(.7, .7, .8, 1)`, matching the light-blue-grey tiling observed).

**Fix:** Stop reading from `getDebugVisualizerCamera()` for the projection matrix
entirely. Always build it explicitly:

```python
projection = p.computeProjectionMatrixFOV(
    fov=60.0,
    aspect=float(width) / float(height),
    nearVal=0.1,
    farVal=100.0,
    physicsClientId=client,
)
```

The `view` matrix was unaffected — it's recomputed every step from the robot's
measured base position via `computeViewMatrixFromYawPitchRoll`, so only the
projection needed fixing.

## Secondary note: lighting/wall color (cosmetic, not a bug)

Early in debugging, frames looked like flat light-blue tiles even when rendering was
technically working, because `p.configureDebugVisualizer(lightPosition=..., rgbBackground=...)`
has no effect on the EGL plugin's renderer — it configures the *debug visualizer's*
renderer, a separate code path. The EGL plugin hardcodes its own lighting defaults
(`m_lightAmbientCoeff = 0.6`, overhead-biased) and clear color. If contrast is still
flat after the fixes above, pass `lightDirection=[1,1,3]` and `shadow=1` directly to
each `getCameraImage()` call (these map to the plugin's `setLightDirection`/`setShadow`
and do take effect), and/or darken the maze wall color from `(0.95, 0.95, 0.98, 1.0)`
to something with more value separation, e.g. `(0.75, 0.78, 0.85, 1.0)`.

## Files changed

- `trial_worker.py` — added `DISPLAY`/`EGL_PLATFORM` env guard before `pybullet` import.
- `recording.py` — replaced the degenerate `getDebugVisualizerCamera()`-derived
  projection with an explicit, always-executed `computeProjectionMatrixFOV` call.
