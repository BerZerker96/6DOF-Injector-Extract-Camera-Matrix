# 6DOF Injector + Probe

A standalone tool that finds a game's camera and prints a complete, copy-pasteable spec for
building an OpenTrack 6DOF head-tracking mod. No game-specific setup.

## Use
1. Run `6DOFInjectGUI.exe` (or the console `6DOFInject.exe`). Match the architecture to the game
   (use the 32-bit build for 32-bit games).
2. Pick the running game from the list and inject. The probe DLL loads into it.
3. Play for ~5 seconds. The probe auto-runs its full discovery pipeline and writes
   `6DOFProbe.log` next to the probe (and to the game folder).
4. Read the log. The two blocks you want are **MOD BUILD SPEC** (the GPU camera picture) and
   **TURNKEY MOD SPEC** (the consolidated build sheet). Send me that log and I build the mod.

## What the probe discovers
- **API / arch / engine / file version / resolution / screen aspect.**
- **GPU camera** (D3D9/10/11/12 + OpenGL): the view / projection / view-projection constant
  buffers, draw-weighted so the MAIN-scene camera stands out from per-object matrices, with
  layout (row/col major), handedness, translation slot, FOV (vertical/horizontal, near/far,
  reversed-Z), and an FOV-override factor.
- **CPU camera source**: correlates the GPU matrix back to a writable address, finds static
  pointer chains to it, and hardware-breakpoints it to capture the exact write instruction
  (decoded to **base register + field offset + code-cave steal length**).
- **Representation classification**: matrix 4x4 (world or view), quaternion, Euler-degrees,
  or eye/target look-at rig - with each field's offset flagged in a struct dump.
- **Spin-test**: briefly sweeps the located camera so you can SEE whether it drives the view
  (the upstream-vs-final-stage check - if the view doesn't move, the struct is derived and the
  mod must hook the final view setter or the GPU buffer instead).
- **TURNKEY MOD SPEC**: one block - representation, locator, layout/math, FOV, recommended INI
  params, and the drive decision - everything needed to build the mod.

## Keys (while injected)
- `INSERT` - re-run the full discovery pipeline
- `END` - write a report now
- `HOME` - scan memory for a CPU camera
- `F7` - snapshot all camera-shaped values; then rotate the in-game view ~45 deg
- `F8` - delta scan: whatever changed is the live camera (find-by-motion; works even when the
  camera never touches a GPU buffer)

## Notes
- The probe is read-only except for the optional spin-test (which restores the matrix after).
- Vulkan present is detected; its matrix capture add-on is pending, but the CPU memory and
  motion-difference routes (HOME / F7+F8) still locate the camera.
- This tool reports data only; it ships no game-specific addresses.
