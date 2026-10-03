# One-command actual-state result export

After a local Anchor job or continuous rollout, run from the repository root:

```bash
python scripts/export_mounted_result.py --run-dir /path/to/run
```

This writes `RESULT.md` and `result_export.json` in that run directory, and invokes the existing `render_mounted_actual.py` for each recorded replay/core stage. The renderer needs numpy, MuJoCo, OpenCV, trimesh and scipy; select its interpreter with `--render-python /path/to/python` if needed. No Isaac/GPU allocation is requested. On a headless host configure the existing MuJoCo rendering backend in the environment as usual.

For an initialization failure, queued/incomplete run, or a machine without rendering dependencies:

```bash
python scripts/export_mounted_result.py --run-dir /path/to/run --report-only
```

The report preserves recorded failures and missing evidence. It never labels initialization failure as interaction failure, treats local Anchor success separately from a complete trajectory, and never creates a video without actual trace and measured mesh data. A missing output directory is an input error; use the actual existing directory after job startup.

Videos reuse measured mesh transforms, without advancing physics or substituting reference animation. Playback uses the native source fps and stride 1; irregular trace sampling is rejected instead of stretched. A fractional failed final interval may require one frame of video quantization; exact measured timing stays in the report. Rendering is not proof of physical/task success. Existing source artifacts are not modified.

An export may exit zero while accurately reporting a failed physical experiment. A renderer/read error exits nonzero after saving the report. MPC candidate trials are excluded from the summary; padded Replay and exported-core replay are shown as separate gates. The startup marker describes the last recorded stage, not live queue state.
