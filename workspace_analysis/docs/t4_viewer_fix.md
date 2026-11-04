# T4 Viewer replay hotfix

The original replay called `t.name.startsWith(...)` for each trace when T4
all-sample constraints were active. Each actual q0/q1 figure contains 41 nameless
traces. The exception occurred before `Plotly.react`, leaving the static first
frame visible. T3's inactive context short-circuited this code path.

`viz/joint_path.py` now normalizes names only when they are strings, validates
target-axis suffixes before indexing, and handles absent optional frame styles
and pair/contact collections. Playback tracks the committed frame, coalesces
pending slider requests, pauses on rendering errors, and replays from frame 0
after the end instead of skipping it. Mesh/TCP/panel updates commit together.
Missing mandatory robot geometry still produces an explicit rendering error.

New regression tests are `tests/test_viewer_replay.py` and its helper
`tests/viewer_js_runtime.py`. They execute the production/exported JavaScript
using the already-installed system JavaScriptCoreGTK through stdlib ctypes.
DOM, Plotly and timers are controlled test doubles; these are dynamic logic tests,
not browser/WebGL screenshots. No dependency was installed.

The old code reproduced the reported TypeError (6 failed / 1 passed in the
initial dynamic regression). The final 8 tests cover missing/null/non-string
names, optional null frame style, every frame, Play/Pause/end restart, pending
render/slider races, error recovery, exported HTML and T3 compatibility.
Full regression: **641 passed, 1 warning, 0 skipped**.

New pages are in `results/task_approach/viewer_fix_20261104_02/`:

- `q0_approach.html`: all 26 frames checked through slider and playback.
- `q1_approach.html`: all 26 frames checked through slider and playback.
- `t3_q0_linear.html`: all 171 frames checked as a compatibility control.
- `t3_q1_linear.html`: all 344 frames checked as a compatibility control.
- `playback_validation.json`: per-frame checks and data identity evidence.
- `manifest.json`: original HTML and current renderer hashes.

Pages are re-exported from existing mesh/native-state payloads. No FK, IK,
collision policy, planning result, JSON/NPZ trajectory or historical HTML is
changed. All original payload fields are identical; legacy T3 pages gain only
an optional null replay-context field. No planning/FCL query is made by export.
The interrupted preliminary `viewer_fix_20261104_01` directory is retained;
use `_02` for review.

Local-file browser automation is blocked by policy, so actual browser/WebGL
acceptance of these newly exported pages remains a user check. The user already
confirmed playback after applying a temporary name guard in the browser.

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest \
  -q -rs -p no:cacheprovider tests/test_viewer_replay.py
```

Git audit: current local `feat/workspace-planning` HEAD is
`02172cb25cd27113c1bafc132fc4dae83d6ca0d1` (T1/T2/T3). T4 implementation and
this hotfix are still uncommitted. The local origin-tracking reference has the
same commit; no fetch, commit or push was performed during this fix.
