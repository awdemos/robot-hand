# Tendon-Driven Robot Hand for Replicanta

A biomechanical tendon-pulley hand with two faces:

1. **Engineering lab page** — `index.html` is a self-contained Three.js kinematic study (no build step, vendored Three.js). It shows 38 colored cords, 19 joints, open-frame stainless links with in-channel tendon routing, sheaves, and a library of 05 movement studies: fist, ripple, pinch, hand signs, every actuator sweep.
2. **Live physics bridge** — `bridge/server.py` runs the tendon simulation as an HTTP/SSE server. Replicanta organisms connect to it and drive the hand by their own volition.

## Files

- `index.html` — standalone kinematic study viewer (open in any static server)
- `bridge/server.py` — stdlib-only HTTP/SSE physics bridge
- `bridge/viewer.js` — older live-SSE 3D hand viewer (also served by the bridge)
- `bridge/vendor/three.min.js` — vendored Three.js (r152)
- `../replicanta/src/replicanta/tendon_hand.py` — `ArmService` exposed to Lua modules
- `../replicanta/modules/tendon-hand/` — Lua module providing `/hand` commands and volitional hooks

## Run the physics bridge

```bash
cd bridge
python3 server.py --port 8765
```

The bridge now hosts the new lab page at the root URL.

Open http://127.0.0.1:8765/ in a WebGL-capable browser.

## Movement library (lab page)

Click a study in the right panel, or deep-link:

- http://127.0.0.1:8765/?study=1   # Make a fist
- http://127.0.0.1:8765/?study=2   # Ripple
- http://127.0.0.1:8765/?study=3   # Pinch
- http://127.0.0.1:8765/?study=4   # Hand signs
- http://127.0.0.1:8765/?study=5   # Every actuator
- http://127.0.0.1:8765/?study=1&t=3.5  # Fist at 3.5 seconds

Keyboard: space = pause, ←/→ = previous/next study, R = restart.

## Hand placement (lab page)

The hand ships palm-up (a 180° roll from the old palm-down pose). The whole
assembly sits under a mount transform, so wrist/forearm animation is
unaffected. The **Hand placement** panel in the right sidebar exposes it to
the user: Position X/Y/Z sliders (scene units) and Yaw/Pitch/Roll sliders
(degrees). Changes apply live, persist in `localStorage`
(`handPlacement.v1`), and **Reset placement** restores the palm-up default.
For console scripting, `window.__hand` exposes `mount`, `root`, `digits`,
`tendons`, `placement` and `applyPlacement()`.

## Live bridge mode (lab page)

When the page is served by the bridge it subscribes to `/events` (SSE) and
renders the **real physics state** instead of the canned studies — so
entity-driven goals (`hand: fist`, `/hand goal wave`, volition) are visible
the moment they happen. The top-left corner shows `LIVE · <goal>` while the
stream is fresh; a few seconds after the stream goes quiet the page falls
back to study playback. The **Live bridge** checkbox in "View & visibility"
switches live rendering off to get back to the studies while the bridge is
up. Joint readouts and EMG bars reflect the live state too.

## Replicanta integration

Replicanta's default voice model is `ternary-bonsai-1.7b-f16` (Ternary-Bonsai-1.7B, F16 GGUF imported into the local Ollama from `../models/prism-ml/`). Override anytime with `OLLAMA_MODEL`.

**The organism can move the hand deliberately.** Its prompt advertises the
hand and the directive syntax; whenever the organism writes a line like
`hand: wave` (optionally with a duration, `hand: fist 3`) in one of its own
utterances, the tendon-hand Lua module's utterance hook executes it against
the bridge (`/goal` or `/posture`). Moves: reach, grasp, release, point,
wave, fist, ripple, pinch, ok, shaka, rock, spock, open. Lifecycle hooks are
real registrations too: the hand reaches on birth, waves on wake, releases
on sleep.

The `tendon-hand` module is enabled by default. It registers `/hand` commands:

- `/hand` — show live bridge state
- `/hand posture <open|fist|pinch|ok|point|shaka|rock|spock|ripple|reach|grasp|release|wave> [dur]`
- `/hand actuator <finger> <joint> <flexor|extensor> <activation 0..1> [dur]`
- `/hand goal <reach|grasp|release|point|wave|fist|ripple|pinch|ok|shaka|rock|spock> [dur]`
- `/hand volition [on|off]` — toggle autonomous control
- `/hand emotion <stress> <arousal> [mood]` — inject emotion

A background `ArmService` thread listens to bridge telemetry and, when volition is enabled, chooses goals from the organism's mood/stress/arousal/chaos.

## Bridge API

- `GET /` — lab page
- `GET /healthz`
- `GET /arm` — current hand state JSON
- `GET /postures` — list of posture names
- `POST /posture` — `{"name": "fist", "duration_s": 4}`
- `POST /actuator` — `{"finger": "middle", "joint": "mcp", "side": "flexor", "activation": 0.8, "duration_s": 4}`
- `POST /goal` — `{"kind": "reach", "duration_s": 4}`
- `POST /pose` — explicit per-finger/wrist/forearm spec
- `POST /emotion` — `{"stress", "arousal", "mood"}`
- `GET /events` — SSE stream (`type: arm` messages)

## Quick test

```bash
# bridge
python3 bridge/server.py --port 8765

# Replicanta (run from this directory)
cd ../replicanta
uv run replicanta --org default
# TUI:
/hand
/hand posture fist 4
/hand actuator middle mcp flexor 0.9 3
```

Or run Replicanta from the hand directory using `--dir`:

```bash
cd /var/home/a/code/robot-hand
uv run --project /var/home/a/code/replicanta replicanta --dir /var/home/a/code/replicanta --org default
```

Or launch the Glasshouse web UI on a separate port:

```bash
cd /var/home/a/code/robot-hand
uv run --project /var/home/a/code/replicanta replicanta --dir /var/home/a/code/replicanta --org default --web --port 9999 --no-browser
```

## Add a 6th study to the lab page

Open `index.html` and edit two places:

1. The `STUDIES` array (around the top of the script) — add a new entry with a name, description, notes, and a `keyframes()` function that returns angle curves for the 22 joints over time.
2. The right-panel movement list in HTML — add a new `<button data-study="5">` (zero-based index).

Copy the structure of an existing study; each keyframe is a function `t -> angle_deg` for every named joint.

## Notes

- The bridge defaults to `127.0.0.1:8765`. Change with `--host`/`--port`.
- The lab page loads Three.js from a CDN; the bridge also serves a vendored fallback if you want to make it fully offline.
- Headless browsers used for CI screenshots often have no WebGL/2D context; verify on a real desktop browser.
