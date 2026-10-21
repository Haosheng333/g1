# g1_voice_control

An independent voice-control frontend to the orchestrator. It matches a
small allowlist of spoken commands, requires a separate confirmation before
starting a task, and calls the **same** `RunRoute` services terminal control
already uses (`rosservice call /orchestrator/run_grasp ...`). It does not
replace terminal control and does not touch the robot/hand SDK. The
`system_bringup` full-system launch exposes it only as an explicit opt-in and
keeps it disabled by default.

Built and offline-tested on a machine with **no ROS, no Vosk, no
microphone, and no real robot**. Everything marked "untested" below needs
verification on the actual workspace before being relied on.

## Orchestrator service mappings

The mappings below were checked against the G1 repository's current
`orchestrator/srv/RunRoute.srv` and service registration. Re-check them after
future orchestrator interface changes before setting `dry_run_backend:=false`.

| Voice command | Orchestrator service | Notes |
|---|---|---|
| "开始抓取" / "start grasping" | `/orchestrator/run_grasp` | Currently reports the orchestrator's unresolved perception-extrinsics blocker verbatim |
| "开始递送" / "start delivery" | `/orchestrator/run_deliver` | |
| "查看状态" / "status" | `/orchestrator/status` (topic, latched `std_msgs/String`) | A **read**, not a service call; does not touch the confirm/dispatch state machine |
| "取消任务" / "cancel task" | **none exists** | See "Why cancel is unavailable" below |
| "确认执行" / "confirm" | (local only) | Confirms a pending grasp/deliver; no orchestrator call by itself |

`RunRoute.srv` request fields used by the current adapter: `route` (set to the route's own
name as a same-name guard), `target`, `destination` (explicitly `""` because
voice control calls the separate routes, not `deliver_task`), `start_step`
(always `""` here -- voice control never resumes a route), `dry_run`.
`event_token` is left `""`;
it is for the handover monitor's own release request, which this package
never sends (see "Release ownership" in `orchestrator/README.md` -- voice
control starts tasks, it does not open the hand).

**`target`'s real semantics** (from `orchestrator/src/orchestrator/sequencer.py`'s
`RouteContext`/`DryRunBackend`): a single string, used as *both* the
navigation waypoint name and the perception object label for `grasp`, and as
the delivery waypoint name for `deliver`. That double use is the orchestrator's
existing wiring, not a simplification made in this package. `side` (which
hand) is **not** a request field at all -- it is server-side config
(`orchestrator/config/orchestrator.yaml`'s `side`), so no voice command needs
to carry it.

### Why "cancel" is unavailable

The orchestrator's interface table and `sequencer.Backend` expose no
cancellation action of any kind. What this package can still do safely and
locally: drop an **unconfirmed** pending grasp/deliver before it is ever
sent (nothing has executed, so nothing needs undoing). It cannot stop a route
already in flight -- `rosservice call` could not do that either today. Saying
"取消任务" while a route is executing gets an explicit "no cancellation
service exists" response, never a false "cancelled".

### Why "开始抓取" reports NOT_READY

The current orchestrator implements `/arm/convert_observation`, including its
declared torso-frame convention offset. The remaining static blocker is the
unverified physical camera extrinsics/datum on the deployed robot. Until that
measurement is cleared, `run_grasp` refuses before motion. This package reports
`result.blockers` exactly as received; it does not bypass or reinterpret them.

## Package layout

```
voice_control/                          (catkin package: g1_voice_control)
├── package.xml, CMakeLists.txt, setup.py
├── requirements.txt                    # vosk, sounddevice, pyyaml -- imported lazily
├── config/
│   ├── commands.yaml                   # phrases, validated targets, confirmation window
│   └── vosk.yaml                       # explicit language, model path, sample rate
├── launch/voice_control.launch         # standalone or optional system_bringup include
├── src/g1_voice_control/
│   ├── commands.py                     # phrase normalisation + matching + negation -- no ROS/Vosk
│   ├── configuration.py                # validated YAML command/target loading -- no ROS/Vosk
│   ├── dispatch.py                     # confirm/expire/busy/cancel state machine -- no ROS/Vosk
│   ├── asr_adapter.py                  # Vosk mic/WAV adapters -- lazy imports
│   └── orchestrator_client.py          # real RunRoute service calls + status topic reader
├── scripts/
│   ├── voice_control_node.py           # the ROS node
│   ├── list_microphones.py             # enumerate input devices
│   ├── recognize_wav.py                # WAV -> transcripts -> command match, no ROS
│   └── download_model.sh               # explicit model download (cn|en); never automatic
├── test/
│   ├── test_commands.py                # offline
│   ├── test_dispatch.py                # offline
│   ├── test_orchestrator_client.py     # offline (rospy/orchestrator.srv faked in sys.modules)
│   └── test_ros_transport.py (+ .test) # REAL ROS master needed; see below
└── models/                             # gitignored; populated by download_model.sh
```

## Setup

Run setup inside the repository's designated ROS Noetic container, whose
Python environment is already selected by `docker/g1.sh enter`:

```bash
cd ~/Workspace/g1_intellect
docker/g1.sh enter 'cd ~/Workspace/DIP && python3 -m pip install -r voice_control/requirements.txt'
docker/g1.sh enter 'cd ~/Workspace/DIP && voice_control/scripts/download_model.sh cn'  # or: en
docker/g1.sh enter 'cd ~/Workspace/DIP/g1_ws && PATH="$CONDA_PREFIX/bin:$PATH" catkin_make -j8 -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release'
```

The tracked symlink `g1_ws/src/g1_voice_control -> ../../voice_control` makes
the package visible to catkin. Do not install these dependencies into the
host's ROS2/Python environment.

`sounddevice` needs PortAudio; if the pip install or the import fails, install
the system package first (`apt install portaudio19-dev` on Debian/Ubuntu).
Verify the target with `uname -m`, `python3 --version`, and
`python3 -m pip install --dry-run -r requirements.txt` before installation.
Do not upgrade the ROS system Python to satisfy a wheel.

## Running it

```bash
# enumerate input devices
rosrun g1_voice_control list_microphones.py

# WAV-file recognition, no ROS, no live mic
rosrun g1_voice_control recognize_wav.py --model "$(rospack find g1_voice_control)/models/vosk-model-small-cn-0.22" --wav sample.wav

# live microphone, matched/confirmed but NO orchestrator calls (default)
roslaunch g1_voice_control voice_control.launch input_device:=2

# live microphone, real dispatch enabled
roslaunch g1_voice_control voice_control.launch input_device:=2 dry_run_backend:=false

# optional full-system integration: still dry-run, and disabled unless requested
roslaunch system_bringup full_system.launch voice_control:=true voice_input_device:=2
```

Diagnostics: `rostopic echo /rosout` while the node runs shows every heard
transcript, match/no-match reason, and dispatch event
(`voice_control: heard ... -> matched/rejected/awaiting_confirmation/dispatched/...`).

### Ubuntu VM/container microphone routing

Do not assume the macOS microphone is visible inside Ubuntu. Check each layer:

```bash
# Ubuntu host/VM
arecord -l
pactl list short sources

# Inside the runtime/container
ls -l /dev/snd
python3 -m sounddevice
rosrun g1_voice_control list_microphones.py
```

For a Linux Docker host, pass the audio device explicitly (for example
`--device /dev/snd` and the host's audio group) and verify permissions inside
the container. Docker Desktop on macOS does not expose the Mac microphone as
a normal `/dev/snd` device; configure an audio bridge deliberately or record a
16 kHz, 16-bit, mono PCM WAV on the host and use `input_mode:=wav`. Never infer
microphone availability merely because macOS granted the terminal permission.

## Supported commands and configuration

| Phrase (zh / en) | Command id | Needs "确认执行"/"confirm"? |
|---|---|---|
| 开始抓取 / start grasping | `grasp` | yes |
| 开始递送 / start delivery | `deliver` | yes |
| 查看状态 / status | `status` | no (read-only) |
| 取消任务 / cancel task | `cancel` | no (local-only; see above) |
| 确认执行 / confirm | `confirm` | -- |

`config/commands.yaml` contains the complete phrase allowlist and route
targets. Shipped targets are deliberately `null`. Before enabling a route,
put an exact value verified from the deployed orchestrator/perception or
navigation configuration in `valid_grasp_targets` or
`valid_deliver_targets`, then select that same value in `grasp_target` or
`deliver_target`. A selected value outside its allowlist is a startup error.
`confirmation_window_sec` defaults to 8 seconds and must be positive.

`config/vosk.yaml` explicitly selects `language`, `model_path`, and
`sample_rate`. Pick **one** language's model (`vosk-model-small-cn-0.22` default, or
`vosk-model-small-en-us-0.15`); a single Vosk model is never bilingual. The
phrase table above already covers both languages for the matcher itself --
only the ASR model needs picking per deployment. Launch arguments
`language`, `vosk_model_path`, and `sample_rate` override the YAML only when
non-empty.

`~dry_run_backend` (launch arg, default `true`): when true, commands are
heard, matched and confirmed, but `orchestrator_client` is never constructed
and no service is ever called -- mirrors the orchestrator's own
`DryRunBackend` pattern (dry-run safety as a property of which object got
injected, not a flag re-checked per call site).

`~service_wait_timeout_s` only bounds waiting for a service to appear. It is
not a route-execution deadline: ROS1 synchronous service calls do not provide
a safe way to cancel an already-started request. An in-flight call stays BUSY,
is never retried automatically, and status/cancel commands remain responsive.

## Behaviour notes

- **Final results only.** Partial Vosk results never reach `commands.py`.
- **No substring matching.** An utterance must equal an allowlisted phrase
  after normalisation. Spaces between adjacent CJK characters are removed to
  accept Vosk tokenisation such as `开始 抓取`; English spaces remain significant. "不要开始抓取"
  cannot match "开始抓取" by containment, and is additionally rejected by an
  explicit negation-marker check (`src/g1_voice_control/commands.py:NEGATION_MARKERS`).
- **One pending command, not a queue.** A second task-start utterance before
  confirmation *replaces* the pending one; nothing has executed yet, so there
  is nothing to protect by keeping the stale one.
- **Busy means no new task starts.** Confirmation first creates a single-use
  dispatch action and marks the state BUSY; only the blocking ROS call runs on
  a worker. While it is in flight, a new grasp/deliver is rejected immediately.
  `status` and the supported local cancellation behavior remain responsive.
- **No auto-retry.** A failed or timed-out `RunRoute` call is reported once;
  `dispatch.py` never calls it again on its own, since the route may already
  have started moving the arm.
- **Nothing replays.** State is in-memory only. Microphone audio uses a bounded
  five-block queue with a one-second maximum age; old blocks are discarded.
  PortAudio input faults clear queued audio and any pending confirmation.

## Tests

```bash
# offline -- no ROS master, run directly or via catkin:
cd voice_control && PYTHONPATH=src python3 -m unittest discover -s test -v
# or: catkin run_tests g1_voice_control

# real ROS transport -- needs a roscore; UNTESTED here, verify on the real workspace:
rostest g1_voice_control test_ros_transport.test
```

**Current local result: 50 passing and one ROS-only test skipped:**
- `test_commands.py`: every supported phrase in both languages, including
  Vosk-style spaces between Chinese words, case/whitespace/full-width
  normalisation, negation rejection, substring
  non-match, ambiguous-phrase construction failure, empty transcript.
- `test_dispatch.py`: confirm-before-dispatch, active confirmation expiry,
  audio-fault reset, correct request-field
  mapping, confirm-with-nothing-pending, expiry just-after and
  just-inside the window, replace-not-stack on a second task-start,
  duplicate-utterance no-double-dispatch, busy rejection (task-start and
  confirm), missing-target refusal, cancel (pending/none-pending/busy),
  failed-call and raising-client both reported once with no retry,
  dry-run client makes no real calls.
- `test_asr_adapter.py`: partial results are not forwarded; final results are
  forwarded once; bounded audio drops oldest/stale blocks and clears cleanly,
  without importing Vosk or opening a microphone.
- `test_configuration.py`: shipped targets are disabled and phrases/model
  selection are loaded from YAML.
- `test_package_integrity.py`: launch XML parses and the rostest node is executable.
- `test_orchestrator_client.py`: `grasp`/`deliver` map to the declared
  service names and request fields, blockers/failure fields pass through
  unchanged, an unmapped command raises rather than guessing a service name,
  service proxies are created lazily.

**Written but blocked here (need real ROS/hardware to actually run):**
- `test_ros_transport.py` -- needs a roscore; verify with `rostest`.
- A real Vosk ASR smoke test over actual recorded/live audio -- needs the
  downloaded model and either a WAV sample or a microphone; `recognize_wav.py`
  is ready for this once a sample exists, but no sample or model was
  available in this environment. **Injected text is not a substitute** for
  this and is not claimed as one anywhere above.
- `voice_control_node.py` has only been checked with `ast.parse` (syntax),
  never run against a live `roscore`/microphone/model.
- Microphone routing for the real Ubuntu VM/container this runs in
  (per `CLAUDE.md`, the system runs inside a Docker container) was not
  verified -- `list_microphones.py` existing is necessary but not sufficient;
  confirm the container actually has an audio device passed through before
  assuming the Mac's (or the operator's) microphone is reachable there.

## Acceptance procedure (dispatch disabled)

First copy `config/commands.yaml`, enter a target that you have verified from
the deployed system in the matching `valid_*` list and selected target field,
and pass the copy through `commands_config:=/absolute/path/to/commands.yaml`.
Keep `dry_run_backend:=true` (the default), then start `roscore` and this node:

1. Say "开始抓取". Expect `awaiting_confirmation` with the verified target.
2. Say unrelated background speech for the confirmation window. Expect no
   dispatch and, after the window, the pending command silently expiring
   (next "确认执行" logs `ignored: nothing pending to confirm`).
3. Say "开始抓取" again, then "确认执行" within the window. Expect
   `dispatched` with `RecordingDryRunClient`'s message
   (`dry run: would call route for 'grasp' ...`). No task-execution service
   call should appear in `rosservice` diagnostics.
4. Say "不要开始抓取". Expect no match/no dispatch (negation rejected).
5. Say "取消任务" with nothing pending. Expect `cancelled: nothing was pending`.
