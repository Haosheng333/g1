# g1_voice_control

An independent voice-control frontend to the orchestrator. It matches a
small allowlist of spoken commands, requires a separate confirmation before
starting a task, and calls the **same** `RunRoute` services terminal control
already uses (`rosservice call /orchestrator/run_grasp ...`). It does not
replace terminal control, does not touch the robot/hand SDK, and is not part
of `system_bringup`'s default full-system launch.

Built and offline-tested on a machine with **no ROS, no Vosk, no
microphone, and no real robot**. Everything marked "untested" below needs
verification on the actual workspace before being relied on.

## Verified service mappings

Checked against `orchestrator/srv/RunRoute.srv` and `orchestrator/README.md`
on branch `feature/grasp-deliver-integration` (where the real orchestrator
lives; `main` still has an empty `orchestrator/` as of this writing):

| Voice command | Orchestrator service | Notes |
|---|---|---|
| "开始抓取" / "start grasping" | `/orchestrator/run_grasp` | Currently always returns `NOT_READY` (the `frame_contract` capability is unresolved) -- reported verbatim, not hidden |
| "开始递送" / "start delivery" | `/orchestrator/run_deliver` | |
| "查看状态" / "status" | `/orchestrator/status` (topic, latched `std_msgs/String`) | A **read**, not a service call; does not touch the confirm/dispatch state machine |
| "取消任务" / "cancel task" | **none exists** | See "Why cancel is unavailable" below |
| "确认执行" / "confirm" | (local only) | Confirms a pending grasp/deliver; no orchestrator call by itself |

`RunRoute.srv` request fields actually used: `route` (set to the route's own
name as a same-name guard), `target`, `start_step` (always `""` here --
voice control never resumes a route), `dry_run`. `event_token` is left `""`;
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

Per `orchestrator/README.md`'s "Why the full grasp is NOT_READY": the
`frame_contract` capability (perception reports in `base_link`, arm_planning
needs `torso_link`, nobody has implemented the conversion) is unresolved, so
`run_grasp`'s preflight always returns that blocker and zero steps run. This
package reports `result.blockers` back exactly as received -- it does not
retry, does not fall back to some other behavior, and does not claim success.

## Package layout

```
voice_control/                          (catkin package: g1_voice_control)
├── package.xml, CMakeLists.txt, setup.py
├── requirements.txt                    # vosk, sounddevice, pyyaml -- imported lazily
├── config/
│   ├── commands.yaml                   # grasp_target, deliver_target, confirmation_window_sec
│   └── vosk.yaml                       # model_path, sample_rate (cn default, en documented)
├── launch/voice_control.launch         # standalone; NOT referenced by system_bringup
├── src/g1_voice_control/
│   ├── commands.py                     # phrase normalisation + matching + negation -- no ROS/Vosk
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

```bash
cd ~/catkin_ws/src
pip install -r voice_control/requirements.txt       # vosk, sounddevice, pyyaml
voice_control/scripts/download_model.sh cn           # or: en
cd ~/catkin_ws
catkin build g1_voice_control                        # orchestrator must build first (package.xml depend)
source devel/setup.bash
```

`sounddevice` needs PortAudio; if the pip install or the import fails, install
the system package first (`apt install portaudio19-dev` on Debian/Ubuntu).
Verify `vosk`'s prebuilt wheel actually covers the real machine's
architecture (Linux x86_64/aarch64, or macOS) -- it does not cover every
target, and this was never checked against the real workspace's arch.

## Running it

```bash
# enumerate input devices
rosrun g1_voice_control list_microphones.py

# WAV-file recognition, no ROS, no live mic
python3 scripts/recognize_wav.py --model voice_control/models/vosk-model-small-cn-0.22 --wav sample.wav

# live microphone, matched/confirmed but NO orchestrator calls (default)
roslaunch g1_voice_control voice_control.launch input_device:=2

# live microphone, real dispatch enabled
roslaunch g1_voice_control voice_control.launch input_device:=2 dry_run_backend:=false
```

Diagnostics: `rostopic echo /rosout` while the node runs shows every heard
transcript, match/no-match reason, and dispatch event
(`voice_control: heard ... -> matched/rejected/awaiting_confirmation/dispatched/...`).

## Supported commands and configuration

| Phrase (zh / en) | Command id | Needs "确认执行"/"confirm"? |
|---|---|---|
| 开始抓取 / start grasping | `grasp` | yes |
| 开始递送 / start delivery | `deliver` | yes |
| 查看状态 / status | `status` | no (read-only) |
| 取消任务 / cancel task | `cancel` | no (local-only; see above) |
| 确认执行 / confirm | `confirm` | -- |

`config/commands.yaml`: `grasp_target`, `deliver_target` (required for those
commands to be dispatchable at all -- unset means the command is refused
with "no target configured", never a guessed destination/label),
`confirmation_window_sec` (default 8s).

`config/vosk.yaml` / `~vosk_model_path`, `~sample_rate`: pick **one**
language's model (`vosk-model-small-cn-0.22` default, or
`vosk-model-small-en-us-0.15`); a single Vosk model is never bilingual. The
phrase table above already covers both languages for the matcher itself --
only the ASR model needs picking per deployment.

`~dry_run_backend` (launch arg, default `true`): when true, commands are
heard, matched and confirmed, but `orchestrator_client` is never constructed
and no service is ever called -- mirrors the orchestrator's own
`DryRunBackend` pattern (dry-run safety as a property of which object got
injected, not a flag re-checked per call site).

## Behaviour notes

- **Final results only.** Partial Vosk results never reach `commands.py`.
- **No substring matching.** An utterance must equal an allowlisted phrase
  after normalisation (case/whitespace/full-width only); "不要开始抓取"
  cannot match "开始抓取" by containment, and is additionally rejected by an
  explicit negation-marker check (`src/g1_voice_control/commands.py:NEGATION_MARKERS`).
- **One pending command, not a queue.** A second task-start utterance before
  confirmation *replaces* the pending one; nothing has executed yet, so there
  is nothing to protect by keeping the stale one.
- **Busy means no new task starts.** While a route call is in flight, a new
  grasp/deliver is rejected outright; `status` is unaffected (it is a
  separate topic read, not routed through the dispatch state machine).
- **No auto-retry.** A failed or timed-out `RunRoute` call is reported once;
  `dispatch.py` never calls it again on its own, since the route may already
  have started moving the arm.
- **Nothing replays.** State is in-memory only (one `Dispatcher` instance,
  one pending slot with a hard expiry); there is no queue that could replay
  an old command after a task finishes or a reconnect.

## Tests

```bash
# offline -- no ROS master, run directly or via catkin:
cd voice_control && PYTHONPATH=src python3 -m unittest discover -s test -v
# or: catkin run_tests g1_voice_control

# real ROS transport -- needs a roscore; UNTESTED here, verify on the real workspace:
rostest g1_voice_control test_ros_transport.test
```

**Run here, all passing (31 tests, offline):**
- `test_commands.py` (10): every supported phrase in both languages,
  case/whitespace/full-width normalisation, negation rejection, substring
  non-match, ambiguous-phrase construction failure, empty transcript.
- `test_dispatch.py` (16): confirm-before-dispatch, correct request-field
  mapping, confirm-with-nothing-pending, expiry just-after and
  just-inside the window, replace-not-stack on a second task-start,
  duplicate-utterance no-double-dispatch, busy rejection (task-start and
  confirm), missing-target refusal, cancel (pending/none-pending/busy),
  failed-call and raising-client both reported once with no retry,
  dry-run client makes no real calls.
- `test_orchestrator_client.py` (5): `grasp`/`deliver` map to the right
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

With `dry_run_backend:=true` (the default) and `roscore` + this node running:

1. Say "开始抓取". Expect a log line: `awaiting_confirmation ... target='bottle'`.
2. Say unrelated background speech for the confirmation window. Expect no
   dispatch and, after the window, the pending command silently expiring
   (next "确认执行" logs `ignored: nothing pending to confirm`).
3. Say "开始抓取" again, then "确认执行" within the window. Expect
   `dispatched` with `RecordingDryRunClient`'s message
   (`dry run: would call route for 'grasp' ...`) and, in the node's own log,
   confirmation that `RosRouteClient` was never constructed.
4. Say "不要开始抓取". Expect no match/no dispatch (negation rejected).
5. Say "取消任务" with nothing pending. Expect `cancelled: nothing was pending`.
