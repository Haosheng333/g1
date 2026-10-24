# g1_voice_control

The spoken command, as one of the orchestrator's inputs. It recognises the
fetch-and-deliver sentence with a small slot grammar, plus a short allowlist of
fixed phrases, requires a confirmation step, and then publishes **one**
`g1_voice_control/TaskCommand` on `/voice/command`. The orchestrator subscribes
to that topic (as g1_intellect's `main_process` subscribes to `/voice/intent`),
runs the route under its own lock and preflight, and answers on
`/orchestrator/command_result`. This package depends on no other DIP package and
never touches the robot or hand SDK; typed `rosservice call`s to the
orchestrator keep working beside it.

## The spoken task

```
"go to the desk, pick up a can, go to the chair, and pass me, over"
   -> fetch_deliver  target=table_01  destination=chair_01  label=bottle
   -> (say "confirm")  -> /voice/command  -> orchestrator route fetch_deliver
```

`src/g1_voice_control/commands.py:TaskParser` is a slot grammar configured by
`config/lexicon.yaml` (verb classes, objects -> perception label, places ->
**standing waypoint** name, priors, wake and end words). It tokenises a final
transcript longest-phrase-first, skips unknown words (the, a, to, please), splits
into clauses on commas / "and" / "then" / a second verb, fills one frame per
clause and lowers the frames to steps, filling the gaps `main_process` fills
with the same priors (an object lives at the table; the user is at the chair).
**Exactly one shape is dispatched** -- navigate, grasp, navigate, handover.
Everything else is logged as *understood but unsupported*, with what was
understood, and nothing is called:

| Said | Result |
|---|---|
| "go to the desk, pick up a can, go to the chair, and pass me, over" | `fetch_deliver` table_01 / bottle / chair_01 |
| "lucy, walk to the table, grab a bottle, then go to the seat and hand it to me, over" | same |
| "bring a can to me, over" / "pick up a can and pass me, over" | same, from the priors |
| "go to the chair, over" | unsupported (navigate only) |
| "don't go to the desk ..." | rejected: negation |
| "what's the weather" | no match |

Places are the only point where vocabulary meets the map: `desk`/`table` ->
`table_01`, `chair`/`seat` -> `chair_01` are records in
`slam/maps/<map>/standing_waypoints.yaml`; the orchestrator refuses a name the
active map does not have. Edit `lexicon.yaml` for another map.

**Utterances end with "over".** Vosk emits a final result at every pause, so
the sentence arrives in pieces; the node buffers them and parses the whole
utterance once a result ends with an end word (`~require_end_word`, default
true; a buffer older than `~utterance_timeout_sec` = 15 s is dropped with a
log line). `confirm`, `cancel task` and `status` act immediately, no end word
needed.

**Confirmation.** `confirmation_mode: confirm` (default) keeps this package's
rule: the task is pending for `confirmation_window_sec` (8 s) and dispatches
only on "confirm" / "确认执行"; silence lets it expire.
`confirmation_mode: cancel_window` is g1_intellect `task_confirm_node`'s rule:
the window is a chance to say "cancel task"; when it closes the task
**dispatches**. There is no TTS in this package, so in that mode watch the log
(`voice_control: ... awaiting_confirmation: heard 'fetch_deliver' ...`).

**Vosk grammar.** With the lexicon loaded the node hands Vosk a grammar (the
lexicon vocabulary plus the fixed phrases, ~70 words), which makes the small
English model dependable on this command set; `use_grammar:=false` restores
free dictation.

## The orchestrator contract

| Topic | Type | Direction |
|---|---|---|
| `/voice/command` | `g1_voice_control/TaskCommand` | published here, subscribed by the orchestrator |
| `/orchestrator/command_result` | `g1_voice_control/TaskResult` | published by the orchestrator, matched here by `seq` |
| `/orchestrator/status` | `std_msgs/String` (latched) | read for the "status" phrase |

`TaskCommand` carries `seq`, `command_id` (the route: `fetch_deliver`, `grasp`,
`deliver`), `target`, `destination`, `label`, `summary` and `transcript`.
`TaskResult` echoes `seq` and carries `accepted`, `success`, `message`,
`blockers`, `dry_run`, `motion_issued`, `final_step`. `TopicCommandClient`
(`src/g1_voice_control/orchestrator_client.py`) publishes one command and blocks
until the matching result, so the dispatcher's BUSY state spans the whole route;
it refuses to publish when nothing subscribes, so a command is never lost.

| Voice command | Route | Fields |
|---|---|---|
| the spoken task | `fetch_deliver` | `target`=pickup standing pose, `destination`=delivery standing pose, `label`=perception class |
| "开始抓取" / "start grasping" | `grasp` | `target` from `commands.yaml` |
| "开始递送" / "start delivery" | `deliver` | `target` from `commands.yaml` |
| "查看状态" / "status" | — | reads `/orchestrator/status` |
| "取消任务" / "cancel task" | — | drops a pending, unconfirmed task; cannot stop a running route |
| "确认执行" / "confirm" | — | confirms the pending task |

## Package layout

```
voice_control/                          (catkin package: g1_voice_control)
├── package.xml, CMakeLists.txt, setup.py
├── msg/TaskCommand.msg, msg/TaskResult.msg   # the contract the orchestrator subscribes to
├── requirements.txt                    # vosk, sounddevice, pyyaml -- imported lazily
├── config/
│   ├── commands.yaml                   # phrases, confirmation mode/window, lexicon pointer
│   ├── lexicon.yaml                    # the slot grammar: verbs, objects, places -> standing waypoints
│   └── vosk.yaml                       # explicit language, model path, sample rate
├── launch/voice_control.launch         # standalone or optional system_bringup include
├── src/g1_voice_control/
│   ├── commands.py                     # exact phrases + TaskParser slot grammar -- no ROS/Vosk
│   ├── utterance.py                    # joins final results until the end word -- no ROS
│   ├── configuration.py                # validated YAML command/target loading -- no ROS/Vosk
│   ├── dispatch.py                     # confirm/expire/busy/cancel state machine -- no ROS/Vosk
│   ├── asr_adapter.py                  # Vosk mic/WAV adapters -- lazy imports
│   └── orchestrator_client.py          # TaskCommand publisher / TaskResult matcher + status reader
├── scripts/
│   ├── voice_control_node.py           # the ROS node
│   ├── list_microphones.py             # enumerate input devices
│   ├── recognize_wav.py                # WAV -> transcripts -> command match, no ROS
│   └── download_model.sh               # explicit model download (cn|en); never automatic
├── test/
│   ├── test_commands.py                # offline
│   ├── test_dispatch.py                # offline
│   ├── test_orchestrator_client.py     # offline (rospy/orchestrator.srv faked in sys.modules)
│   ├── test_task_parser.py             # offline: the spoken task, refusals, the utterance buffer
│   └── test_ros_transport.py (+ .test) # REAL ROS master needed; see below
└── models/                             # gitignored; populated by download_model.sh
```

## Setup

Run setup inside the repository's designated ROS Noetic container, whose
Python environment is already selected by `docker/g1.sh enter`:

```bash
cd ~/Workspace/g1_intellect
docker/g1.sh enter 'cd ~/Workspace/DIP && python3 -m pip install -r voice_control/requirements.txt'
docker/g1.sh enter 'cd ~/Workspace/DIP && voice_control/scripts/download_model.sh en'  # or: cn
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

# WAV-file recognition, no ROS, no live mic (16 kHz, 16-bit mono PCM)
rosrun g1_voice_control recognize_wav.py --model "$(rospack find g1_voice_control)/models/vosk-model-small-en-us-0.15" --wav sample.wav

# no audio at all: replay transcripts through match -> confirm -> dispatch
roslaunch g1_voice_control voice_control.launch input_mode:=scripted \
    scripted_transcripts:="go to the desk pick up a can go to the chair and pass me over|confirm"

# live microphone, matched/confirmed but NO orchestrator calls (default)
roslaunch g1_voice_control voice_control.launch input_device:=4

# live microphone, real dispatch enabled (the orchestrator still dry-runs unless ITS enable_motion is true)
roslaunch g1_voice_control voice_control.launch input_device:=4 dry_run_backend:=false

# the whole spoken-task stack, voice on
roslaunch system_bringup fetch_deliver.launch map_name:=lab_map_DIP voice_control:=true voice_dry_run_backend:=false
```

Diagnostics: `rostopic echo /rosout` while the node runs shows every heard
transcript, match/no-match reason, and dispatch event
(`voice_control: heard ... -> matched/rejected/awaiting_confirmation/dispatched/...`).

### Microphone sample rate

The workstation's ALC245 codec only runs at 44.1 kHz and above through ALSA,
and the container has no PulseAudio to resample for it, so a 16 kHz stream
cannot be opened on it. `VoskMicrophoneAdapter` therefore asks the device for
16 kHz, and if that is refused opens it at its own default rate (48 kHz here)
and downsamples to the model rate in pure python
(`asr_adapter.Resampler`: a 3:1 box average for 48 k -> 16 k, linear
interpolation otherwise). The startup log line
`microphone device 4 opened at 48000 Hz -> 16000 Hz for the model, latency 0.50s`
shows which path is in use. The stream is opened with an explicit 0.5 s
PortAudio latency (`stream_latency_sec`): with the library default this codec
overflows on 200 ms blocks even with an empty callback, and every overflow is
a fault that clears a pending confirmation.

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
`sample_rate`. Pick **one** language's model (`vosk-model-small-en-us-0.15`
default, because the spoken task is English; or `vosk-model-small-cn-0.22`);
a single Vosk model is never bilingual. The
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
# offline -- no ROS master, run directly or via catkin (83 tests):
cd voice_control && PYTHONPATH=src python3 -m unittest discover -s test -v
# or: catkin run_tests g1_voice_control

# real ROS transport -- needs a roscore; UNTESTED here, verify on the real workspace:
rostest g1_voice_control test_ros_transport.test
```

**Current local result: 83 passing and one ROS-only test skipped:**
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
