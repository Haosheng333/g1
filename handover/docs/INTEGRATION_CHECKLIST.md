# Handover Integration Checklist

This guide follows the current main-branch handover and orchestrator contracts.

## Ownership and data flow

```text
hand_control: acquire real tactile frames
    -> handover: validate frames, baseline, detect a sustained pull
    -> orchestrator: validate the session and handover phase
    -> hand_control: release
    -> arm_control: retract and stow
```

The handover monitor must not call hand or arm motion services. The orchestrator owns movement, the execution lock, and held-object bookkeeping.

## Before running hardware

- Identify the installed Revo2 variant; confirm it actually has tactile sensors.
- Obtain the vendor SDK field definitions and confirm channel units, indexing, timestamps, validity flags, and sample rate.
- Implement the real tactile publisher in hand_control using the verified API.
- Configure the mappings in `config/auto_release.yaml`; do not infer physical units from raw numbers.
- Record stationary holding, settling, pulls from several directions, and disconnection behavior.
- Calibrate thresholds and debounce using the procedure in `../README.md`, then validate on independent recordings.
- Start in detection-only mode (`release_integration: false`).
- Confirm the arm planning and execution capabilities required by `orchestrator/README.md`.
- Enable release integration only after detector validation and orchestration checks.

The shipped monitor is intentionally NOT_READY until schema and calibration prerequisites are supplied. A successful unit test does not remove those prerequisites.

## Current interfaces

| Interface | Contract |
|---|---|
| `/auto_release_monitor/arm` | `handover/ArmMonitor`: side and orchestrator-issued session_token |
| `/auto_release_monitor/disarm` | `std_srvs/Trigger` |
| `/orchestrator/run_auto_release` | `orchestrator/RunRoute`; validates and records the event for the task owning the lock |
| `/auto_release_monitor/ready` | Latched Bool readiness |
| `/auto_release_monitor/state` | Latched String diagnostic state |
| `/auto_release_monitor/event` | Candidate, confirmed, and fault events |

The monitor echoes the orchestrator's token; it must not generate a replacement. See the service definitions and orchestrator README for complete request fields.

## Superseded prototype

The earlier PR prototype used arm/label arming, a Trigger release callback, an independent handover coordinator, and a guessed flat tactile array. Those contracts do not match current main and have been removed from this branch.

Use `launch/auto_release_monitor.launch`, `config/auto_release.yaml`, and the orchestrator-owned sequence. Do not run the old demo.launch or handover_node.py against the current orchestrator.

## Offline checks

From the repository root:

```bash
python3 -m unittest discover -s handover/test
python3 -m unittest discover -s orchestrator/test
```

The tests use synthetic data and ROS substitutes. ROS builds, SDK support, physical motion, and measured release latency still require separate validation.
