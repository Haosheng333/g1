# Isolated ROS integration harness

Not part of the offline suite: there is deliberately **no `__init__.py`**, so
`python3 -m unittest discover -s test` cannot import these modules (they need
`rospy`). They are also **not installed** as package executables and are
referenced by **no launch file** in the repository.

Run them with their own script, which starts its own ROS master:

```bash
orchestrator/test/integration/run_integration.sh
```

| File | Role |
|---|---|
| `run_integration.sh` | starts an isolated `roscore` on :11399, uploads test-only params, starts the doubles plus the **real** orchestrator and monitor, runs the test, tears down |
| `doubles_node.py` | hardware-facing test doubles. Real generated service/message types, faked behaviour; imports no SDK |
| `monitor_params.test.yaml` | test-only force schema so the real detector can be armed. Never loaded by `handover/launch/auto_release_monitor.launch` |
| `test_ros_pipeline.py` | the driver: real service calls, boundary-payload assertions, failure scenarios |

Evidence lands in `/tmp/pipeline_itest/evidence.json` and the node logs beside it.
