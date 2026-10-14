"""handover -- auto-release handover monitor.

`release_monitor` is the ROS-independent adaptive-baseline state machine.
`tactile_adapter` is the configurable, DISABLED-by-default bridge from a
tactile message to the scalar the state machine consumes.

The monitor only ever *detects* and then asks the orchestrator to act. It
never commands the arm or the hand.
"""
