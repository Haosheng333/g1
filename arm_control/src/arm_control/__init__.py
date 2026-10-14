"""arm_control -- Module 3: the 250 Hz arm control loop and its waypoint API.

The control loop lives in `arm_control.arm_controller`, which imports
unitree_sdk2py (the robot's DDS layer) at module level.

Importing THIS package deliberately does not import it. catkin generates
`arm_control.srv` into the same package namespace, so a consumer that only
wants the service type -- the orchestrator's preflight, a unit test, any
tooling that inspects MoveArmJoints without talking to the robot -- would
otherwise be forced to have unitree_sdk2py installed just to say
`from arm_control.srv import MoveArmJoints`.

Every name below is still re-exported and reachable exactly as before; PEP 562
module `__getattr__` resolves each one from `arm_control.arm_controller` on
first attribute access, and caches it. No control logic, gain, joint index or
timing is affected: the same module is imported, just later. The only
behaviour change is WHEN a missing SDK surfaces: on first use of the
controller, rather than on `import arm_control`.
"""

#: Module the public names are re-exported from, imported on demand.
_LAZY_MODULE = "arm_control.arm_controller"

__all__ = [
    "ArmController",
    "Execute_Waypoint_Path",
    "Get_Measured",
    "JointIndex",
    "Mode",
    "Run_Arm_Controller",
    "Set_Target",
    "Wait_Until_Arrived",
    "left_target",
    "right_target",
]


def __getattr__(name):
    """Resolve a public name from the controller module on first access."""
    if name in __all__:
        from importlib import import_module

        value = getattr(import_module(_LAZY_MODULE), name)
        globals()[name] = value          # cache: later accesses skip importlib
        return value
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


def __dir__():
    return sorted(set(list(globals()) + __all__))
