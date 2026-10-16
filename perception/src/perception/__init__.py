"""perception -- Module 1: detect a target object and report its base-frame pose.

The algorithm lives in `perception.m1_perception`, which imports ultralytics
(and therefore torch) plus the YOLO weights at module level.

Importing THIS package deliberately does not import it. catkin generates
`perception.srv` into the same package namespace, so a consumer that only wants
the service type -- the orchestrator's preflight, a unit test, any node that
calls /perception/get_object_position without running the detector -- would
otherwise be forced to have ultralytics installed just to say
`from perception.srv import GetObjectPosition`.

Every name below is still re-exported and reachable exactly as before; PEP 562
module `__getattr__` resolves each one from `perception.m1_perception` on first
attribute access, and caches it. The only behaviour change is WHEN a missing
optional dependency surfaces: on first use of the algorithm, rather than on
`import perception`.
"""

#: Module the public names are re-exported from, imported on demand.
_LAZY_MODULE = "perception.m1_perception"

__all__ = [
    "CameraIntrinsics",
    "Detection",
    "Extrinsics",
    "ObjectPositionResult",
    "backproject_pixel",
    "bbox_center",
    "handle_get_object_position",
    "median_nonzero_depth",
    "optical_to_base",
    "push_along_ray",
    "run_detection_pipeline",
]


def __getattr__(name):
    """Resolve a public name from the algorithm module on first access."""
    if name in __all__:
        from importlib import import_module

        value = getattr(import_module(_LAZY_MODULE), name)
        globals()[name] = value          # cache: later accesses skip importlib
        return value
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


def __dir__():
    return sorted(set(list(globals()) + __all__))
