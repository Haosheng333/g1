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
    "cylinder_sample_pixel",
    "draw_detections",
    "extrinsics_from_calibration_yaml",
    "extrinsics_from_mount",
    "handle_get_object_position",
    "median_depth_in_box",
    "median_nonzero_depth",
    "optical_to_base",
    "project_rows",
    "push_along_ray",
    "radius_from_silhouette",
    "run_detection_pipeline",
    "table_height_below_box",
    "top_height_in_box",
    "upright_cylinder_center",
]


def __getattr__(name):
    """Resolve a public name from the algorithm module on first access."""
    if name in __all__:
        from importlib import import_module

        module = "perception.overlay" if name == "draw_detections" else _LAZY_MODULE
        value = getattr(import_module(module), name)
        globals()[name] = value          # cache: later accesses skip importlib
        return value
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


def __dir__():
    return sorted(set(list(globals()) + __all__))
