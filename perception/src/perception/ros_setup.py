"""Shared rospy-side configuration for perception_node and perception_viewer.

Both nodes take the same private params (~model, ~calibration_file,
~camera_translation, ~camera_rotation_ypr), which perception.launch sets on
each of them, so they are guaranteed to use the same detector and extrinsics.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rospy

from perception.m1_perception import (
    CAMERA_ROTATION_YPR,
    CAMERA_TRANSLATION,
    Extrinsics,
    extrinsics_from_calibration_yaml,
    extrinsics_from_mount,
)

#: Fine-tuned on real captured video of the actual target object (a drink can,
#: labeled "bottle" -- see CLASS_RADIUS_M's comment in m1_perception.py).
MODEL_PATH = Path(__file__).resolve().parents[2] / "models" / "bottle_finetuned.pt"
STOCK_MODEL = "yolov8n.pt"


def load_model(tag: str):
    from ultralytics import YOLO

    model_path = rospy.get_param("~model", "")
    if model_path:
        pass  # explicit weights, e.g. model:=yolov8n.pt for the stock COCO classes
    elif MODEL_PATH.exists():
        model_path = str(MODEL_PATH)
    else:
        rospy.logwarn("[%s] %s is missing; falling back to the stock %s, which detects the can "
                      "poorly (see perception/README.md 'Detector')", tag, MODEL_PATH, STOCK_MODEL)
        model_path = STOCK_MODEL
    model = YOLO(model_path)
    rospy.loginfo("[%s] detector %s classes=%s", tag, model_path, list(model.names.values()))
    return model


def load_extrinsics(tag: str) -> Extrinsics:
    calib = rospy.get_param("~calibration_file", "")
    if calib:
        ext = extrinsics_from_calibration_yaml(calib)
        rospy.logwarn("[%s] extrinsics from %s (validate against a tape-measured object)", tag, calib)
    else:
        ext = extrinsics_from_mount(
            rospy.get_param("~camera_translation", list(CAMERA_TRANSLATION)),
            rospy.get_param("~camera_rotation_ypr", list(CAMERA_ROTATION_YPR)),
        )
    rospy.loginfo("[%s] t_base_opt=%s", tag, np.round(ext.t_base_opt, 4).tolist())
    return ext
