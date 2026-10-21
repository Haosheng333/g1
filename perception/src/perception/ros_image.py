"""Decode sensor_msgs/Image into numpy arrays without cv_bridge.

Noetic's cv_bridge is compiled against numpy 1 and raises
"AttributeError: _ARRAY_API not found" under the container's numpy 2, so the
node decodes the raw buffer itself. Duck-typed on the message fields
(encoding, height, width, step, is_bigendian, data) so this stays ROS-free.
"""

from __future__ import annotations

import numpy as np

COLOUR_ENCODINGS = ("rgb8", "bgr8")
DEPTH_ENCODINGS = ("16uc1", "mono16")


def image_to_array(msg, expect_channels: int) -> np.ndarray:
    """Return BGR HxWx3 uint8 for colour (rgb8/bgr8), HxW uint16 for 16UC1 depth."""
    enc = msg.encoding.lower()
    if expect_channels == 3:
        if enc not in COLOUR_ENCODINGS:
            raise ValueError(f"unsupported colour encoding {msg.encoding!r} (want rgb8/bgr8)")
        dtype, channels = np.dtype(np.uint8), 3
    elif expect_channels == 1:
        if enc not in DEPTH_ENCODINGS:
            raise ValueError(f"unsupported depth encoding {msg.encoding!r} (want 16UC1 mm)")
        dtype = np.dtype(np.uint16).newbyteorder(">" if msg.is_bigendian else "<")
        channels = 1
    else:
        raise ValueError(f"expect_channels must be 1 or 3, got {expect_channels}")

    row_bytes = msg.width * channels * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) < msg.height * msg.step:
        raise ValueError(f"image buffer too small: {len(msg.data)} bytes for {msg.height}x{msg.step}")
    rows = np.frombuffer(msg.data, dtype=np.uint8)[: msg.height * msg.step].reshape(msg.height, msg.step)
    arr = rows[:, :row_bytes].view(dtype)
    arr = arr.reshape(msg.height, msg.width, channels) if channels > 1 else arr.reshape(msg.height, msg.width)
    if expect_channels == 3 and enc == "rgb8":
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr, dtype=arr.dtype.newbyteorder("="))
