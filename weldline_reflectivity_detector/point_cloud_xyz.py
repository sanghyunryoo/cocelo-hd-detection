"""PointCloud2 XYZ decoding without a PCL dependency."""
from __future__ import annotations

from dataclasses import dataclass
import struct

from sensor_msgs.msg import PointField


@dataclass
class Point3D:
    x: float
    y: float
    z: float


def decode_point_cloud_xyz(message, point_stride: int = 1) -> list[Point3D]:
    if point_stride <= 0 or not message.point_step: raise ValueError("point_stride and point_step must be positive")
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")): raise ValueError("PointCloud2 is missing x, y, or z")
    formats = {PointField.FLOAT32: ("f", 4), PointField.FLOAT64: ("d", 8)}
    layout = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype not in formats or field.count < 1: raise ValueError("XYZ fields must be float32 or float64")
        code, size = formats[field.datatype]
        if field.offset + size > message.point_step: raise ValueError("invalid PointCloud2 layout")
        layout.append((field.offset, code))
    prefix, points, index = (">" if message.is_bigendian else "<"), [], 0
    for row in range(message.height):
        for column in range(message.width):
            if index % point_stride == 0:
                offset = row * message.row_step + column * message.point_step
                values = [struct.unpack_from(prefix + code, message.data, offset + field_offset)[0] for field_offset, code in layout]
                if all(value == value and abs(value) != float("inf") for value in values): points.append(Point3D(*values))
            index += 1
    return points
