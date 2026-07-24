#!/usr/bin/env python3
"""Fast RealSense USB visualizer with optional OpenCV-DNN detection overlay.

This diagnostic tool intentionally does not use ROS 2.  It opens every connected
RealSense color stream directly through librealsense, overlays serial/USB IDs,
and can draw detections from a lightweight ONNX model for quick field checks.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
import re
import signal
import sys
import time
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as error:
    rs = None
    PYREALSENSE_IMPORT_ERROR = error
else:
    PYREALSENSE_IMPORT_ERROR = None


Box = Tuple[int, int, int, int, float, int]


@dataclass(frozen=True)
class RealSenseDevice:
    serial: str
    usb_port_id: str
    physical_port: str
    name: str


@dataclass
class CameraRuntime:
    device: RealSenseDevice
    pipeline: object
    last_frame: Optional[np.ndarray] = None
    last_boxes: List[Box] = field(default_factory=list)
    last_infer_time: float = 0.0
    frame_count: int = 0
    fps_time: float = field(default_factory=time.monotonic)
    fps: float = 0.0


def default_person_model() -> Path:
    script = Path(__file__).resolve()
    candidates = [
        script.parents[1] / "weights" / "person_yolov5n.onnx",
        script.parents[2] / "share" / "weldline_reflectivity_detector" / "weights" / "person_yolov5n.onnx",
        Path.cwd() / "weights" / "person_yolov5n.onnx",
    ]
    return next((path for path in candidates if path.is_file()), candidates[0])


def normalize_usb_port_id(physical_port: str) -> str:
    matches = re.findall(r"(?:^|/)([0-9]+-[0-9]+(?:\.[0-9]+)*(?::[0-9.]+)?)(?:/|$)", physical_port)
    return matches[-1].split(":", 1)[0] if matches else physical_port


def enumerate_devices() -> List[RealSenseDevice]:
    if rs is None:
        raise RuntimeError(f"pyrealsense2 is required: {PYREALSENSE_IMPORT_ERROR}")
    devices = []
    for device in rs.context().query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        name = device.get_info(rs.camera_info.name)
        try:
            physical_port = device.get_info(rs.camera_info.physical_port)
        except RuntimeError:
            physical_port = ""
        devices.append(RealSenseDevice(serial, normalize_usb_port_id(physical_port), physical_port, name))
    return sorted(devices, key=lambda item: (item.usb_port_id, item.serial))


def parse_profile(profile: str) -> Tuple[int, int, int]:
    if not re.fullmatch(r"[1-9][0-9]*,[1-9][0-9]*,[1-9][0-9]*", profile):
        raise argparse.ArgumentTypeError("profile must be width,height,fps, for example 640,480,30")
    width, height, fps = (int(value) for value in profile.split(","))
    return width, height, fps


def parse_tile_size(tile_size: str) -> Tuple[int, int]:
    if not re.fullmatch(r"[1-9][0-9]*,[1-9][0-9]*", tile_size):
        raise argparse.ArgumentTypeError("tile size must be width,height, for example 640,360")
    width, height = (int(value) for value in tile_size.split(","))
    return width, height


def letterbox(image: np.ndarray, size: int) -> Tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale = min(size / float(width), size / float(height))
    resized_w, resized_h = int(round(width * scale)), int(round(height * scale))
    pad_x, pad_y = (size - resized_w) // 2, (size - resized_h) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    canvas[pad_y : pad_y + resized_h, pad_x : pad_x + resized_w] = resized
    return canvas, scale, pad_x, pad_y


class OpenCVDnnDetector:
    def __init__(
        self,
        weights: Path,
        input_size: int,
        confidence_threshold: float,
        nms_threshold: float,
        target_class_id: int,
    ) -> None:
        if not weights.is_file():
            raise FileNotFoundError(f"Detection ONNX file not found: {weights}")
        self.input_size = input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.target_class_id = target_class_id
        self.net = cv2.dnn.readNetFromONNX(str(weights))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect(self, bgr: np.ndarray) -> List[Box]:
        input_image, scale, pad_x, pad_y = letterbox(bgr, self.input_size)
        blob = cv2.dnn.blobFromImage(input_image, 1.0 / 255.0, (self.input_size, self.input_size), swapRB=True, crop=False)
        self.net.setInput(blob)
        output = self.net.forward()
        rows = self._as_rows(output)
        boxes: List[List[int]] = []
        scores: List[float] = []
        class_ids: List[int] = []
        image_h, image_w = bgr.shape[:2]

        for row in rows:
            if row.shape[0] < 5:
                continue
            score = float(row[4])
            class_id = -1
            if row.shape[0] == 6:
                class_id = int(round(float(row[5])))
            elif row.shape[0] > 6:
                has_objectness = row.shape[0] != 84
                class_start = 5 if has_objectness else 4
                class_scores = row[class_start:]
                if class_scores.size == 0:
                    continue
                objectness = float(row[4]) if has_objectness else 1.0
                if self.target_class_id >= 0:
                    if self.target_class_id >= class_scores.size:
                        continue
                    class_id = self.target_class_id
                    score = objectness * float(class_scores[self.target_class_id])
                else:
                    class_id = int(np.argmax(class_scores))
                    score = objectness * float(class_scores[class_id])
            if self.target_class_id >= 0 and class_id >= 0 and class_id != self.target_class_id:
                continue
            if self.target_class_id > 0 and class_id < 0:
                continue
            if score < self.confidence_threshold:
                continue

            x1, y1, x2, y2 = (float(row[0]), float(row[1]), float(row[2]), float(row[3]))
            if x2 <= x1 or y2 <= y1:
                cx, cy, width, height = x1, y1, x2, y2
                x1, y1, x2, y2 = cx - width / 2.0, cy - height / 2.0, cx + width / 2.0, cy + height / 2.0
            x1 = (x1 - pad_x) / scale
            y1 = (y1 - pad_y) / scale
            x2 = (x2 - pad_x) / scale
            y2 = (y2 - pad_y) / scale
            left = max(0, min(image_w - 1, int(round(x1))))
            top = max(0, min(image_h - 1, int(round(y1))))
            right = max(left + 1, min(image_w, int(round(x2))))
            bottom = max(top + 1, min(image_h, int(round(y2))))
            boxes.append([left, top, right - left, bottom - top])
            scores.append(score)
            class_ids.append(class_id)

        kept = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold, self.nms_threshold)
        if len(kept) == 0:
            return []
        indices = np.array(kept).reshape(-1)
        return [
            (boxes[index][0], boxes[index][1], boxes[index][2], boxes[index][3], scores[index], class_ids[index])
            for index in indices
        ]

    @staticmethod
    def _as_rows(output: np.ndarray) -> np.ndarray:
        output = np.asarray(output)
        if output.ndim == 3:
            output = output[0]
        if output.ndim != 2:
            return np.empty((0, 0), dtype=np.float32)
        if output.shape[0] < output.shape[1] and output.shape[0] in (5, 6, 84, 85):
            output = output.T
        return output.astype(np.float32, copy=False)


def start_cameras(devices: Sequence[RealSenseDevice], profile: Tuple[int, int, int]) -> List[CameraRuntime]:
    width, height, fps = profile
    runtimes: List[CameraRuntime] = []
    for device in devices:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(device.serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            pipeline.start(config)
        except RuntimeError as error:
            print(f"[ERROR] Cannot open serial={device.serial}, usb_port_id={device.usb_port_id or 'n/a'}: {error}", file=sys.stderr)
            continue
        print(
            f"[INFO] Opened {device.name}: serial={device.serial}, "
            f"usb_port_id={device.usb_port_id or 'n/a'}, physical_port={device.physical_port or 'n/a'}"
        )
        runtimes.append(CameraRuntime(device=device, pipeline=pipeline))
    return runtimes


def update_frame(runtime: CameraRuntime) -> None:
    frames = runtime.pipeline.poll_for_frames()
    if not frames:
        return
    color = frames.get_color_frame()
    if not color:
        return
    runtime.last_frame = np.asanyarray(color.get_data())
    runtime.frame_count += 1
    now = time.monotonic()
    elapsed = now - runtime.fps_time
    if elapsed >= 1.0:
        runtime.fps = runtime.frame_count / elapsed
        runtime.frame_count = 0
        runtime.fps_time = now


def draw_overlay(image: np.ndarray, runtime: CameraRuntime, boxes: Sequence[Box]) -> np.ndarray:
    frame = image.copy()
    header_h = 70
    cv2.rectangle(frame, (0, 0), (frame.shape[1], header_h), (0, 0, 0), cv2.FILLED)
    cv2.putText(frame, runtime.device.name, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(
        frame,
        f"serial={runtime.device.serial} | usb_port_id={runtime.device.usb_port_id or 'n/a'}",
        (8, 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"fps={runtime.fps:4.1f} | detections={len(boxes)}",
        (8, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    for left, top, width, height, score, class_id in boxes:
        right, bottom = left + width, top + height
        cv2.rectangle(frame, (left, top), (right, bottom), (60, 220, 255), 2)
        label = f"id={class_id} {score:.2f}" if class_id >= 0 else f"{score:.2f}"
        cv2.putText(frame, label, (left, max(18, top - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 220, 255), 1, cv2.LINE_AA)
    return frame


def make_canvas(runtimes: Sequence[CameraRuntime], tile_size: Tuple[int, int]) -> np.ndarray:
    tile_w, tile_h = tile_size
    tiles = []
    for runtime in runtimes:
        frame = runtime.last_frame
        if frame is None:
            frame = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
            cv2.putText(frame, "waiting for RGB stream", (20, tile_h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA)
        else:
            frame = draw_overlay(frame, runtime, runtime.last_boxes)
            frame = cv2.resize(frame, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
        tiles.append(frame)
    columns = max(1, math.ceil(math.sqrt(len(tiles))))
    rows = math.ceil(len(tiles) / columns)
    blank = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    tiles.extend([blank] * (rows * columns - len(tiles)))
    return np.vstack([np.hstack(tiles[index * columns : (index + 1) * columns]) for index in range(rows)])


def stop_cameras(runtimes: Sequence[CameraRuntime]) -> None:
    for runtime in runtimes:
        try:
            runtime.pipeline.stop()
            print(f"[INFO] Released serial={runtime.device.serial}, usb_port_id={runtime.device.usb_port_id or 'n/a'}")
        except RuntimeError as error:
            print(f"[WARN] Release failed for serial={runtime.device.serial}: {error}", file=sys.stderr)
    cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-only", action="store_true", help="Only print connected RealSense serial/USB IDs and exit.")
    parser.add_argument("--color-profile", default="640,480,30", type=parse_profile, help="Color stream profile: width,height,fps.")
    parser.add_argument("--tile-size", default="640,360", type=parse_tile_size, help="Display tile size: width,height.")
    parser.add_argument("--weights", type=Path, default=default_person_model(), help="ONNX detector model. Default: weights/person_yolov5n.onnx.")
    parser.add_argument("--no-detect", action="store_true", help="Disable OpenCV-DNN detection overlay.")
    parser.add_argument("--detect-fps", type=float, default=15.0, help="Maximum detector FPS per camera; display still runs as fast as possible.")
    parser.add_argument("--input-size", type=int, default=640, help="Square detector input size.")
    parser.add_argument("--confidence-threshold", type=float, default=0.35)
    parser.add_argument("--nms-threshold", type=float, default=0.45)
    parser.add_argument("--target-class-id", type=int, default=0, help="COCO person is 0. Use -1 for best class.")
    args = parser.parse_args()

    if len(args.tile_size) != 2 or args.tile_size[0] <= 0 or args.tile_size[1] <= 0:
        parser.error("--tile-size must be width,height")
    if args.detect_fps <= 0.0:
        parser.error("--detect-fps must be > 0")
    if args.input_size <= 0:
        parser.error("--input-size must be > 0")

    try:
        devices = enumerate_devices()
    except RuntimeError as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 2
    if not devices:
        print("[ERROR] No RealSense devices found.", file=sys.stderr)
        return 1
    print("[INFO] Connected RealSense devices:")
    for device in devices:
        print(f"  {device.name}: serial={device.serial}, usb_port_id={device.usb_port_id or 'n/a'}, physical_port={device.physical_port or 'n/a'}")
    if args.list_only:
        return 0

    detector: Optional[OpenCVDnnDetector] = None
    if not args.no_detect:
        try:
            detector = OpenCVDnnDetector(args.weights, args.input_size, args.confidence_threshold, args.nms_threshold, args.target_class_id)
        except Exception as error:
            print(f"[ERROR] Cannot load detector with OpenCV DNN: {error}", file=sys.stderr)
            return 2
        print(f"[INFO] Detection enabled: weights={args.weights}, target_class_id={args.target_class_id}")

    stop_requested = False

    def handle_signal(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    runtimes = start_cameras(devices, args.color_profile)
    if not runtimes:
        return 1

    window_name = "RealSense USB visualizer + detection (q/Esc/Ctrl+C to quit)"
    interval = 1.0 / args.detect_fps
    try:
        while not stop_requested:
            now = time.monotonic()
            for runtime in runtimes:
                update_frame(runtime)
                if detector is not None and runtime.last_frame is not None and now - runtime.last_infer_time >= interval:
                    runtime.last_boxes = detector.detect(runtime.last_frame)
                    runtime.last_infer_time = now
            canvas = make_canvas(runtimes, args.tile_size)
            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        stop_cameras(runtimes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
