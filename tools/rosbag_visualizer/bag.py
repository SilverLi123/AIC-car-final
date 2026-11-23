"""Streaming bag adapters. Only deserialization is imported from rclpy."""

from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import yaml

from telemetry import log_event


def read_metadata(path):
    if not path.is_dir():
        return {}
    entry = path / "metadata.yaml"
    if not entry.is_file():
        raise ValueError(f"{path} has no metadata.yaml; pass one .db3 or .mcap file")
    document = yaml.safe_load(entry.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("rosbag2_bagfile_information"), dict):
        raise ValueError(f"Invalid ROS 2 metadata: {entry}")
    return document["rosbag2_bagfile_information"]


def storage_for(path, explicit, metadata):
    formats = {".db3": "sqlite3", ".mcap": "mcap"}
    detected = metadata.get("storage_identifier") if path.is_dir() else formats.get(path.suffix.lower())
    if explicit != "auto" and detected and explicit != detected:
        raise ValueError(f"Bag storage is {detected}, but --storage requested {explicit}.")
    if explicit != "auto":
        return explicit
    if not detected:
        raise ValueError("Cannot infer bag storage; pass --storage sqlite3 or mcap")
    return detected


def header_ns(msg):
    stamp = getattr(getattr(msg, "header", msg), "stamp", None)
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec) if stamp is not None else None


class Bag:
    def __init__(self, path: Path, storage, backend):
        self.path = path
        if not path.exists():
            raise ValueError(f"Bag does not exist: {path}")
        self.metadata = read_metadata(path)
        self.storage = storage_for(path, storage, self.metadata)
        self.compression = str(self.metadata.get("compression_mode", "")).lower()
        if backend == "auto" and self.compression == "file":
            backend = "rosbags"
        self.backend = backend
        self.seen = {}
        self.decode_errors = Counter()
        if backend != "rosbags":
            try:
                import rosbag2_py
                self.native = rosbag2_py
                self.backend = "rosbag2_py"
            except ImportError:
                if backend == "rosbag2_py":
                    raise ValueError("rosbag2_py unavailable. Source Humble or TROS first.") from None
                self.backend = "rosbags"

    def messages(self, wanted):
        if self.backend == "rosbag2_py":
            yield from self._native_messages(wanted)
        else:
            yield from self._portable_messages(wanted)

    def _native_messages(self, wanted):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        native = self.native
        fmt = native.StorageOptions(uri=str(self.path), storage_id=self.storage)
        converter = native.ConverterOptions("", "")
        if self.compression == "file":
            # Humble's native file decompressor can create a sibling in the input directory.
            raise ValueError("Use --backend rosbags for ROS file-compressed bags, "
                             "or analyze a separately decompressed copy.")
        reader = (native.SequentialCompressionReader() if self.compression == "message"
                  else native.SequentialReader())
        try:
            reader.open(fmt, converter)
        except RuntimeError as error:
            raise ValueError(f"Cannot open {self.storage} bag with rosbag2_py: {error}") from error
        self.seen.update((x.name, x.type) for x in reader.get_all_topics_and_types())
        if not wanted.intersection(self.seen):
            return
        reader.set_filter(native.StorageFilter(topics=sorted(wanted)))
        types = {name: get_message(self.seen[name]) for name in wanted.intersection(self.seen)}
        while reader.has_next():
            topic, raw, ns = reader.read_next()
            if topic not in types:
                continue
            try:
                msg = deserialize_message(raw, types[topic])
            except (ValueError, RuntimeError):
                self.decode_errors[topic] += 1
                continue
            yield topic, int(ns), msg

    def _portable_messages(self, wanted):
        try:
            from rosbags.rosbag2 import Reader, ReaderError
            from rosbags.typesys import Stores, get_typestore
        except ImportError:
            raise ValueError("Install rosbags for local decoding or source ROS 2 Humble/TROS.") from None
        # Only our standard telemetry types need to be installed. Many bags
        # also contain TROS custom message definitions we never deserialize.
        store = get_typestore(Stores.ROS2_HUMBLE)
        try:
            with Reader(self.path) as reader:
                self.seen.update((x.topic, x.msgtype) for x in reader.connections)
                selected = [x for x in reader.connections if x.topic in wanted]
                if not selected:
                    return
                for connection, ns, raw in reader.messages(connections=selected):
                    try:
                        msg = store.deserialize_cdr(raw, connection.msgtype)
                    except (KeyError, ValueError, RuntimeError, AssertionError):
                        self.decode_errors[connection.topic] += 1
                        continue
                    yield connection.topic, int(ns), msg
        except ReaderError as error:
            raise ValueError(f"Cannot open/read bag with rosbags: {error}") from error


def decode_image(msg):
    if not hasattr(msg, "encoding"):
        if "compresseddepth" in getattr(msg, "format", "").lower():
            raise ValueError("Depth images are not supported")
        raw = np.frombuffer(msg.data, np.uint8)
        result = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if result is None:
            raise ValueError("Compressed image could not be decoded")
        return result
    encodings = {
        "bgr8": (3, None), "rgb8": (3, cv2.COLOR_RGB2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR), "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
    }
    if msg.encoding.lower() not in encodings:
        raise ValueError(f"Unsupported sensor_msgs/Image encoding: {msg.encoding}")
    channels, conversion = encodings[msg.encoding.lower()]
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if h < 1 or w < 1 or step < w*channels or len(msg.data) != step*h:
        raise ValueError("Invalid image dimensions, step or data length")
    rows = np.frombuffer(msg.data, np.uint8).reshape(h, step)
    frame = np.ascontiguousarray(rows[:, :w*channels].reshape(h, w, channels))
    return cv2.cvtColor(frame, conversion) if conversion is not None else frame


def telemetry_value(role, msg, logger):
    if role in {"cmd", "odom"}:
        twist = getattr(msg, "twist", msg)
        twist = getattr(twist, "twist", twist)
        return float(twist.linear.x), float(twist.angular.z)
    if role == "state":
        return (str(msg.data),)
    if role == "start":
        return (bool(msg.data),)
    if role == "centroid":
        return float(msg.x), float(msg.y)
    if role == "log":
        name = logger.strip("/").replace("/", ".")
        if msg.name == name:
            return log_event(msg.msg)
    return None
