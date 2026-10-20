"""動画用のカメラ画像を JPEG で保存する (spec v2.0 §57-§59 の素材)。

    ros2 run video_compositor frame_grabber --ros-args \
        -p output_dir:=/workspace/output/<run_id>

保存先:
    <output_dir>/frames/overview/NNNNNN.jpg
    <output_dir>/frames/leader/NNNNNN.jpg
    <output_dir>/frames/follower/NNNNNN.jpg
    <output_dir>/frames/index.csv     stream,frame,sim_time (逐次書き)

**実行中は JPEG へ書くだけ**にして、合成は実行後にまとめて行う。
理由は 2 つある。

1. rosbag へ画像を入れると巨大になる。overview は 800x720 なので
   10 Hz x 300 s で 5 GB を超える。JPEG なら 200 MB 程度。
2. 実行中に重い処理を足すと追従性能が落ちる。Python で高レートの
   トピックを記録して follow RMSE が 0.59 -> 2.17 m に悪化した
   前例がある (DEVLOG S-10)。ここは 10 Hz x 3 本で、cv2.imencode は
   C++ 実装なので軽いが、**影響の有無は record_frames の
   あり / なしで実測して確かめる**。

Overview Camera は評価・動画専用である (spec §58)。
Follower の制御系はこのトピックを購読していない (spec §18, §70)。
"""

from __future__ import annotations

import csv
import os
import sys
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image

# Gazebo のカメラは best effort。reliable で購読すると 1 枚も来ない
# (DEVLOG S-7 と同根)
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STREAMS = {
    "overview": "/overview/image_raw",
    "leader": "/leader/camera/image_raw",
    "follower": "/follower/camera/image_raw",
}


class FrameGrabber(Node):
    def __init__(self) -> None:
        super().__init__("frame_grabber")
        self.declare_parameter("output_dir", "/workspace/output/latest")
        self.declare_parameter("jpeg_quality", 85)

        self.frames_dir = Path(str(self.get_parameter("output_dir").value)) / "frames"
        self.quality = int(self.get_parameter("jpeg_quality").value)
        self.bridge = CvBridge()
        self.counts = {name: 0 for name in STREAMS}
        self.dropped_before_clock = 0

        for name in STREAMS:
            (self.frames_dir / name).mkdir(parents=True, exist_ok=True)

        # **索引は逐次書く。** 終了時にまとめて書いていたため、
        # 記録完了を合図に走る合成が「index.csv が無い」で落ちた
        # (frame_grabber はデモ終了まで生き続けるので、終了時書き出しでは
        # 間に合わない)。30 行ごとに flush する (DEVLOG V-5)。
        self.index_file = open(self.frames_dir / "index.csv", "w", newline="")
        self.index_writer = csv.DictWriter(
            self.index_file, fieldnames=["stream", "frame", "sim_time"])
        self.index_writer.writeheader()
        self.index_file.flush()
        self.since_flush = 0
        for name, topic in STREAMS.items():
            self.create_subscription(
                Image, topic, self.make_callback(name), SENSOR_QOS)

        self.create_timer(30.0, self.report)
        self.get_logger().info(
            f"動画用のフレームを保存する: {self.frames_dir} / "
            f"{len(STREAMS)} 本 / JPEG 品質 {self.quality}")

    def make_callback(self, stream: str):
        def callback(msg: Image) -> None:
            # /clock が来る前は sim time が 0 で、telemetry と
            # 突き合わせられない。捨てる (DEVLOG S-9 と同じ注意)
            sim_time = self.get_clock().now().nanoseconds * 1e-9
            if sim_time <= 0.0:
                self.dropped_before_clock += 1
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:                    # noqa: BLE001
                self.get_logger().warn(f"{stream} の変換に失敗: {exc}")
                return
            number = self.counts[stream]
            path = self.frames_dir / stream / f"{number:06d}.jpg"
            cv2.imwrite(str(path), frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            self.counts[stream] = number + 1
            self.index_writer.writerow({"stream": stream, "frame": number,
                                        "sim_time": round(sim_time, 3)})
            self.since_flush += 1
            if self.since_flush >= 30:
                self.since_flush = 0
                self.index_file.flush()
        return callback

    def report(self) -> None:
        detail = " / ".join(f"{k}={v}" for k, v in self.counts.items())
        self.get_logger().info(f"frames: {detail}")
        empty = [k for k, v in self.counts.items() if v == 0]
        if empty:
            # 購読できていないストリームは黙って 0 枚になる。鳴らす
            self.get_logger().warn(f"まだ 1 枚も保存できていない: {empty}")

    def close_index(self) -> None:
        """索引を閉じる。本体は逐次書いているので flush だけ。"""
        try:
            self.index_file.flush()
            self.index_file.close()
        except Exception:                               # noqa: BLE001
            pass
        self.get_logger().info(
            f"索引: {self.frames_dir / 'index.csv'} "
            f"({sum(self.counts.values())} 行 / "
            f"/clock 待ちで {self.dropped_before_clock} 枚を捨てた)")


def main() -> int:
    rclpy.init()
    node = FrameGrabber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                   # noqa: BLE001
        # shutdown 中は KeyboardInterrupt が別の型に化けることがある
        # (DEVLOG R-5)。まだ生きているなら本物の異常
        if rclpy.ok():
            raise
    finally:
        node.report()
        node.close_index()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
