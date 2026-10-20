# syntax=docker/dockerfile:1
# robot-follow-lab : ROS 2 Jazzy + Gazebo Harmonic (vendored) + Nav2 + CPU AI
#
# Stages:
#   sim     - Gazebo Harmonic + ros_gz のみ (Phase 0 レンダリング計測用)
#   control - + ros2_control / gz_ros2_control / Nav2
#   full    - + PyTorch CPU / ONNX Runtime / OpenCV / FFmpeg  (既定)
ARG ROS_DISTRO=jazzy

########################  stage: sim  ########################
FROM ros:${ROS_DISTRO}-ros-base AS sim
ARG ROS_DISTRO
SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-image \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher \
      libgl1-mesa-dri libglx-mesa0 libegl1 libgles2 libglu1-mesa \
      xvfb x11-utils procps python3-numpy \
 && rm -rf /var/lib/apt/lists/*

# CPU-only / headless レンダリング (llvmpipe)
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    QT_QPA_PLATFORM=offscreen \
    ROS_DOMAIN_ID=42

WORKDIR /workspace

########################  stage: control  ########################
FROM sim AS control
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-gz-ros2-control \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-nav2-simple-commander \
      ros-${ROS_DISTRO}-tf-transformations \
 && rm -rf /var/lib/apt/lists/*

########################  stage: full  ########################
FROM control AS full
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-rosbag2 \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      python3-opencv python3-pip python3-venv python3-yaml ffmpeg \
 && rm -rf /var/lib/apt/lists/*

# PyTorch / ONNX Runtime は venv に隔離する。
#   * apt 管理の numpy(1.26) / protobuf を pip が削除しようとして壊すのを防ぐ
#   * --system-site-packages により rclpy・cv_bridge 等の ROS パッケージは見える
#   * ROS Jazzy の C 拡張は numpy 1.x ABI でビルドされているため numpy<2 を維持する
ENV VIRTUAL_ENV=/opt/mlvenv
RUN python3 -m venv --system-site-packages ${VIRTUAL_ENV} \
 && echo "numpy<2" > /tmp/constraints.txt \
 && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir --upgrade pip \
 && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir -c /tmp/constraints.txt \
      --index-url https://download.pytorch.org/whl/cpu torch \
 && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir -c /tmp/constraints.txt onnxruntime onnx \
 && rm -f /tmp/constraints.txt \
 && ${VIRTUAL_ENV}/bin/python -c "import numpy; assert numpy.__version__.startswith('1.'), numpy.__version__" \
 && ${VIRTUAL_ENV}/bin/python -c "import torch, onnxruntime, numpy; print('torch', torch.__version__, '/ ort', onnxruntime.__version__, '/ numpy', numpy.__version__)"
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"
# ROS の console script (ros2 run が実行する entry point) は shebang が
# #!/usr/bin/python3 になり、**システム python** で動く。venv を PATH 先頭に
# 置いても効かないため、venv の site-packages を PYTHONPATH に追加して
# どちらの interpreter からも onnxruntime / torch を import できるようにする。
# numpy は venv に入れていないのでシステム版 1.26 が使われ、ABI も保たれる
# (DEVLOG E-3)。
ENV PYTHONPATH="${VIRTUAL_ENV}/lib/python3.12/site-packages:${PYTHONPATH}"

ENV OMP_NUM_THREADS=4

########################  stage: app  ########################
# ソースを取り込み、ワークスペースをビルドする。
# compose からは models/ data/ output/ のみをマウントする (spec §51)。
FROM full AS app
ARG ROS_DISTRO

COPY ros_ws/src /workspace/ros_ws/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && cd /workspace/ros_ws \
 && colcon build --symlink-install --event-handlers console_direct+

COPY worlds     /workspace/worlds
COPY scenarios  /workspace/scenarios
COPY training   /workspace/training
COPY scripts    /workspace/scripts

# ドキュメントと Makefile / compose.yaml も入れる。
# run_tests.sh の DEVLOG 整合・ドキュメント整合の検査が読む。
# 入れていなかったため `make test` が「README.md が無い」で落ち、
# DEVLOG の検査は実行すらされていなかった (DEVLOG S-23)。
# 変更頻度が高いので、ビルドし直しが軽くなるよう最後に置く
COPY README.md EXERCISE.md INSTRUCTOR_NOTES.md DEVLOG.md \
     Makefile compose.yaml /workspace/
COPY docs /workspace/docs
COPY .claude /workspace/.claude

ENV FOLLOWBOT_WORKSPACE=/workspace
WORKDIR /workspace
ENTRYPOINT ["/workspace/scripts/entrypoint.sh"]
CMD ["bash"]
