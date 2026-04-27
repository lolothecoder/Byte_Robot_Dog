FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-serial \
      ros-jazzy-xacro \
      ros-jazzy-joy \
      ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
 && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

CMD ["bash"]
