# ======= base Dockerfile.dev (as provided; unchanged behavior) =======
# Base: ROS 2 Humble (Ubuntu 22.04)
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# 1) Core packages and build tools
RUN apt-get update && apt-get install -y \
    locales \
    git curl wget unzip nano \
    build-essential \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    python3-rosdep \
    udev usbutils \
 && rm -rf /var/lib/apt/lists/*

# 2) Locale
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# 3) rosdep
RUN rosdep init || true && rosdep update

# 4) Workspace for micro-ROS Agent
RUN mkdir -p /opt/uros_ws/src
WORKDIR /opt/uros_ws

# 5) Branch pins (for Humble)
ARG MICRO_ROS_AGENT_REF=humble
ARG MICRO_ROS_MSGS_REF=humble

# 6) Clone micro-ROS Agent
RUN git clone --depth 1 --branch "${MICRO_ROS_AGENT_REF}" \
      https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent

# 7) Clone micro_ros_msgs (underscore in repo name is intentional)
RUN set -eux; \
  if git clone --depth 1 --branch "${MICRO_ROS_MSGS_REF}" \
       https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs; then \
    echo "micro_ros_msgs cloned via git"; \
  else \
    echo "git clone failed, falling back to archive download"; \
    curl -fL "https://github.com/micro-ROS/micro_ros_msgs/archive/refs/heads/${MICRO_ROS_MSGS_REF}.zip" -o /tmp/micro_ros_msgs.zip; \
    mkdir -p src; unzip -q /tmp/micro_ros_msgs.zip -d /tmp; \
    mv "/tmp/micro_ros_msgs-${MICRO_ROS_MSGS_REF}" src/micro_ros_msgs; \
    rm -f /tmp/micro_ros_msgs.zip; \
  fi

# 8) Resolve dependencies and build up to micro_ros_agent
RUN . /opt/ros/humble/setup.sh \
 && rosdep install --from-paths src -i -y \
 && colcon build --symlink-install --packages-up-to micro_ros_agent

# 9) Non-root user (dialout/plugdev for serial devices)
ARG USER=devuser
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} ${USER} || true \
 && useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USER} || true \
 && usermod -a -G dialout,plugdev ${USER}

# 10) Auto-source base/agent overlays in interactive shells
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
 && echo 'if [ -f /opt/uros_ws/install/setup.bash ]; then source /opt/uros_ws/install/setup.bash; fi' >> /etc/bash.bashrc

# 11) Ownership
RUN chown -R ${USER}:${USER} /opt/uros_ws

USER ${USER}
WORKDIR /home/${USER}

CMD ["bash"]

# ======= additions derived from actual command history =======

# A) Extra tools used interactively: Midnight Commander + empy (fixes rosidl "em" import)
USER root
RUN apt-get update && apt-get install -y \
    mc \
    python3-empy \
 && rm -rf /var/lib/apt/lists/*
USER ${USER}

# B) Add helper functions to ~/.bashrc (safe heredoc; keeps ROS and ESP-IDF isolated)
RUN cat >> "$HOME/.bashrc" <<'BASHRC'
rosenv() {
  unset USE_ESP_IDF VIRTUAL_ENV PYTHONHOME PYTHONPATH
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  source /opt/ros/humble/setup.bash
  [ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
  echo "[✓] ROS env active (no ESP-IDF)"
}
idfenv() {
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_CURRENT_PREFIX
  if [ -f "$HOME/toolchains/esp-idf/export.sh" ]; then
    # ESP-IDF should be installed by the user into $HOME/toolchains (v4.4.7 recommended)
    source "$HOME/toolchains/esp-idf/export.sh"
    echo "[✓] ESP-IDF env active (no ROS)"
  else
    echo "[!] ESP-IDF not found. Clone to: ~/toolchains/esp-idf (v4.4.7) and run ./install.sh"
  fi
}
BASHRC

# C) Prepare user ROS workspace and build micro_ros_setup (humble branch)
RUN mkdir -p "$HOME/ros2_ws/src"
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git "$HOME/ros2_ws/src/micro_ros_setup" && \
    bash -lc 'source /opt/ros/humble/setup.bash && cd "$HOME/ros2_ws" && colcon build --symlink-install --packages-select micro_ros_setup'

# D) (Optional) Bring in your message package the way it was done (HTTPS clone).
#     Disable via --build-arg INCLUDE_OMNI_MSGS=0
ARG INCLUDE_OMNI_MSGS=1
RUN if [ "$INCLUDE_OMNI_MSGS" = "1" ]; then \
      git clone https://github.com/IvanNekrasov/omnirevolve-ros2-messages.git "$HOME/ros2_ws/src/omnirevolve_ros2_messages" || true; \
      bash -lc 'source /opt/ros/humble/setup.bash && cd "$HOME/ros2_ws" && colcon build --symlink-install'; \
    else \
      echo "Skipping omnirevolve_ros2_messages clone/build"; \
    fi

# E) Hint for interactive sessions
RUN echo -e "\n# Hint: run 'rosenv' to activate your user overlay" >> "$HOME/.bashrc"
