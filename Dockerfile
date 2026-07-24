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
#    Plus: sudo (rosdep needs it to apt-install ESP-IDF deps as non-root),
#    python-is-python3 (ESP-IDF v4.1 install.sh calls `python`).
USER root
RUN apt-get update && apt-get install -y \
    mc \
    python3-empy \
    sudo \
    python-is-python3 \
 && rm -rf /var/lib/apt/lists/*
# Passwordless sudo for the dev user so `rosdep install` can apt-install firmware deps.
RUN echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} \
 && chmod 440 /etc/sudoers.d/${USER}
USER ${USER}
# rosdep cache must exist for THIS user (Dockerfile init ran it only as root).
RUN rosdep update

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

# D) Bring in the shared message package (public repo; the old IvanNekrasov URL was dead).
#     Disable via --build-arg INCLUDE_OMNI_MSGS=0
ARG INCLUDE_OMNI_MSGS=1
RUN if [ "$INCLUDE_OMNI_MSGS" = "1" ]; then \
      git clone https://github.com/omnirevolve/omnirevolve-ros2-messages.git "$HOME/ros2_ws/src/omnirevolve_ros2_messages"; \
      bash -lc 'source /opt/ros/humble/setup.bash && cd "$HOME/ros2_ws" && colcon build --symlink-install'; \
    else \
      echo "Skipping omnirevolve_ros2_messages clone/build"; \
    fi

# E) Auto-source the full env in interactive shells (ROS + micro-ROS Agent overlay +
#    user ws). NOTE: omnirevolve_ros2_messages installs its ament hook under share/
#    (catkin-style), so colcon's aggregator does NOT chain it — source it explicitly.
RUN cat >> "$HOME/.bashrc" <<'BASHRC'

# --- omnirevolve autosource ---
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f /opt/uros_ws/install/setup.bash ] && source /opt/uros_ws/install/setup.bash 2>/dev/null
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
_omni_msgs="$HOME/ros2_ws/install/omnirevolve_ros2_messages/share/omnirevolve_ros2_messages/local_setup.bash"
[ -f "$_omni_msgs" ] && source "$_omni_msgs" 2>/dev/null
# --- end omnirevolve autosource ---

# Hint: run 'rosenv' to activate your user overlay
BASHRC

# F) ESP32 firmware workspace (micro-ROS freertos + ESP-IDF v4.1), baked into the image
#    so it survives `compose down -v` / disk loss. Adds ~4 GB. Disable with
#    --build-arg INCLUDE_ESP32_FW=0. App repo overridable via ESP32_MICROROS_URL.
#    This reproduces `firmware.sh prepare` + the recovery steps needed because the
#    upstream esp32 create.sh aborts (ESP-IDF v4.1 tooling needs setuptools<66 for
#    pkg_resources) BEFORE it imports the freertos_apps template.
ARG INCLUDE_ESP32_FW=1
ARG ESP32_MICROROS_URL=https://github.com/omnirevolve/omnirevolve-esp32-microros.git
RUN if [ "$INCLUDE_ESP32_FW" != "1" ]; then echo "Skipping ESP32 firmware ws"; else \
      set -ex; \
      source /opt/ros/humble/setup.bash; \
      source "$HOME/ros2_ws/install/setup.bash"; \
      export PIP_CONSTRAINT=/tmp/esp-idf-constraints.txt; \
      printf 'setuptools<66\npip<24\n' > "$PIP_CONSTRAINT"; \
      sudo apt-get update; \
      cd "$HOME/ros2_ws"; \
      ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32 || true; \
      VENV="$HOME/ros2_ws/firmware/toolchain/espressif/python_env/idf4.1_py3.10_env"; \
      if [ -x "$VENV/bin/python" ]; then "$VENV/bin/python" -m pip install 'setuptools<66' 'pip<24'; fi; \
      ( cd "$HOME/ros2_ws/firmware/toolchain/esp-idf" && ./install.sh esp32 ); \
      "$VENV/bin/python" -m pip install 'setuptools<66' 'pip<24'; \
      cd "$HOME/ros2_ws/firmware"; \
      if [ ! -d freertos_apps/microros_esp32_extensions ]; then \
        if [ -d freertos_apps/apps ]; then mv freertos_apps/apps /tmp/_apps_stash; fi; \
        rm -rf freertos_apps; \
        vcs import --input "$HOME/ros2_ws/src/micro_ros_setup/config/freertos/esp32/board.repos"; \
        if [ -d /tmp/_apps_stash ]; then cp -a /tmp/_apps_stash/. freertos_apps/apps/; rm -rf /tmp/_apps_stash; fi; \
      fi; \
      for p in ros2/rcl_logging/rcl_logging_spdlog ros2/rcl ros2/rosidl/rosidl_typesupport_introspection_cpp ros2/rcpputils uros/rcl/rcl_yaml_param_parser uros/rclc/rclc_examples; do \
        if [ -d "mcu_ws/$p" ]; then touch "mcu_ws/$p/COLCON_IGNORE"; fi; \
      done; \
      cd "$HOME/ros2_ws/firmware/freertos_apps/apps"; \
      rm -rf omnirevolve_esp32_microros; \
      git clone --recursive "$ESP32_MICROROS_URL" omnirevolve_esp32_microros; \
      chmod +x omnirevolve_esp32_microros/firmware.sh; \
      cd omnirevolve_esp32_microros; \
      ./firmware.sh --agent-ip 192.168.1.100 --agent-port 8888 configure; \
      sudo rm -rf /var/lib/apt/lists/*; \
    fi
