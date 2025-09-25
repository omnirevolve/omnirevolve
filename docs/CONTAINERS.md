# Containers & Firmware Workflow

This document describes how to use the GHCR dev image and build/flash the ESP32 micro‑ROS firmware. It consolidates the end‑to‑end workflow engineers will use to reproduce the setup.

> Based on your detailed firmware README; commands preserved and re‑organized for clarity. 

---

## 1) Host prerequisites
- Linux host (tested on Ubuntu 22.04)
- Docker Engine and Docker Compose plugin:
  ```bash
  docker --version
  docker compose version
  ```
- Connect the ESP32 board before starting the container so `/dev/ttyUSB*` is present (or restart the container after plugging in).

---

## 2) Run the dev container (GHCR)

```bash
docker pull ghcr.io/omnirevolve/omnirevolve-dev:humble
docker run -it --name omni_dev --network host   --device /dev/ttyUSB0 --device /dev/ttyACM0   -v $HOME/omnirevolve_ws:/work   ghcr.io/omnirevolve/omnirevolve-dev:humble
```
Or with Compose (recommended):
```bash
docker compose pull
docker compose up -d
docker exec -it omni_dev bash
```

Environment helpers available in the image:
- `rosenv` — source ROS 2 only (no ESP‑IDF)
- `idfenv` — source ESP‑IDF only (expects `~/toolchains/esp-idf` installed)
- `bothenv` — source both (if required)

Check serial devices:
```bash
ls -l /dev/ttyUSB*
```

---

## 3) (Optional) Build UI & messages inside the container

```bash
docker exec -it omni_dev bash
rosenv

cd ~/ros2_ws/src
git clone https://github.com/omnirevolve/omnirevolve-ros2-messages.git omnirevolve_ros2_messages
# (optional) UI
# git clone https://github.com/omnirevolve/omnirevolve-ros2-ui.git omnirevolve_ros2_ui

cd ~/ros2_ws
colcon build --symlink-install
rosenv  # re-source

# sanity check
ros2 interface show omnirevolve_ros2_messages/msg/PlotterTelemetry
```

---

## 4) ESP32 firmware (micro‑ROS node)

> Use the provided convenience script; it wraps micro‑ROS + ESP‑IDF tooling.

1. Go to the app:
   ```bash
   cd ~/ros2_ws/firmware/freertos_apps/apps/omnirevolve_esp32_microros
   ```

2. Clean (optional):
   ```bash
   ./firmware.sh clean
   ```

3. Configure transport and Agent endpoint:
   ```bash
   ./firmware.sh --agent-ip <HOST_IP> --agent-port 8888 configure
   ```

4. First‑time Wi‑Fi credentials via `menuconfig`:
   ```bash
   cd ~/ros2_ws
   source ~/ros2_ws/install/setup.bash
   ros2 run micro_ros_setup build_firmware.sh menuconfig
   ```

5. Build / Flash / Monitor:
   ```bash
   ./firmware.sh build
   ./firmware.sh flash
   ./firmware.sh monitor
   ```

Notes:
- If flashing fails, simply retry; after `monitor`, ESP32 may detach briefly — replug and retry.
- Ensure the Agent is reachable on the IP/port you configured.

---

## 5) micro‑ROS Agent

Run outside the container (host) or inside:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

---

## 6) Troubleshooting

- **`Package 'micro_ros_setup' not found`**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  ```
- **Interfaces not visible**
  ```bash
  source ~/ros2_ws/install/omnirevolve_ros2_messages/share/omnirevolve_ros2_messages/local_setup.bash
  ```
- **Serial device missing**
  Replug the USB cable and verify `/dev/ttyUSB*`; then retry `flash`/`monitor`.

---

## 7) Lifecycle

```bash
docker compose stop
docker compose start
docker exec -it omni_dev bash
docker compose down -v   # wipe and start fresh
```

---

**Happy plotting.**
