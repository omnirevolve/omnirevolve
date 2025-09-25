# Containers (Dev Environment)

## Pull & Run (single container)

```bash
docker pull ghcr.io/omnirevolve/omnirevolve-dev:humble
docker run -it --name omni_dev --network host   --device /dev/ttyUSB0 --device /dev/ttyACM0   -v $HOME/omnirevolve_ws:/work   ghcr.io/omnirevolve/omnirevolve-dev:humble
```

## Compose (recommended)

```bash
docker compose pull
docker compose up -d
docker compose exec dev bash
```

## Workspace inside container

- `/work` is a bind mount to `$HOME/omnirevolve_ws` on the host.
- Clone public repos there:

```bash
mkdir -p /work && cd /work
git clone https://github.com/omnirevolve/omnirevolve-stm32-firmware.git
git clone https://github.com/omnirevolve/omnirevolve-esp32-microros.git
git clone https://github.com/omnirevolve/omnirevolve-esp32-core.git
git clone https://github.com/omnirevolve/omnirevolve-ros2-ui.git
git clone https://github.com/omnirevolve/omnirevolve-ros2-messages.git
git clone https://github.com/omnirevolve/omnirevolve-image-processor.git
git clone https://github.com/omnirevolve/omnirevolve-protocol.git
```

### Examples

- micro-ROS Agent:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

- ESP32 micro-ROS node:
```bash
cd /work/omnirevolve-esp32-microros
./scripts/prepare && ./scripts/configure && ./scripts/build
./scripts/flash && ./scripts/monitor
```

- UI:
```bash
cd /work/omnirevolve-ros2-ui
python3 ui.py
# or: ros2 run omnirevolve_ros2_ui ui
```
