# OmniRevolve Plotter

XY-plotter (4 цвета) · STM32F446 + ESP32 (microROS) · ROS 2 UI · SPI DMA 512B · TIM4@10kHz

## Quick Start
- Docker image: `ghcr.io/omnirevolve/omnirevolve-dev:humble` (скоро)
- UI: `ros2 run omnirevolve_ros2_ui ui.py` (или `python3 ui.py`)

## Repos
- image-processor · stm32-firmware · esp32-core · esp32-microros · ros2-ui · ros2-messages · protocol

## Architecture (draft)
```mermaid
flowchart LR
UI -- byte_stream/cmds --> ESP32
ESP32 -- SPI DMA 512B --> STM32
STM32 -- UART status --> ESP32
ESP32 -- telemetry --> UI
STM32 --> TB6600 --> NEMA17

