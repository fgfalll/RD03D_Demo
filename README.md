# RD-03D Radar + WiFi Web Interface

This project implements a multi-target tracking radar system using an **ESP32-C6 (Seeed Studio XIAO)** and an **RD-03D Radar** module. 

The application sets up the ESP32 as a WiFi Station, tracks targets using the radar sensor, and serves a live, interactive "Tactical HUD" web interface over HTTP and WebSockets.

## Features

- **Hardware**: Built for the Seeed XIAO ESP32-C6 and RD-03D Radar module.
- **Multi-Target Tracking**: Tracks up to 3 targets simultaneously with distance, speed, and position (X/Y coordinates) calculations.
- **Classification**: Differentiates between static and active targets, including human detection logic and filtering.
- **Real-Time Web UI**: Embedded single-page web application (`radar_ui.html`) serving a sleek, real-time "Tactical HUD".
- **WebSocket Streaming**: Broadcasts target data to the web UI at ~20Hz (every 50ms) for smooth animations.
- **Serial Dashboard**: Optional live dashboard displayed over the serial monitor for debugging.

## Requirements

- PlatformIO
- ESP-IDF Framework (configured via PlatformIO)
- Seeed Studio XIAO ESP32-C6
- RD-03D Radar Module

## Wiring

| RD-03D Pin | ESP32-C6 Pin |
| ---------- | ------------ |
| TX         | RX (GPIO 0)  |
| RX         | TX (GPIO 1)  |
| VCC        | 3.3V / 5V    |
| GND        | GND          |

*Note: Ensure your power supply can handle the spikes when the ESP32 enables its WiFi radio.*

## Configuration

1. Open `src/main.c`.
2. Update the WiFi credentials in the configuration section:
   ```c
   #define WIFI_SSID "Your_SSID"
   #define WIFI_PASS "Your_PASSWORD"
   ```
3. Update any other tracker constants as needed (e.g., maximum distance, timeouts).

## Building and Flashing

This project uses PlatformIO. To build and upload to your board:

```bash
pio run -t upload
```

Make sure the serial monitor is running to capture the ESP32's IP address once it connects to your WiFi network:

```bash
pio device monitor
```

## Usage

1. Flash the code to the ESP32-C6.
2. Open the serial monitor to view the connection progress.
3. Once connected, the ESP32 will print its IP address (e.g., `192.168.x.x`).
4. Open a web browser on any device on the same network and navigate to `http://<ESP32_IP_ADDRESS>`.
5. You should see the Tactical HUD initializing and tracking targets in real-time.

## Project Structure

- `src/main.c`: The core C application (FreeRTOS tasks, Radar UART parsing, WebServer).
- `src/radar_ui.html`: The HTML/CSS/JS for the Web UI.
- `generate_html_header.py`: A script that runs before building to embed the HTML file as a C header (`radar_web_ui.h`).
- `platformio.ini`: PlatformIO configuration file.
