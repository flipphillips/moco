# OLED UI Plan - DMC Project

## Goal
Add a local I2C OLED status display to the Arduino Giga R1 (M7 Core) to provide real-time feedback on system state, motor activity, and camera triggers without requiring a connected host computer.

## Hardware Specifications
*   **Display:** 0.96" or 1.3" I2C OLED (SSD1306 driver).
*   **Resolution:** 128x64 pixels.
*   **Logic Level:** **3.3V ONLY**.
    *   *Warning:* The Arduino Giga R1 is strictly 3.3V. Powering the OLED from 5V or using 5V logic will damage pins D20 (SDA) and D21 (SCL) due to the display's onboard pull-up resistors.
*   **Pinout:**
    *   VCC: 3.3V
    *   GND: GND
    *   SCL: D21 (Arduino Giga SCL)
    *   SDA: D20 (Arduino Giga SDA)

## Software Architecture
To maintain the high-precision motor pulse timing on the M4 core, all UI rendering is restricted to the M7 core.

*   **Core:** M7 (Main application core).
*   **Library:** `Adafruit_SSD1306` combined with `Adafruit_GFX`.
*   **Update Frequency:** 4Hz (250ms interval).
*   **Non-Blocking Updates:** Rendering must occur inside a non-blocking timer loop (using `millis()`) to ensure the M7 can still process DMC-Lite serial packets with minimal latency.

## UI Layout Design (128x64)

### Header (Status Bar)
*   **System State:** Text label (READY, MOVING, E-STOP, UPLOADING).
*   **Connection:** Icon or text indicating if a host (Dragonframe/Unreal) is active.

### Main Body (Motor Status)
*   **Axis Activity:** A grid or list showing the status of up to 8 axes.
    *   Indicator for movement (e.g., small arrows or animated dots).
    *   Position display (optional, based on space).
*   **Camera Trigger:** Visual flash or "CAM" text when the shutter is active.

### Footer (Hardware Info)
*   **Heartbeat:** Small blinking pixel to confirm the M7 loop is alive.
*   **IP Address:** If networking is enabled, scroll the current IP.

## Implementation Phases
1.  **Phase 1: Hardware Validation.** Basic "Hello World" on the OLED using the 3.3V rail.
2.  **Phase 2: Core Integration.** Add the OLED library to the M7 project and implement the 4Hz refresh loop.
3.  **Phase 3: State Mapping.** Map the internal `dmc-lite` state variables to the visual UI elements.
4.  **Phase 4: Optimization.** Ensure I2C transactions do not cause packet drops on the serial interface.
