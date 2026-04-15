# Development Notes - DMC Project

## System Status & Current Goals

* The directory `dmcDIST` has the default (and pretty much functional) Dragonframe code. This code should act as reference for everyting in the `src` directory.
* The minimal changes we're working on:
	* We want to change the system to 'pull down' instead of up to support the Kuper motion control cards.
	* We want to add some hardware and software debugging. Mostly hardware in m4 and software in m7 due to timing.
* **FIXED (2026-03-05):** The "bursty" pulse train issue has been resolved.
	* The cause was a combination of a custom `nop`-based pulse-width loop and an incorrect `OUTER_LOOP_TICKS` value in the M4 code.
	* The fix involved reverting the pulse generation algorithm and timing constants in `src/dmc_m4/` to match the simpler, more stable implementation from the `dmcDIST` reference code.
* **Hardware Driver Info:**
	* **Interface Type:** Common Anode (Opto-isolated). The motor controllers provide a shared +5V (Anode) to the step/direction pins. We "sink" current to Ground to trigger a pulse.
	* **Logic Level:** Active-Low (Falling Edge). The software is configured with `INVERT_STEP_PULSE = true` in `config.h`, meaning the pins are HIGH (3.3V/5V) when idle and pulse LOW (0V) to step.
	* **Buffer IC:** **SN74AHCT541N** powered by **5V Vcc**. 
		* The "T" (TTL) inputs safely accept the Arduino's 3.3V signals.
		* The 5V Vcc ensures the buffer outputs a full 5.0V when HIGH, matching the motor controller's internal rail and ensuring the opto-isolators are 100% OFF.
	* **Board Modification:** External pull-up resistor packs have been removed. The AHCT541 actively drives the lines, and the motor controller's internal pull-up (via the opto LED) provides the reference voltage.
	* **Legacy Support:** This configuration mimics the "Kuper" behavior (falling edge trigger) while safely interfacing the 3.3V Arduino with 5V industrial motor controllers.

## Current Plans & Observations

### 1. Local OLED Status Display (M7 Core) - **IMPLEMENTED (2026-04-14)**
*   **Goal:** Add an I2C OLED (SSD1306) to show high-level system state (READY, MOVING, E-STOP), motor activity, and camera triggers.
*   **Hardware Constraints:** The Arduino Giga R1 is strictly **3.3V logic**. The OLED *must* be powered via the 3.3V pin. Using 5V will damage the D20 (SDA) and D21 (SCL) pins due to the display's onboard pull-up resistors.
*   **Implementation:**
    *   **Library:** `Adafruit SSD1306` and `Adafruit GFX`.
    *   **Architecture:** Rendering restricted to M7 core at 4Hz (250ms interval) using a non-blocking `millis()` loop.
    *   **UI Layout:** 
        *   **Header:** System state (READY/MOVING/E-STOP) and "CAM" indicator.
        *   **Body:** 8-axis activity indicators (`^` for forward, `v` for reverse, `-` for idle).
        *   **Footer:** Heartbeat pixel/character (`*`) to confirm M7 loop health.
*   **Validation:** Verified via `pio run -e giga_r1_m7`. I2C transactions are handled by the M7 to prevent interference with M4 step timing.
### 2. Unreal Engine Plugin (DMCLite) - **IMPLEMENTED (2026-04-15)**
*   **Goal:** Build a high-performance C++ plugin to allow direct control of the rig from Unreal Engine (Coppola MoCo Rig project).
*   **Architecture:** 
    *   **Asynchronous Serial:** Uses a dedicated background thread (`FDMCSerialWorker`) to handle non-blocking I/O at 1ms poll rates.
    *   **Protocol:** Implements the full DMC-Lite binary protocol (Sync: 'DF', 10-byte header, Fletcher-16 zero-sum checksum).
    *   **Cross-Platform:** Support for Mac, Linux, and Windows using native serial APIs (termios/Windows COM).
*   **Key Features:**
    *   **Serial Port Discovery:** Static Blueprint function `GetAvailableSerialPorts()` to automatically list `/dev/tty.*` (Mac), `/dev/ttyUSB*` (Linux), and `REG_SZ` COM ports (Windows).
    *   **Handshake Actor:** `ADMCActor` provides a drop-in testing interface to verify the `HI` command handshake with hardware.
    *   **Blueprint Ready:** All core functions (`Connect`, `Disconnect`, `SendCommand`) are exposed to Blueprints. `MessageID` uses `int32` for BP compatibility.
*   **Build System:** 
    *   Integrated into `Unreal/Coppola_MoCo_Rig/Plugins/DMCLite`.
    *   `build_dmclite.sh` script provided for one-click rebuilding on Mac/Linux.

*** The reset of this document is more 'background' than imperative stuff, consult but don't take as gospel, esp re: timing, etc ***

## Git Best Practices (Unreal Integration)

The project now uses a professional Unreal-Git hybrid structure:
1. **Git LFS:** Mandatory for `.uasset` and `.umap` files. Check `.gitattributes` for details.
2. **Surgical .gitignore:** Blanket ignores for `Unreal/Coppola_MoCo_Rig/` have been removed. We now explicitly track the `Config/`, `Content/`, and `Source/` directories while ignoring `Saved/`, `Intermediate/`, `Binaries/`, and `DerivedDataCache/`.
3. **Plugin Portability:** The `DMCLite` plugin is project-local, ensuring the rig is self-contained.

## PlatformIO Configuration

### Port Configuration
This project uses separate ports for different purposes:

- **Upload Port**: `/dev/ttyACM0` (Arduino Giga R1 main USB port)
- **Monitor Port**: `/dev/ttyUSB0` (External UART-to-USB adapter)

### Why Separate Ports?
The main device port (`/dev/ttyACM0`) is used by the application software, so monitoring must use a different port to avoid conflicts.

### UART-to-USB Adapter Wiring
For Arduino Giga R1 with external UART adapter:

```
UART-to-USB Adapter    →    Arduino Giga R1
─────────────────────       ───────────────
BLACK (GND)           →    GND
GREEN (TX)            →    Pin 13 (RX1)  
WHITE (RX)            →    Pin 14 (TX1)
RED (VCC)             →    **DO NOT CONNECT**
```

**Important**: Don't connect VCC - the Arduino powers itself via USB.

### Platform-Specific Port Names
- **Linux**: `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyACM0`
- **macOS**: `/dev/cu.usbserial-*`, `/dev/cu.usbmodem-*`
- **Windows**: `COM3`, `COM4`, etc.

### VS Code PlatformIO Extension Bug
The VS Code PlatformIO extension's monitor button (plug icon) has a known bug where it ignores the `monitor_port` setting in `platformio.ini` and uses auto-detection instead.

**Workaround**: Use manual command in PlatformIO Core CLI:
```bash
pio device monitor -e giga_r1_m7
```

### Code Configuration
In  Arduino code, use `Serial1` for UART communication:
```cpp
void setup() {
  Serial1.begin(115200);  // UART adapter communication
  Serial.begin(115200);   // USB serial for debugging (if needed)
}
```

## Git History
This configuration was established on July 3, 2025, after troubleshooting port configuration issues across different development platforms.

## Keybinding Chaos

There are strange keybinding problems with VSCode here in the Linux-verse. 

```json
[
	{
        "key": "f10",
		"command": "workbench.action.terminal.newWithCwd",
		"args": {
			"cwd": "${workspaceFolder}/Arc Motion Control/dmc"
		}
	},
	{
        "key": "f9",
		"command": "workbench.action.terminal.sendSequence",
		"args": {
			"text": "source ~/.platformio/penv/bin/activate && pio device monitor -e giga_r1_m7 --port /dev/ttyUSB0\n"
		}
	}
]
```

## Algorithm Explainer

1. Accumulator Structure

The accumulator (accum[i]) is a 64-bit fixed-point value:

| Bits        | Purpose             |
|-------------|---------------------|
| 63 ... 32   | Integer position    |
| 31 ... 0    | Fractional position |

Each cycle, `speed[i]` (a fixed-point increment) is added to `accum[i]`, smoothly advancing the position.

2. Bit Detection Pattern

The key expression:

```text
((accum[i] >> 31) ^ (accum[i] >> 30)) & 0x1
```

- Extracts bits 31 and 30 (the two highest bits of the 32-bit fractional part).
- XORs them to create a quadrature-like signal.
- This bit toggles as the accumulator crosses quarter and half-step boundaries.


3. Timing Diagram
Below is a conceptual timing diagram showing how the accumulator and bit pattern interact to generate step pulses:

```text
Accumulator Value (Fractional bits 31...0):
|----|----|----|----|----|----|----|----|
0x00000000                       0xFFFFFFFF

Bit 31:  |0...............|1...............|
Bit 30:  |00......|11......|00......|11....|
XOR:     |0|1|0|1|0|1|0|1|0|1|0|1|0|1|0|1|

Step Pulse Output:
        __    __    __    __
_______|  |__|  |__|  |__|  |____
    (output toggles on XOR change)
```

As the accumulator increments, bits 31 and 30 toggle at different rates.

The XOR pattern changes state every time the accumulator crosses a quarter of its range.

The algorithm detects this change and toggles the step pin, creating a precise step pulse.

5. Key Advantages

No floating-point math: Efficient for microcontrollers.

Smooth, high-resolution stepping: Sub-step accuracy.

Consistent timing: Independent of step rate.

Handles direction changes: Works for both positive and negative speeds.
