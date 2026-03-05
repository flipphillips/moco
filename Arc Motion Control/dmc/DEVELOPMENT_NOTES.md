# Development Notes - DMC Project

## Crucial Stuff

* The directory `dmcDIST` has the default (and pretty much functional) Dragonframe code. 
* This code should act as reference for everyting in the `src` directory.
* It should not be modified.
* The minimal changes we're working on:
	* We want to change the system to 'pull down' instead of up.
	* We want to add some hardware and software debugging. Mostly hardware in m4 and software in m7 due to timing.
	* We'd like to change the pulse widths that are present in the `dmcDIST`.
	* THIS IS PRESENTLY BROKEN - Bursts of pulses come out instead of well spaced ones. 
	* I suspect the `nop` loop but might be red-herring.
* Going forward:
	* Let's get the pulse trains corrected by reverting as close as possible to `dmcDIST` for the pulse algorithm and timing.
	* Let's add in the debugging (it doesn't seem to be working presently).

*** The reset of this document is more 'background' than imperative stuff, consult but don't take as gospel, esp re: timing, etc ***


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
In your Arduino code, use `Serial1` for UART communication:
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

4. How This Relates to Bresenham

Like Bresenham’s algorithm, this method uses integer math and an accumulator to decide when to "step."

The accumulator’s overflow (or bit pattern change) triggers the next action, ensuring accurate, evenly spaced pulses.

5. Key Advantages

No floating-point math: Efficient for microcontrollers.

Smooth, high-resolution stepping: Sub-step accuracy.

Consistent timing: Independent of step rate.

Handles direction changes: Works for both positive and negative speeds.

## Pulse Math

The timer math works like this - 

```c
TIMER_TICK_HZ: 4000000
TIMER_TICK_NS: 250.00
ISR_RATE_HZ: 200000
ISR_PERIOD_US: 5.00
TIMER_PERIOD: 20
PULSE_WIDTH_US: 1
NOP_COUNT: 4
OUTER_LOOP_TICKS: 10
SPEED_SCALE: 0
```

BUT THIS MAY BE INCORRECT. There are timing problems.