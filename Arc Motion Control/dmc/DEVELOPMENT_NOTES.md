# Development Notes - DMC Project

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

### ON WASTING TIME WITH GENERATIVE AI

- I spent _two hours_ trying to track the above down with ChatGPT. 
- We failed to ever get anything working. 
- It kept trying to solve the massive big picture here instead of breaking the task down.
- _It refused to fail_
- I had to keep guiding it back to the basic problem, simple examples, etc.
- After two hours I became frustrated and gave up.
- I went to Google and MS Copilot and asked about the problem. 
- It told me that it was NOT POSSIBLE.
- ChatGPT NEVER CHECKED.
