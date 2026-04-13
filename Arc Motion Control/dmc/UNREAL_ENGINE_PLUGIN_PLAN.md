# Unreal Engine Plugin Plan - DMC Project

## Goal
Build a dedicated Unreal Engine C++ plugin to enable the rig as a native motion control device for Virtual Production (VP) and Pre-visualization. The plugin will act as a "Host" to the Arduino-based DMC-Lite controller.

## Protocol: DMC-Lite Binary
Unlike the text-based DFMoco protocol, the DMC-Lite protocol is a performance-optimized binary protocol.

### Packet Structure (10-byte Header)
| Offset | Field | Size | Description |
|---|---|---|---|
| 0 | Sync | 2 bytes | Literal chars 'D' 'F' |
| 2 | ID | 4 bytes | 32-bit unique message ID (Little-endian) |
| 6 | Type | 2 bytes | 16-bit Command Type (Little-endian) |
| 8 | Length | 2 bytes | 16-bit Data Payload Length (Little-endian) |
| 10+ | Payload | variable | The command-specific binary data |
| End | Checksum| 2 bytes | 16-bit Fletcher-style checksum |

### Key Command IDs
*   `0x0100`: `DMC_MSG_RT_UPLOAD_MOVE_BEGIN`
*   `0x0101`: `DMC_MSG_RT_UPLOAD_MOVE_AXIS` - Uploads curve data for a single axis.
*   `0x0111`: `DMC_MSG_RT_RUN_MOVE` - Triggers the hardware to execute the uploaded move.
*   `0x0034`: `DMC_MSG_MOTOR_GET_POSITION` - Real-time position query.

## Plugin Architecture

### 1. Core Serial Driver (`FDMCSerialHandler`)
*   Manages the low-level asynchronous serial port connection (`COMx` or `/dev/ttyACM0`).
*   Handles packet framing (detecting 'DF'), checksum calculation, and retry logic.

### 2. Rig Asset Actor (`ADMCRigActor`)
*   An Unreal Actor that represents the physical rig in the viewport.
*   Contains 8 customizable components corresponding to the physical axes.
*   Visualizes the current position received from the rig during rehearsal.

### 3. Sequencer Integration
*   **Sequencer Track:** A custom track type to record and play back rig movement.
*   **Curve Sampler:** A utility to sample Unreal's Bezier curves and convert them into the discrete steps required by the `DMC_MSG_RT_UPLOAD_MOVE_AXIS` command.

## "Load and Go" Workflow (Primary)
1.  **Preparation:** User designs a camera move in the Unreal Sequencer.
2.  **Sampling:** The plugin samples the curve at the target frame rate (e.g., 24fps).
3.  **Upload:** Binary packets are streamed to the Arduino via `DMC_MSG_RT_UPLOAD_MOVE_AXIS`. The Arduino stores these in its local buffer.
4.  **Execution:** On the "Go" signal (`DMC_MSG_RT_RUN_MOVE`), the Arduino takes control of timing, ensuring perfectly smooth pulses and synchronized camera triggers.

## Implementation Phases
1.  **Phase 1: Binary Serial Bridge.** Basic C++ implementation of the DMC-Lite packet structure in Unreal.
2.  **Phase 2: Live Viewport Link.** Allow the hardware to drive the virtual camera in Unreal (Pre-viz mode).
3.  **Phase 3: Sequencer Upload Utility.** Sampling Unreal curves and uploading them as raw step data to the hardware.
4.  **Phase 4: Sync & Trigger.** Implementing frame-accurate triggers and "Go" signal coordination.
