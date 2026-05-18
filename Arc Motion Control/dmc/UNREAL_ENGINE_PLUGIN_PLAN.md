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


 yep                                                                                           
▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  Responding with gemini-3-flash-preview
✦ I've completed the implementation and build of the DMCLite Unreal Engine plugin. You're all set
  to launch the Coppola_MoCo_Rig project and begin testing.

  Final Verification Checklist:
   * M7 Core: Running and ready to accept serial commands (Blue LED heartbeat active).
   * OLED: Displaying system state (Hardware fix confirmed).
   * Unreal Plugin: Updated with Blueprint events (OnHandshakeReceived, etc.) and pre-built for
     Linux.

  If you encounter any issues with port permissions when Unreal tries to open /dev/ttyACM0, you
  might need to run:

   sudo usermod -a -G dialout $USER
  (You'll need to log out and back in for this to take effect, or just sudo chmod 666
  /dev/ttyACM0 for a temporary fix).

  Enjoy the rig! Let me know if you want to tweak the motion control logic or add more UI
  features once you've had a chance to play with it in Unreal.
▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 > one small gotcha - I'm not familiar with the latest versionns of UE Editor, so I need a       
   -little- more hand holding                                                                    
▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  Responding with gemini-3-flash-preview

  Unreal Editor Testing Guide: 
  Provide a step-by-step guide for testing the DMCLite plugin in the Unreal Engine 5.7
  Editor.

╭───────────────────────────────────────────────────────────────────────────────────────────╮
│ ✓  Shell /home/local/UE/Engine/Binaries/Linux/UnrealEditor "/home/flip/GitHub/moco/Unrea… │
│                                                                                           │
│ Command moved to background (PID: 23792). Output hidden. Press Ctrl+B to view.            │
╰───────────────────────────────────────────────────────────────────────────────────────────╯
  ✓  ReadFile  UNREAL_ENGINE_PLUGIN_PLAN.md

✦ No problem at all! Unreal 5.7 has a few layout changes, but the core workflow remains the same.
  I've just launched the editor for you in the background.

  Here is your "Hand-Holding Guide" to testing the plugin:

  1. Verify the Plugin is Active
  Once the project opens:
   * Go to the top menu: Edit > Plugins.
   * Search for "DMCLite".
   * Ensure the checkbox is Enabled. (If you just built it, it should be green).

  2. Find and Place the DMCActor
  The DMCActor is the "brain" that talks to your Arduino.
   * Open the Content Drawer (usually Ctrl + Space or a button at the bottom left).
   * Go to All > Plugins > DMCLite Content (or search for DMCActor in the search bar).
   * Drag the DMCActor into your 3D viewport.

  3. Configure the Connection
  With the DMCActor selected in the viewport:
   * Look at the Details Panel (usually on the right side).
   * Scroll down to the DMCLite section.
   * Port Name: Enter /dev/ttyACM0 (or whatever your Arduino shows up as).
   * Baud Rate: Ensure it is 115200.

  4. Test the Handshake (The "Magic" Moment)
   * Press Play (or the Green Arrow icon at the top) to start the simulation.
   * While the game is running, select the DMCActor in the "Outliner" (top right).
   * In the Details Panel, you will see buttons under the "DMCLite" category:
       * Click Send Handshake.
   * Check the Output Log: (Menu: Window > Output Log). 
       * If everything is working, you should see: DMC: Handshake SUCCESS! Hardware replied to
         HI.
       * On the physical hardware, your OLED should reflect the connection, and the Blue LED will
         react to the data burst.

  5. Using the New Events (Advanced)
  If you open the Level Blueprint (the icon that looks like a graph at the top), you can select
  the DMCActor and right-click to add Events:
   * Search for On Handshake Received.
   * You can connect this to a Print String or a UI element to confirm the rig is "Alive"
     visually in your game.

  Pro-Tip: If the buttons in the Details Panel don't appear, make sure you are looking at the
  DMCActor and that you haven't filtered the details view.

  I'll stay on standby while you check the Editor!