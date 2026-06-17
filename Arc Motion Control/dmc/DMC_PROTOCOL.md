# DMC (Dragonframe Motion Control) Protocol Documentation

The DMC protocol is a high-performance, binary, little-endian protocol used by Dragonframe to communicate with professional motion control hardware and newer DIY firmware like `dmc-lite`.

### **1. Packet Structure**
Every message starts with a 10-byte header, followed by an optional payload and a 2-byte checksum.

| Offset | Type | Field | Description |
| :--- | :--- | :--- | :--- |
| 0 | `char` | **Sync 1** | Always `'D'` |
| 1 | `char` | **Sync 2** | Always `'F'` |
| 2 | `uint32_t` | **Message ID** | Arbitrary ID for tracking responses/ACKs. |
| 6 | `uint16_t` | **Type** | Command or Response type (see below). |
| 8 | `uint16_t` | **Length** | Number of bytes in the payload. |
| 10 | `byte[]` | **Payload** | Data bytes (specific to command). |
| 10 + N | `uint16_t` | **Checksum** | Fletcher-16 based checksum. |

### **2. Checksum Algorithm**
The checksum is calculated over the entire packet (Header + Payload). When sending, the device calculates the checksum and appends two bytes (`c0`, `c1`) such that the total checksum of the resulting packet is zero.

### **3. Common Command Types**
Responses usually have the `DMC_MSG_FLAG_ACK` (`0x8000`) bit set.

| Type | Command | Payload |
| :--- | :--- | :--- |
| `0x0001` | **HI (Handshake)** | Returns capabilities (`uint16`), axes (`uint8`), and version. |
| `0x0031` | **MOTOR_MOVE** | `uint8` index, `int32` target position (steps). |
| `0x0032` | **MOTOR_STOP** | `uint8` index. |
| `0x0034` | **GET_POSITION** | Triggers a broadcast of current motor positions. |
| `0x0030` | **MOTOR_STATUS** | Returns a `uint32` bitmask of moving motors. |
| `0x0038` | **SET_SPEED** | `uint8` index, `uint32` max velocity, `uint32` max acceleration. |
| `0x0021` | **GIO_OUT** | `uint32` bitmask for logic/trigger outputs. |

### **4. Real-Time (RT) Playback**
For complex keyframe moves, the protocol supports streaming path data:
- **`0x0100` (RT_UPLOAD_MOVE_BEGIN):** Sets start frame and frame count.
- **`0x0101` (RT_UPLOAD_MOVE_AXIS):** Streams a chunk of point-to-point step data for a specific axis.
- **`0x0111` (RT_RUN_MOVE):** Executes the synchronized move.

---

### **DMC (Binary) vs. DFMoco (ASCII)**
While this **DMC Binary** protocol is used for precision and high axis counts, Dragonframe also supports a simpler **DFMoco ASCII** protocol (version 2.0.0):
- **Baud:** 57600
- **Format:** Simple text like `mm 1 250\r\n` (Move Motor 1 to position 250).
- **Usage:** Primarily for basic Arduino Uno/Mega-based stepper setups.

`dmc-lite` (for Giga R1 / Portenta H7) uses the **DMC Binary** protocol for its advanced multi-axis and real-time features.
