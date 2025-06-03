/*
 * config.h
 * dmc-lite source code for user configuration
 *
 * Note that all signals are 3.3V TTL
 *
 */

#ifdef ARDUINO_ARCH_MBED_GIGA

#define PIN_STEP1 22
#define PIN_STEP2 24
#define PIN_STEP3 26
#define PIN_STEP4 28
#define PIN_STEP5 30
#define PIN_STEP6 32
#define PIN_STEP7 34
#define PIN_STEP8 36

#define PIN_DIR1  23
#define PIN_DIR2  25
#define PIN_DIR3  27
#define PIN_DIR4  29
#define PIN_DIR5  31
#define PIN_DIR6  33
#define PIN_DIR7  35
#define PIN_DIR8  37

// camera signals should be run through a relay
#define PIN_CAM_METER D52
#define PIN_CAM_SHUTTER D53

#elif defined(ARDUINO_ARCH_MBED_PORTENTA)


#define LAST_ARDUINO_PIN_NUMBER LEDB + 1

#define PIN_STEP1 LAST_ARDUINO_PIN_NUMBER + PC_3 // SPI1_COPI
#define PIN_STEP2 LAST_ARDUINO_PIN_NUMBER + PI_1 // SPI1_CK
#define PIN_STEP3 LAST_ARDUINO_PIN_NUMBER + PG_9 // UART2_RX
#define PIN_STEP4 LAST_ARDUINO_PIN_NUMBER + PI_7 // CAMERA_D3P (CAM D7)
#define PIN_STEP5 LAST_ARDUINO_PIN_NUMBER + PI_4 // CAMERA_D2P (CAM D5)
#define PIN_STEP6 LAST_ARDUINO_PIN_NUMBER + PH_12 // CAMERA_D1P (CAM D3)
#define PIN_STEP7 LAST_ARDUINO_PIN_NUMBER + PH_10 // CAMERA_D0P (CAM D1)
#define PIN_STEP8 LAST_ARDUINO_PIN_NUMBER + PI_5  // CAMERA_CKP (CAM VS)

#define PIN_DIR1  LAST_ARDUINO_PIN_NUMBER + PC_2 // SPI1_CIPO
#define PIN_DIR2  LAST_ARDUINO_PIN_NUMBER + PI_0 // SPI1_CS
#define PIN_DIR3  LAST_ARDUINO_PIN_NUMBER + PG_14 // UART2_TX
#define PIN_DIR4  LAST_ARDUINO_PIN_NUMBER + PI_6  // CAMERA_D3N (CAM D6)
#define PIN_DIR5  LAST_ARDUINO_PIN_NUMBER + PH_14 // CAMERA_D2N (CAM D4)
#define PIN_DIR6  LAST_ARDUINO_PIN_NUMBER + PH_11 // CAMERA_D1N (CAM D2)
#define PIN_DIR7  LAST_ARDUINO_PIN_NUMBER + PH_9  // CAMERA_D0N (CAM D0)
#define PIN_DIR8  LAST_ARDUINO_PIN_NUMBER + PA_6 // CAMERA_CKN (CAM CLK)

// camera signals should be run through a relay
#define PIN_CAM_METER LAST_ARDUINO_PIN_NUMBER + PC_13   // GPIO 0
#define PIN_CAM_SHUTTER LAST_ARDUINO_PIN_NUMBER + PC_15 // GPIO 1

#define LOGIC_OUT_0 LAST_ARDUINO_PIN_NUMBER + PD_4       // GPIO 2
#define LOGIC_OUT_1 LAST_ARDUINO_PIN_NUMBER + PD_5       // GPIO 3

#else

#error "Board not supported for dmc-lite sketch"

#endif

// Stuff for computing times


#ifdef ARDUINO_ARCH_MBED_GIGA
    constexpr uint32_t CPU_FREQ_HZ = 480'000'000;   // System clock (Giga R1 M4)
    constexpr uint32_t TIMER_PRESCALER = 59;        // 480 MHz / (59+1) = 8 MHz
#elif defined(ARDUINO_ARCH_MBED_PORTENTA)
    constexpr uint32_t CPU_FREQ_HZ = 400'000'000;   // System clock (Portenta H7 M4)
    constexpr uint32_t TIMER_PRESCALER = 49;       // 400 MHz / (49+1)  = 8 MHz
#endif

constexpr uint32_t TIMER_TICK_HZ = CPU_FREQ_HZ / (TIMER_PRESCALER + 1); // 8 MHz
constexpr double TIMER_TICK_NS = 1e9 / TIMER_TICK_HZ; // 125 ns per tick

constexpr uint32_t ISR_RATE_HZ = 200'000;       // ISR frequency (every 5 µs)
constexpr double ISR_PERIOD_US = 1e6 / ISR_RATE_HZ;  // 5 µs
constexpr uint32_t TIMER_PERIOD = static_cast<uint32_t>(TIMER_TICK_HZ / ISR_RATE_HZ); // 40 ticks

constexpr uint32_t PULSE_WIDTH_US = 5; // Override with -DPULSE_WIDTH_US=<value>
constexpr uint32_t NOP_COUNT = static_cast<uint32_t>((PULSE_WIDTH_US * 1000) / TIMER_TICK_NS); // for pulse delay loop

// Outer loop tick: number of ISR cycles per outer loop tick
constexpr uint32_t OUTER_LOOP_TICKS = 10; // Adjustable: number of ISR cycles per outer loop tick
constexpr double SPEED_SCALE = static_cast<double>(1ULL << 32) / (double)ISR_RATE_HZ;

