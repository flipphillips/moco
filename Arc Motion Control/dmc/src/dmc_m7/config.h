/*
 * config.h
 * dmc-lite source code for user configuration
 *
 * Note that all signals are 3.3V TTL
 *
 */

#define DEBUG


#define HWDEBUG
#undef HWDEBUG

#ifdef HWDEBUG
#define PIN_TEST D40
#define TOGGLE_PIN(PORT, PIN) (PORT->ODR ^= (1 << PIN))
#endif // HWDEBUG

#if defined(DEBUG)
#define DEBUG_SERIAL Serial1
#endif // DEBUG

#ifdef ARDUINO_ARCH_MBED_GIGA

#define LOGIC_OUT_0 D40
#define LOGIC_OUT_1 D41

// set a pin for an e-stop switch. uses pull-up resistor, so switch needs to connect pin to ground.
#define KILL_SWITCH_PIN  D48

// set a pin for a logic switch input. uses pull-up resistor, so switch needs to connect pin to ground.
// #define LOGIC_SWITCH_PIN  D49

// display
#define OLED_SDA D20
#define OLED_SCL D21

#elif defined(ARDUINO_ARCH_MBED_PORTENTA)


#define LAST_ARDUINO_PIN_NUMBER LEDB + 1


#define LOGIC_OUT_0 LAST_ARDUINO_PIN_NUMBER + PD_4       // GPIO 2
#define LOGIC_OUT_1 LAST_ARDUINO_PIN_NUMBER + PD_5       // GPIO 3

// set a pin for an e-stop switch
//#define KILL_SWITCH_PIN  LAST_ARDUINO_PIN_NUMBER + PE_3  // GPIO 4

// set a pin for a logic switch input
//#define LOGIC_SWITCH_PIN  LAST_ARDUINO_PIN_NUMBER + PG_3  // GPIO 5

#else

#error "Board not supported for dmc-lite sketch"

#endif
