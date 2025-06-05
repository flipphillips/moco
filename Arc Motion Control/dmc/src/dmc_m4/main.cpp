/*
 * dmc_m4.ino
 * dmc-lite source code
 * Copyright 2023 by DZED Systems LLC
 *
 * Pulse width mods -
 * Copyright 2025 by Flip Philips
 * RITMPS
 * 
 * Target core: M4 Co-processor
 * Flash split: 1.5MB M7 + 0.5MB M4
 */

#include <Arduino.h>
#include <RPC.h>
#include <pinDefinitions.h>
#include "config.h"

#define CAMERA_OFF 0x0
#define CAMERA_SHUTTER 0x1
#define CAMERA_METER 0x2

#undef DEBUG
#define DEBUG

#undef HWDEBUG
#define HWDEBUG

#ifdef HWDEBUG
#define PIN_TEST D51
#define TOGGLE_PIN(PORT, PIN) (PORT->ODR ^= (1 << PIN))
#endif // HWDEBUG

#if defined(DEBUG)
#define DEBUG_SERIAL Serial1
#endif // DEBUG

#ifdef CORE_CM7
#error "Make sure to target the M4 Co-processor with flash split 1.5MB M7 + 0.5MB M4"
#endif

#define MOTOR_COUNT 8
#define MOTOR_CAM_COUNT 9

static GPIO_TypeDef *const port_table[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK};
static const uint16_t mask_table[] = {
    1 << 0, 1 << 1, 1 << 2,  1 << 3,  1 << 4,  1 << 5,  1 << 6,  1 << 7,
    1 << 8, 1 << 9, 1 << 10, 1 << 11, 1 << 12, 1 << 13, 1 << 14, 1 << 15
  };

static inline void digitalWriteFast(pin_size_t pin, PinStatus val)
{
  PinName pin_name = g_APinDescription[pin].name;
  uint16_t mask = mask_table[pin_name & 0xf];
  GPIO_TypeDef *const port = port_table[pin_name >> 4];
  if (val)
    port->BSRR = mask;
  else
    port->BSRR = (uint32_t)(mask << 16);
}

// This version you takes in a pin name (PinName) like LED_RED
static inline void digitalWriteFast(PinName pin_name, PinStatus val) __attribute__((always_inline, unused));
static inline void digitalWriteFast(PinName pin_name, PinStatus val)
{
  uint16_t mask = mask_table[pin_name & 0xf];
  GPIO_TypeDef *const port = port_table[pin_name >> 4];
  if (val)
    port->BSRR = mask;
  else
    port->BSRR = (uint32_t)(mask << 16);
}

int64_t speed[MOTOR_CAM_COUNT];
uint8_t cameraValue;
uint16_t cameraOpenAngle;
uint16_t cameraCloseAngle;

struct DmcSharedData
{
  volatile uint32_t motorDataLoaded;
  volatile int64_t nextSpeed[MOTOR_CAM_COUNT];
  volatile int64_t accum[MOTOR_CAM_COUNT];
  volatile uint32_t motorDirection;
  volatile uint8_t cameraValue;
  volatile uint16_t cameraOpenAngle;
  volatile uint16_t cameraCloseAngle;
};

DmcSharedData *sharedDataPtr;

// special fast timer TIM1
TIM_HandleTypeDef htim1;

static const int stepPins[] = { PIN_STEP1, PIN_STEP2, PIN_STEP3, PIN_STEP4, PIN_STEP5, PIN_STEP6, PIN_STEP7, PIN_STEP8 };
static const int dirPins[] = { PIN_DIR1, PIN_DIR2, PIN_DIR3, PIN_DIR4, PIN_DIR5, PIN_DIR6, PIN_DIR7, PIN_DIR8 };

inline int64_t stepsPerSecondToSpeed(double stepsPerSec) {
    return static_cast<int64_t>(stepsPerSec * SPEED_SCALE / ISR_RATE_HZ);
}

inline double speedToStepsPerSecond(int64_t speed) {
    return static_cast<double>(speed) * ISR_RATE_HZ / SPEED_SCALE;
}

#ifdef DEBUG
volatile uint8_t debugHead = 0;
volatile uint8_t debugTail = 0;
const int DEBUG_BUF_SIZE = 64;
struct DebugMsg {
  uint8_t motor;
  int64_t speed;
  uint32_t timestamp;
};
volatile DebugMsg debugBuffer[DEBUG_BUF_SIZE];

void enqueueDebug(uint8_t motor, int64_t spd, uint32_t timestamp) {
  uint8_t nextHead = (debugHead + 1) % DEBUG_BUF_SIZE;
  if (nextHead != debugTail) {
    debugBuffer[debugHead].motor = motor;
    debugBuffer[debugHead].speed = spd;
    debugBuffer[debugHead].timestamp = timestamp;
    debugHead = nextHead;
  }
}
#endif // DEBUG

void setup()
{
  RPC.begin();

  cameraValue = 0;
  sharedDataPtr = 0;
  sharedDataPtr = (DmcSharedData *)0x3800fd00;

  delay(100);
  while (!sharedDataPtr)
  {
    sharedDataPtr = (DmcSharedData *)RPC.call("sharedDataPointer").as<uintptr_t>();
    if (sharedDataPtr)
      break;
    delay(100);
  }

  pinMode(LEDG, OUTPUT);
  for (int i = 0; i < 3; ++i)
  {
    digitalWrite(LEDG, LOW);
    delay(200);
    digitalWrite(LEDG, HIGH);
    delay(200);
  }

  for (int i = 0; i < MOTOR_COUNT; ++i)
  {
    sharedDataPtr->accum[i] = 0;
    speed[i] = 0;
    int stepPin = stepPins[i];
    int dirPin = dirPins[i];
    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);

    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW);
  }
  sharedDataPtr->accum[MOTOR_COUNT] = 0;
  speed[MOTOR_COUNT] = 0;

  // set up camera
  pinMode(PIN_CAM_METER, OUTPUT);
  digitalWrite(PIN_CAM_METER, LOW);
  pinMode(PIN_CAM_SHUTTER, OUTPUT);
  digitalWrite(PIN_CAM_SHUTTER, LOW);

  // Timer setup - see constants in config.h
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIMER_PRESCALER;
  htim1.Init.Period = TIMER_PERIOD - 1; // 20 ticks at 4 MHz

  __HAL_RCC_TIM1_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  HAL_TIM_Base_Init(&htim1);
  HAL_TIM_Base_Start_IT(&htim1);

  // debuggery
  #ifdef HWDEBUG
  pinMode(PIN_TEST, OUTPUT);
  digitalWrite(PIN_TEST, LOW);  
  #endif // HWDEBUG

  #if defined(DEBUG)
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {
    ; // Wait for serial monitor to open
  }
  DEBUG_SERIAL.println("M4 USB Serial active");
  DEBUG_SERIAL.println("dmc_m4 started");

  DEBUG_SERIAL.println("TIMER_TICK_HZ: " + String(TIMER_TICK_HZ));
  DEBUG_SERIAL.println("TIMER_TICK_NS: " + String(TIMER_TICK_NS));
  DEBUG_SERIAL.println("ISR_RATE_HZ: " + String(ISR_RATE_HZ));
  DEBUG_SERIAL.println("ISR_PERIOD_US: " + String(ISR_PERIOD_US));
  DEBUG_SERIAL.println("TIMER_PERIOD: " + String(TIMER_PERIOD));
  DEBUG_SERIAL.println("PULSE_WIDTH_US: " + String(PULSE_WIDTH_US));
  DEBUG_SERIAL.println("NOP_COUNT: " + String(NOP_COUNT));
  DEBUG_SERIAL.println("OUTER_LOOP_TICKS: " + String(OUTER_LOOP_TICKS));
  DEBUG_SERIAL.println("SPEED_SCALE: " + String((unsigned long)SPEED_SCALE, 10));

  #endif
}

void loop()
{
  while (1)
  {
    #ifdef DEBUG
    // while (debugTail != debugHead) {
    //   DebugMsg msg = *((DebugMsg*)&debugBuffer[debugTail]);
    //   debugTail = (debugTail + 1) % DEBUG_BUF_SIZE;
    //   DEBUG_SERIAL.print(msg.timestamp);
    //   DEBUG_SERIAL.print(", ");
    //   DEBUG_SERIAL.print(msg.motor);
    //   DEBUG_SERIAL.print(", ");
    //   DEBUG_SERIAL.print(msg.speed);
    //   DEBUG_SERIAL.print("\n");
    // }
    DEBUG_SERIAL.print("nextSpeed[0]: ");
    DEBUG_SERIAL.println((long long)sharedDataPtr->nextSpeed[0]);
    DEBUG_SERIAL.print("steps/sec: ");
    DEBUG_SERIAL.println(speedToStepsPerSecond(sharedDataPtr->nextSpeed[0]));
    #endif // DEBUG

    delay(100);
  }
}

extern "C"
{
  void TIM1_UP_IRQHandler(void)
  {
    HAL_TIM_IRQHandler(&htim1);
  }
}

uint32_t counter = 0;
uint32_t ledCounter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    #ifdef HWDEBUG
    //TOGGLE_PIN(GPIOE, 5); // toggle test pin once per ISR call - E5 = D51
    #endif

    //
    // outer loop - update things
    // called every OUTER_LOOP_INTERVAL_MS ms
    //
    if (++counter == OUTER_LOOP_TICKS)
    {
      // led blinkinlights + counter reset
      ++ledCounter;
      if (ledCounter == 90)
      {
        digitalWrite(LEDG, LOW);
      }
      else if (ledCounter == 100)
      {
        ledCounter = 0;
        digitalWrite(LEDG, HIGH);
      }
      counter = 0;

      // shutter
      cameraOpenAngle = sharedDataPtr->cameraOpenAngle;
      cameraCloseAngle = sharedDataPtr->cameraCloseAngle;
      speed[MOTOR_COUNT] = sharedDataPtr->nextSpeed[MOTOR_COUNT]; // camera

      //
      // outer loop - update motor directions and speeds
      //
      for (int i = 0; i < MOTOR_COUNT; ++i)
      {
        digitalWriteFast(dirPins[i], ((1 << i) & sharedDataPtr->motorDirection) ? HIGH : LOW);

        int64_t prevSpeed = speed[i];
        speed[i] = sharedDataPtr->nextSpeed[i];

        // ensure that we land with the pulse off
        if (prevSpeed && !speed[i] && (sharedDataPtr->accum[i] & 0xFFFFFFFF))
        {
          int32_t before = ((sharedDataPtr->accum[i] >> 31) ^ (sharedDataPtr->accum[i] >> 30)) & 0x1;
          if (before)
          {
            int64_t accum = sharedDataPtr->accum[i];
            bool negative = (accum < 0);
            if (negative)
            {
              accum = -accum;
              prevSpeed = -prevSpeed;
            }
            uint8_t dirSection = (accum >> 30) & 0x3;
            accum = (accum & 0xFFFFFFFF00000000UL);
            if ((prevSpeed > 0 && dirSection) || (prevSpeed < 0 && dirSection == 3))
            {
              accum += (1LL << 32);
            }
            if (negative)
              accum = -accum;
            sharedDataPtr->accum[i] = accum;
            int32_t after = ((sharedDataPtr->accum[i] >> 31) ^ (sharedDataPtr->accum[i] >> 30)) & 0x1;
            
            // I belive this is sort of a 'safety off' thing.
            if (before != after)
              digitalWriteFast(stepPins[i], after ? HIGH : LOW);
          }
        }
      }
      sharedDataPtr->motorDataLoaded = 0;
    }

    // This is the fast inner loop
    // ISR 
    // Called every 5 µs (200 kHz)
    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
      int32_t before = ((sharedDataPtr->accum[i] >> 31) ^ (sharedDataPtr->accum[i] >> 30)) & 0x1;
      sharedDataPtr->accum[i] += speed[i];
      int32_t after = ((sharedDataPtr->accum[i] >> 31) ^ (sharedDataPtr->accum[i] >> 30)) & 0x1;
      // send a PULSE_WIDTH_US pulse only on 0->1 transition
      if (before == 0 && after == 1) {
        #ifdef HWDEBUG
        TOGGLE_PIN(GPIOE, 5); // toggle test pin once per ISR call - E5 = D51
        #endif // HWDEBUG
        #ifdef DEBUG
        enqueueDebug(i, speedToStepsPerSecond(speed[i]), micros());
        #endif // DEBUG
        
        digitalWriteFast(stepPins[i], HIGH);

        // NOP-based delay loop to approximate PULSE_WIDTH_US duration
        // Avoids using delayMicroseconds() inside ISR
        for (volatile uint32_t d = 0; d < NOP_COUNT; ++d) {
          __asm__ __volatile__("nop");
        }
        
        digitalWriteFast(stepPins[i], LOW);

      }
      else if (before == 1 && after == 0) {
        // This is a 1->0 transition, we don't do anything here
        // but we could if we wanted to
      }
      else {
        // This is a no-op, we don't do anything here
        // but we could if we wanted to     
      }
    }

    uint8_t nextCameraPosition;
    if (speed[MOTOR_COUNT])
    {
      sharedDataPtr->accum[MOTOR_COUNT] += speed[MOTOR_COUNT];
      uint32_t camPos = (sharedDataPtr->accum[MOTOR_COUNT] >> 32) & 0xFFF;
      nextCameraPosition = (camPos >= cameraOpenAngle && camPos <= cameraCloseAngle) ? 0x3 : 0x0;
    }
    else
    {
      nextCameraPosition = sharedDataPtr->cameraValue;
    }

    if (cameraValue != nextCameraPosition)
    {
      cameraValue = nextCameraPosition;
      digitalWrite(PIN_CAM_METER, ((cameraValue)&CAMERA_METER) ? HIGH : LOW);
      digitalWrite(PIN_CAM_SHUTTER, ((cameraValue)&CAMERA_SHUTTER) ? HIGH : LOW);
    }
  }
}
