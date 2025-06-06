#define DFMOCO_VERSION 2
#define DFMOCO_VERSION_STRING "2.1.0"

/*
  DFMoco version 2.1.0
  
  Multi-axis motion control.
  For use with the Arc motion control system in Dragonframe 5 or newer.
  Generates step and direction signals, which can be sent to stepper motor drivers.
   
  Control up to four axes with an Uno, Duemilanove or 101 board.
  Control up to eight axes with a Mega or Mega 2560.

  Version History

  Version 2.1.0 Changes kill switch to also disable movement
  Version 2.0.1 Fixes variable speed jogging when adjusting speed in same direction
  Version 2.0.0 Adds variable speed jogging for DF5
  Version 1.3.1 Report if go-motion speed cannot be reached.
  Version 1.3.0 Arduino 101 support. Remove non-Arduino support (chipKit, Maple).
  Version 1.2.7 Direction setup time.
  Version 1.2.6 Add PINOUT_VERSION option to use older pinout.
  Version 1.2.5 Fix jogging with low pulse rate.
  Version 1.2.4 Fix pin assignments
  Version 1.2.3 New Position command
  Version 1.2.2 Jog and Inch commands
  Version 1.2.1 Moved step/direction pins for motors 5-8.
                Detects board type automatically.
  Version 1.2.0 Basic go-motion capabilities
  Version 1.1.2 Smooth transitions when changing direction
  Version 1.1.1 Save/restore motor position
  Version 1.1.0 Major rework 
  Version 1.0.2 Moved pulses into interrupt handler
  Version 1.0.1 Added delay for pulse widths  
  Version 1.0.0 Initial public release.

  Getting Started:
  
   1. Install IDE (Integrated Development Environment):
      Go to https://www.arduino.cc/en/software and download the Arduino Software for your OS.
   2. Run the IDE you installed.
   3. Open this file in the IDE.
   4. Go to the Tools menu of the IDE and choose the Board type you are using.
   5. Verify/Compile the sketch. (Command-R on Mac, Control-R on Windows.)
   6. After this finishes, Upload the code to the board. (Command-U on Mac, Control-U on Windows.)
   

  Pin configuration:
  
  channel 1
        PIN   4   step
        PIN   5   direction
  channel 2
        PIN   6   step
        PIN   7   direction
  channel 3
        PIN   8   step
        PIN   9   direction
  channel 4
        PIN  10   step
        PIN  11   direction

  channel 5
        PIN  28   step
        PIN  29   direction
  channel 6
        PIN  30   step
        PIN  31   direction
  channel 7
        PIN  32   step
        PIN  33   direction
  channel 8
        PIN  34   step
        PIN  35   direction
 */

// change this to 1 if you want original pinout for channels 5-8
#define PINOUT_VERSION 2

/*
  This is PINOUT_VERSION 1
  
  channel 5
        PIN  22   step
        PIN  23   direction
  channel 6
        PIN  24   step
        PIN  25   direction
  channel 7
        PIN  26   step
        PIN  27   direction
  channel 8
        PIN  28   step
        PIN  29   direction
*/

// detect board type
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define BOARD_MEGA 1
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)  || defined(__AVR_ATmega168__)
  #define BOARD_UNO 1
#elif defined(ARDUINO_ARCH_ARC32) // Intel Curie/101
  #define BOARD_101 1
  #include "CurieTimerOne.h"
#else
  #error Cannot identify board
#endif

// USER: if you want a kill switch, uncomment out the next line by removing the // characters
//#define KILL_SWITCH_PIN 2

#define SERIAL_DEVICE Serial

#if defined(BOARD_101)
  #define PIN_ON(port, pin)  { digitalWrite(pin, 1); }
  #define PIN_OFF(port, pin) { digitalWrite(pin, 0); }
#else
  #define PIN_ON(port, pin)  { port |= pin; }
  #define PIN_OFF(port, pin) { port &= ~pin; }
#endif

// Arduino Uno/Duemilanove  -> 4 MOTORS MAX
// Arduino Mega 2560 / Mega -> 8 MOTORS MAX
#if defined(BOARD_UNO) || defined(BOARD_101)
#define MOTOR_COUNT 4
#else
#define MOTOR_COUNT 8
#endif

#define TIME_CHUNK 50
#define SEND_POSITION_COUNT 20000

// update velocities 20 x second
#define VELOCITY_UPDATE_RATE (50000 / TIME_CHUNK)
#define VELOCITY_INC(maxrate) (max(1.0f, maxrate / 70.0f))
#define VELOCITY_CONVERSION_FACTOR 0.30517578125f /* 20 / 65.536f */

#define MAX_VELOCITY 20000

// setup step and direction pins
#if defined(BOARD_101)

  #define MOTOR0_STEP_PORT 0
  #define MOTOR0_STEP_PIN  4
  
  #define MOTOR1_STEP_PORT 0
  #define MOTOR1_STEP_PIN  6

  #define MOTOR2_STEP_PORT 0
  #define MOTOR2_STEP_PIN  8

  #define MOTOR3_STEP_PORT 0
  #define MOTOR3_STEP_PIN  10

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

  #define MOTOR0_STEP_PORT PORTG
  #define MOTOR0_STEP_PIN  B00100000
  
  #define MOTOR1_STEP_PORT PORTH
  #define MOTOR1_STEP_PIN  B00001000

  #define MOTOR2_STEP_PORT PORTH
  #define MOTOR2_STEP_PIN  B00100000

  #define MOTOR3_STEP_PORT PORTB
  #define MOTOR3_STEP_PIN  B00010000

  #if ( PINOUT_VERSION == 2 )
  
    #define MOTOR4_STEP_PORT PORTA
    #define MOTOR4_STEP_PIN  B01000000
  
    #define MOTOR5_STEP_PORT PORTC
    #define MOTOR5_STEP_PIN  B10000000
  
    #define MOTOR6_STEP_PORT PORTC
    #define MOTOR6_STEP_PIN  B00100000
  
    #define MOTOR7_STEP_PORT PORTC
    #define MOTOR7_STEP_PIN  B00001000

  #elif ( PINOUT_VERSION == 1 )
  
    #define MOTOR4_STEP_PORT PORTA
    #define MOTOR4_STEP_PIN  B00000001
  
    #define MOTOR5_STEP_PORT PORTA
    #define MOTOR5_STEP_PIN  B00000100
  
    #define MOTOR6_STEP_PORT PORTA
    #define MOTOR6_STEP_PIN  B00010000
  
    #define MOTOR7_STEP_PORT PORTA
    #define MOTOR7_STEP_PIN  B01000000

  #endif

#elif defined(BOARD_UNO)

  #define MOTOR0_STEP_PORT PORTD
  #define MOTOR0_STEP_PIN  B00010000
  
  #define MOTOR1_STEP_PORT PORTD
  #define MOTOR1_STEP_PIN  B01000000

  #define MOTOR2_STEP_PORT PORTB
  #define MOTOR2_STEP_PIN  B00000001

  #define MOTOR3_STEP_PORT PORTB
  #define MOTOR3_STEP_PIN  B00000100

#endif



/**
 * Serial output specialization
 */
#if defined(UBRRH)
#define TX_UCSRA UCSRA
#define TX_UDRE  UDRE
#define TX_UDR   UDR
#else
#define TX_UCSRA UCSR0A
#define TX_UDRE  UDRE0
#define TX_UDR   UDR0
#endif
 
char txBuf[32];
char *txBufPtr;

#define TX_MSG_BUF_SIZE 16

#define MSG_STATE_START 0
#define MSG_STATE_CMD   1
#define MSG_STATE_DATA  2
#define MSG_STATE_ERR   3

#define MSG_STATE_DONE  100

/*
 * Command codes from user
 */
#define USER_CMD_ARGS 40

#define CMD_NONE       0
#define CMD_HI         10
#define CMD_MS         30
#define CMD_NP         31
#define CMD_MM         40 // move motor
#define CMD_PR         41 // pulse rate
#define CMD_SM         42 // stop motor
#define CMD_MP         43 // motor position
#define CMD_ZM         44 // zero motor
#define CMD_SA         50 // stop all (hard)
#define CMD_BF         60 // blur frame
#define CMD_GO         61 // go!

#define CMD_JM         70 // jog motor
#define CMD_IM         71 // inch motor


#define MSG_HI 01
#define MSG_MM 02
#define MSG_MP 03
#define MSG_MS 04
#define MSG_PR 05
#define MSG_SM 06
#define MSG_SA 07
#define MSG_BF 10
#define MSG_GO 11
#define MSG_JM 12
#define MSG_IM 13


struct UserCmd
{
  byte command;
  byte argCount;
  int32_t args[USER_CMD_ARGS];
} ;

/*
 * Message state machine variables.
 */
byte lastUserData;
int  msgState;
int  msgNumberSign;
UserCmd userCmd;


struct txMsg
{
  byte msg;
  byte motor;
};

struct TxMsgBuffer
{
  txMsg buffer[TX_MSG_BUF_SIZE];
  byte head;
  byte tail;
};

TxMsgBuffer txMsgBuffer;


/*
 Motor data.
 */

uint16_t           motorAccumulator0;
uint16_t           motorAccumulator1;
uint16_t           motorAccumulator2;
uint16_t           motorAccumulator3;
#if MOTOR_COUNT > 4
uint16_t           motorAccumulator4;
uint16_t           motorAccumulator5;
uint16_t           motorAccumulator6;
uint16_t           motorAccumulator7;
#endif
uint16_t*          motorAccumulator[MOTOR_COUNT] =
{
  &motorAccumulator0, &motorAccumulator1, &motorAccumulator2, &motorAccumulator3, 
#if MOTOR_COUNT > 4
  &motorAccumulator4, &motorAccumulator5, &motorAccumulator6, &motorAccumulator7 
#endif
};

uint16_t           motorMoveSteps0;
uint16_t           motorMoveSteps1;
uint16_t           motorMoveSteps2;
uint16_t           motorMoveSteps3;
#if MOTOR_COUNT > 4
uint16_t           motorMoveSteps4;
uint16_t           motorMoveSteps5;
uint16_t           motorMoveSteps6;
uint16_t           motorMoveSteps7;
#endif
uint16_t*          motorMoveSteps[MOTOR_COUNT] =
{
  &motorMoveSteps0, &motorMoveSteps1, &motorMoveSteps2, &motorMoveSteps3,
#if MOTOR_COUNT > 4
  &motorMoveSteps4, &motorMoveSteps5, &motorMoveSteps6, &motorMoveSteps7
#endif
};


uint16_t           motorMoveSpeed0;
uint16_t           motorMoveSpeed1;
uint16_t           motorMoveSpeed2;
uint16_t           motorMoveSpeed3;
#if MOTOR_COUNT > 4
uint16_t           motorMoveSpeed4;
uint16_t           motorMoveSpeed5;
uint16_t           motorMoveSpeed6;
uint16_t           motorMoveSpeed7;
#endif
uint16_t         * motorMoveSpeed[MOTOR_COUNT] =
{
  &motorMoveSpeed0, &motorMoveSpeed1, &motorMoveSpeed2, &motorMoveSpeed3,
#if MOTOR_COUNT > 4
  &motorMoveSpeed4, &motorMoveSpeed5, &motorMoveSpeed6, &motorMoveSpeed7
#endif
};

volatile boolean nextMoveLoaded;


unsigned int   velocityUpdateCounter;
byte           sendPositionCounter;
boolean        hardStopRequested;

byte sendPosition = 0;
byte motorMoving = 0;
byte toggleStep = 0;


#define P2P_MOVE_COUNT 7

struct Motor
{
  byte   stepPin;
  byte   dirPin;

  // pre-computed move
  float   moveTime[P2P_MOVE_COUNT];
  int32_t movePosition[P2P_MOVE_COUNT];
  float   moveVelocity[P2P_MOVE_COUNT];
  float   moveAcceleration[P2P_MOVE_COUNT];

  float   gomoMoveTime[P2P_MOVE_COUNT];
  int32_t gomoMovePosition[P2P_MOVE_COUNT];
  float   gomoMoveVelocity[P2P_MOVE_COUNT];
  float   gomoMoveAcceleration[P2P_MOVE_COUNT];

  int       currentMove;
  float     currentMoveTime;
  
  volatile  boolean   dir;

  int32_t   position;
  int32_t   destination;
  float     maxVelocity;
  float     maxAcceleration;
  
  uint16_t  nextMotorMoveSteps;
  float     nextMotorMoveSpeed;

};

boolean goMoReady;
int     goMoDelayTime;

Motor motors[MOTOR_COUNT];

int killSwitchState;
      
/*
 * setup() gets called once, at the start of the program.
 */
void setup()
{
  goMoReady = false;
  lastUserData = 0;
  msgState = MSG_STATE_START;
  velocityUpdateCounter = 0;
  sendPositionCounter = 10;
  nextMoveLoaded = false;
  hardStopRequested = false;

  for (int i = 0; i < 32; i++)
    txBuf[i] = 0;
  
  txBufPtr = txBuf;
  
  killSwitchState = 0;
#ifdef KILL_SWITCH_PIN
  killSwitchState = digitalRead(KILL_SWITCH_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  // initialize motor structures
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    // setup motor pins - you can customize/modify these after loop
    // default sets step/dir pairs together, with first four motors at 4/5, 6/7, 8/9, 10/11
    // then, for the Mega boards, it jumps to 28/29, 30/31, 32/33, 34/35
    #if ( PINOUT_VERSION == 2 )
      motors[i].stepPin = (i * 2) + ( (i < 4) ? 4 : 20 );
    #elif ( PINOUT_VERSION == 1 )
      motors[i].stepPin = (i * 2) + ( (i < 4) ? 4 : 14 );
    #endif
    
    motors[i].dirPin = motors[i].stepPin + 1;
    motors[i].dir = true; // forward
    motors[i].position = 0L;
    motors[i].destination = 0L;

    motors[i].nextMotorMoveSteps = 0;
    motors[i].nextMotorMoveSpeed = 0;
    
    setPulsesPerSecond(i, 5000);
  }


  // set output pins
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

    // disable PWM
    switch (motors[i].stepPin)
    {
      #if defined(TCCR3A) && defined(COM3B1)
      case 4:
        TCCR3A &= ~COM3B1;
        break;
      #endif

      #if defined(TCCR4A) && defined(COM4A1)
      case 6:
        TCCR4A &= ~COM4A1;
        break;
      #endif

      #if defined(TCCR4A) && defined(COM4C1)
      case 8:
        TCCR4A &= ~COM4C1;
        break;
      #endif

      #if defined(TCCR2A) && defined(COM2A1)
      case 10:
        TCCR2A &= ~COM2A1;
        break;
      #endif
    }
    
#else
    
    switch (motors[i].stepPin)
    {
      #if defined(TCCR1A) && defined(COM1B1)
      case 10:
        TCCR1A &= ~COM1B1;
        break;
      #endif

    }

#endif
  }
  
  // set initial direction
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    digitalWrite( motors[i].dirPin, motors[i].dir ? HIGH : LOW );
  }

  // setup serial connection
  Serial.begin(57600);

  sendMessage(MSG_HI, 0);
    
  // SET UP interrupt timer  
  #if defined(BOARD_UNO) || defined(BOARD_MEGA)

    TCCR1A = 0;
    TCCR1B = _BV(WGM13);
  
    ICR1 = (F_CPU / 4000000) * TIME_CHUNK; // goes twice as often as time chunk, but every other event turns off pins
    TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
    TIMSK1 = _BV(TOIE1);
    TCCR1B |= _BV(CS10);

  #elif defined(BOARD_101)

    CurieTimerOne.start(25, &updateStepDirection);

  #endif
}

#if defined(BOARD_101)
void updateStepDirection(void)
{
#else
ISR(TIMER1_OVF_vect)
{
#endif

  toggleStep = !toggleStep;
  
  if (toggleStep)
  {
    // MOTOR 1
    if (motorMoveSteps0)
    {
      uint16_t a = motorAccumulator0;
      motorAccumulator0 += motorMoveSpeed0;
      if (motorAccumulator0 < a)
      {
        motorMoveSteps0--;
        
        PIN_ON(MOTOR0_STEP_PORT, MOTOR0_STEP_PIN);
      }
    }

    // MOTOR 2
    if (motorMoveSteps1)
    {
      uint16_t a = motorAccumulator1;
      motorAccumulator1 += motorMoveSpeed1;
      if (motorAccumulator1 < a)
      {
        motorMoveSteps1--;
        
        PIN_ON(MOTOR1_STEP_PORT, MOTOR1_STEP_PIN);
      }
    }

    // MOTOR 3
    if (motorMoveSteps2)
    {
      uint16_t a = motorAccumulator2;
      motorAccumulator2 += motorMoveSpeed2;
      if (motorAccumulator2 < a)
      {
        motorMoveSteps2--;
        
        PIN_ON(MOTOR2_STEP_PORT, MOTOR2_STEP_PIN);
      }
    }

    // MOTOR 4
    if (motorMoveSteps3)
    {
      uint16_t a = motorAccumulator3;
      motorAccumulator3 += motorMoveSpeed3;
      if (motorAccumulator3 < a)
      {
        motorMoveSteps3--;
        
        PIN_ON(MOTOR3_STEP_PORT, MOTOR3_STEP_PIN);
      }
    }

#if MOTOR_COUNT > 4

    // MOTOR 5
    if (motorMoveSteps4)
    {
      uint16_t a = motorAccumulator4;
      motorAccumulator4 += motorMoveSpeed4;
      if (motorAccumulator4 < a)
      {
        motorMoveSteps4--;
        
        PIN_ON(MOTOR4_STEP_PORT, MOTOR4_STEP_PIN);
      }
    }

    // MOTOR 6
    if (motorMoveSteps5)
    {
      uint16_t a = motorAccumulator5;
      motorAccumulator5 += motorMoveSpeed5;
      if (motorAccumulator5 < a)
      {
        motorMoveSteps5--;
        
        PIN_ON(MOTOR5_STEP_PORT, MOTOR5_STEP_PIN);
      }
    }

    // MOTOR 7
    if (motorMoveSteps6)
    {
      uint16_t a = motorAccumulator6;
      motorAccumulator6 += motorMoveSpeed6;
      if (motorAccumulator6 < a)
      {
        motorMoveSteps6--;
        
        PIN_ON(MOTOR6_STEP_PORT, MOTOR6_STEP_PIN);
      }
    }

    // MOTOR 8
    if (motorMoveSteps7)
    {
      uint16_t a = motorAccumulator7;
      motorAccumulator7 += motorMoveSpeed7;
      if (motorAccumulator7 < a)
      {
        motorMoveSteps7--;
        
        PIN_ON(MOTOR7_STEP_PORT, MOTOR7_STEP_PIN);
      }
    }

#endif

  }
  else
  {
    velocityUpdateCounter++;
    if (velocityUpdateCounter == VELOCITY_UPDATE_RATE)
    {
      velocityUpdateCounter = 0;
      
      if (sendPositionCounter)
      {
        sendPositionCounter--;
      }
      
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
        if (*motorMoveSpeed[i] && !motors[i].nextMotorMoveSpeed)
        {
          bitSet(sendPosition, i);
        }

        *motorMoveSteps[i] = motors[i].nextMotorMoveSteps;
        *motorMoveSpeed[i] = motors[i].nextMotorMoveSpeed;
        digitalWrite(motors[i].dirPin, motors[i].dir);

        *motorAccumulator[i] = 65535;
      }
      nextMoveLoaded = false; // ready for new move
    }
    
    PIN_OFF(MOTOR0_STEP_PORT, MOTOR0_STEP_PIN);
    PIN_OFF(MOTOR1_STEP_PORT, MOTOR1_STEP_PIN);
    PIN_OFF(MOTOR2_STEP_PORT, MOTOR2_STEP_PIN);
    PIN_OFF(MOTOR3_STEP_PORT, MOTOR3_STEP_PIN);

    #if MOTOR_COUNT > 4
      PIN_OFF(MOTOR4_STEP_PORT, MOTOR4_STEP_PIN);
      PIN_OFF(MOTOR5_STEP_PORT, MOTOR5_STEP_PIN);
      PIN_OFF(MOTOR6_STEP_PORT, MOTOR6_STEP_PIN);
      PIN_OFF(MOTOR7_STEP_PORT, MOTOR7_STEP_PIN);
    #endif
  }
}

/*
 * For stepper-motor timing, every clock cycle counts.
 */
void loop()
{
  int32_t *ramValues = (int32_t *)malloc(sizeof(int32_t) * MOTOR_COUNT);
  int32_t *ramNotValues = (int32_t *)malloc(sizeof(int32_t) * MOTOR_COUNT);

  for (int i = 0; i < MOTOR_COUNT; i++)
  { 
    if (ramValues[i] == ~ramNotValues[i])
    {
      motors[i].position = motors[i].destination = ramValues[i];
    }
  }  

  while (true)
  {
#ifdef KILL_SWITCH_PIN
    int currentState = digitalRead(KILL_SWITCH_PIN);
    if (currentState && !killSwitchState)
    {
      hardStopRequested = true;
    }
    killSwitchState = currentState;
    digitalWrite(LED_BUILTIN, killSwitchState);
#endif

    if (!nextMoveLoaded)
      updateMotorVelocities();
    
    processSerialCommand();
    
    // check if we have serial output
    #if defined(BOARD_UNO) || defined(BOARD_MEGA)
    if (*txBufPtr)
    {
      if ((TX_UCSRA) & (1 << TX_UDRE))
      {
        TX_UDR = *txBufPtr++;
  
        // we are done with this msg, get the next one
        if (!*txBufPtr)
          nextMessage();
      }
    }
    #endif

    if (!sendPositionCounter)
    {
      sendPositionCounter = 20;

      byte i;
      for (i = 0; i < MOTOR_COUNT; i++)
      {
        if (bitRead(motorMoving, i) || bitRead(sendPosition, i))
        {
          sendMessage(MSG_MP, i);
          ramValues[i] = motors[i].position;
          ramNotValues[i] = ~motors[i].position;
        }
      }

      sendPosition = 0;
    }
  }
}

/**
 * Update velocities.
 */

void updateMotorVelocities()
{
  // process hard stop interrupt request
  if (hardStopRequested)
  {
    hardStopRequested = 0;
    hardStop();
  }
  
  for (int m = 0; m < MOTOR_COUNT; m++)
  {
    Motor *motor = &motors[m];
    motor->nextMotorMoveSteps = 0;
    motor->nextMotorMoveSpeed = 0;

    if (bitRead(motorMoving, m))
    {
      int seg = motor->currentMove;
      
      if (motor->moveTime[seg] == 0)
      {
        bitClear(motorMoving, m);
      }
      else
      {
        float originalMoveTime = motor->currentMoveTime;
        int originalMove = motor->currentMove;
        
        motor->currentMoveTime += 0.05f;
        
        if (motor->currentMoveTime >= motor->moveTime[seg])
        {
          motor->currentMoveTime -= motor->moveTime[seg];
          motor->currentMove++;
          seg++;
        }
        float t = motor->currentMoveTime;
        int32_t xn = (int32_t)(motor->movePosition[seg] + motor->moveVelocity[seg] * t + motor->moveAcceleration[seg] * t * t); // accel was already multiplied * 0.5

        int32_t dx = abs(xn - motor->position);

        if (!dx) // don't change direction flag unless we are actually stepping in new direction
          continue;
          
        boolean forward = xn > motor->position;

        if (forward != motor->dir) // direction setup time 1/20th second should be plenty
        {
          // revert everything except for dir flag
          motor->currentMoveTime = originalMoveTime;
          motor->currentMove = originalMove;
        }
        else
        {
          motor->nextMotorMoveSpeed = max(1, min(65535, dx * 65.6f));
          motor->nextMotorMoveSteps = dx;
          motor->position = xn;
        }
        
        motor->dir = forward;
      }      
    }
  }
  nextMoveLoaded = true;
}

/*
 * Set up the axis for pulses per second (approximate)
 */
void setPulsesPerSecond(int motorIndex, uint16_t pulsesPerSecond)
{
  if (pulsesPerSecond > MAX_VELOCITY)
    pulsesPerSecond = MAX_VELOCITY;
  if (pulsesPerSecond < 100)
    pulsesPerSecond = 100;
    
  motors[motorIndex].maxVelocity = pulsesPerSecond;
  motors[motorIndex].maxAcceleration = pulsesPerSecond * 0.5f;  
}


void setupMotorMove(int motorIndex, int32_t destination)
{
  motors[motorIndex].destination = destination;

  if (destination != motors[motorIndex].position)
  {
    calculatePointToPoint(motorIndex, destination);
    bitSet(motorMoving, motorIndex);
  }

}


void hardStop()
{
  // set the destination to the current location, so they won't move any more
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    stopMotor(i);
  }
}

void stopMotor(int motorIndex)
{
  int32_t delta = (motors[motorIndex].destination - motors[motorIndex].position);
  if (!delta)
    return;

  Motor *motor = &motors[motorIndex];
  int i;

  for (i = 0; i < P2P_MOVE_COUNT; i++)
  {
    motor->moveTime[i] = 0;
    motor->moveVelocity[i] = 0;
    motor->movePosition[i] = 0;
  }

  float v = VELOCITY_CONVERSION_FACTOR * motors[motorIndex].nextMotorMoveSpeed;
  float maxA = motor->maxAcceleration;
  float maxV = motor->maxVelocity;

  if (v > maxV)
    v = maxV;

  if (!motor->dir)
    v = -v;

  float t = fabsf(v / maxA);

  motor->moveTime[0] = t;
  motor->movePosition[0] = motor->position;
  motor->moveVelocity[0] = v;
  motor->moveAcceleration[0] = (v > 0) ? -maxA : maxA;

  motor->moveTime[1] = 0;
  motor->movePosition[1] = (int32_t)(motor->movePosition[0] + motor->moveVelocity[0] * t + 0.5f * motor->moveAcceleration[0] * t * t);
  motor->moveVelocity[1] = 0;
  motor->moveAcceleration[1] = 0;

  motor->moveAcceleration[0] *= 0.5f;

  motor->destination = motor->movePosition[1];
  
  motor->currentMoveTime = 0;
  motor->currentMove = 0;  
}

boolean isValidMotor(int motorIndex)
{
  return (motorIndex >=0 && motorIndex < MOTOR_COUNT);
}


void processGoPosition(int motorIndex, int32_t pos)
{
  if (motors[motorIndex].position != pos)
  {
    setupMotorMove(motorIndex, pos);
    sendMessage(MSG_MM, motorIndex);
  }
  else
  {
    sendMessage(MSG_MP, motorIndex);
  }
}

/*

Command format

ASCII
[command two bytes]

Version
"hi"
-> "hi 1"

zero motor
"zm 1"
-> "z 1"

move motor
"mm 1 +1111111111

motor position?
mp 1

MOTOR STATUS
"ms"
-> "ms [busy motor count]"

SET PULSE PER SECOND
pr 1 200

STOP MOTOR
sm 1

STOP ALL
sa

*/

/*
 * int processUserMessage(char data)
 *
 * Read user data (from virtual com port), processing one byte at a time.
 * Implemented with a state machine to reduce memory overhead.
 *
 * Returns command code for completed command.
 */
byte processUserMessage(char data)
{
  byte cmd = CMD_NONE;

  switch (msgState)
  {
  case MSG_STATE_START:
    if (data != '\r' && data != '\n')
    {
      msgState = MSG_STATE_CMD;
      msgNumberSign = 1;
      userCmd.command = CMD_NONE;
      userCmd.argCount = 0;
      userCmd.args[0] = 0;
    }
    break;

  case MSG_STATE_CMD:
    if (lastUserData == 'h' && data == 'i')
    {
      userCmd.command = CMD_HI;
      msgState = MSG_STATE_DONE;
    }
    else if (lastUserData == 'm' && data == 's')
    {
      userCmd.command = CMD_MS;
      msgState = MSG_STATE_DONE;
    }
    else if (lastUserData == 's' && data == 'a')
    {
      userCmd.command = CMD_SA;
      msgState = MSG_STATE_DONE;
    }
    else if (lastUserData == 'm' && data == 'm')
    {
      userCmd.command = CMD_MM;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 'n' && data == 'p')
    {
      userCmd.command = CMD_NP;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 'm' && data == 'p')
    {
      userCmd.command = CMD_MP;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 'z' && data == 'm')
    {
      userCmd.command = CMD_ZM;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 's' && data == 'm')
    {
      userCmd.command = CMD_SM;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 'p' && data == 'r')
    {
      userCmd.command = CMD_PR;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 'b' && data == 'f')
    {
      userCmd.command = CMD_BF;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 'g' && data == 'o')
    {
      userCmd.command = CMD_GO;
      msgState = MSG_STATE_DONE;
    }
    else if (lastUserData == 'j' && data == 'm') // jm [motor] [destination position]
    {
      userCmd.command = CMD_JM;
      msgState = MSG_STATE_DATA;
    }
    else if (lastUserData == 'i' && data == 'm') // im [motor] [destination position]
    {
      userCmd.command = CMD_IM;
      msgState = MSG_STATE_DATA;
    }
    else
    {
      // error msg? unknown command?
      msgState = MSG_STATE_START;
    }
    break;

  case MSG_STATE_DATA:
    if (((data >= '0' && data <= '9') || data == '-') && lastUserData == ' ')
    {
      userCmd.argCount++;
      if (userCmd.argCount >= USER_CMD_ARGS)
      {
        SERIAL_DEVICE.print("error: too many args\r\n");
        msgState = MSG_STATE_ERR;
      }
      else
      {
        userCmd.args[userCmd.argCount - 1] = 0;
        if (data == '-')
        {
          msgNumberSign = -1;
        }
        else
        {
          msgNumberSign = 1;
          userCmd.args[userCmd.argCount - 1] = (data - '0');
        }
      }
    }
    else if (data >= '0' && data <= '9')
    {
      userCmd.args[userCmd.argCount - 1] = userCmd.args[userCmd.argCount - 1] * 10 + (data - '0');
    }
    else if (data == ' ' || data == '\r')
    {
      if (lastUserData  >= '0' && lastUserData <= '9')
      {
        if (userCmd.argCount > 0)
          userCmd.args[userCmd.argCount - 1] *= msgNumberSign;
      }
      if (data == '\r')
      {
        msgState = MSG_STATE_DONE;
      }
    }
    break;


  case MSG_STATE_ERR:
    userCmd.command = CMD_NONE;
    msgState = MSG_STATE_DONE;
    break;

  case MSG_STATE_DONE:
    // wait for newline, then reset
    if (data == '\n' && lastUserData == '\r')
    {
      cmd = userCmd.command;
      msgState = MSG_STATE_START;
      lastUserData = 0;
    }
    break;

  default: // unknown state -> revert to begin
    msgState = MSG_STATE_START;
    lastUserData = 0;
  }

  lastUserData = data;

  return cmd;
}

void processSerialCommand()
{
  byte avail = SERIAL_DEVICE.available();
  byte motor;
  int m;

  for (int i = 0; i < avail; i++)
  {
    int cmd = processUserMessage(SERIAL_DEVICE.read());
    
    if (cmd != CMD_NONE)
    {
      boolean parseError = false;

      motor = userCmd.args[0] - 1;
      
      switch (cmd)
      {
        case CMD_HI:
          sendMessage(MSG_HI, 0);
          break;
        
        case CMD_ZM:
          parseError = (userCmd.argCount != 1 || !isValidMotor(motor));
          if (!parseError)
          {
            motors[motor].position = 0;
            setupMotorMove(motor, 0);
            processGoPosition(motor, 0);
            bitSet(sendPosition, motor);
          }
          break;

        case CMD_MM:
          parseError = (userCmd.argCount != 2 || !isValidMotor(motor));
          if (!parseError && !killSwitchState)
          {
            processGoPosition(motor, (int32_t)userCmd.args[1]);
          }
          break;

        case CMD_NP:
          parseError = (userCmd.argCount != 2 || !isValidMotor(motor));
          if (!parseError)
          {
            motors[motor].position = userCmd.args[1];
            sendMessage(MSG_MP, motor);
          }
          break;


        case CMD_MP:
          parseError = (userCmd.argCount != 1 || !isValidMotor(motor));
          if (!parseError)
          {
            sendMessage(MSG_MP, motor);
          }
          break;

        case CMD_MS:
          parseError = (userCmd.argCount != 0);
          if (!parseError)
          {
            sendMessage(MSG_MS, 0);
          }
          break;

        case CMD_SM:
          parseError = (userCmd.argCount != 1 || !isValidMotor(motor));
          if (!parseError)
          {
            stopMotor(motor);
            sendMessage(MSG_SM, motor);
            sendMessage(MSG_MP, motor);
          }
          break;

        case CMD_SA:
          parseError = (userCmd.argCount != 0);
          if (!parseError)
          {
            hardStop();
            sendMessage(MSG_SA, 0);
          }
          break;

        case CMD_PR:
          parseError = (userCmd.argCount != 2 || !isValidMotor(motor));
          if (!parseError)
          {
            setPulsesPerSecond(motor, (uint16_t)userCmd.args[1]);
            sendMessage(MSG_PR, motor);
          }
          break;

        case CMD_BF:
          parseError = motorMoving || userCmd.argCount < 5 || ((userCmd.argCount - 2) % 4) != 0;
          if (!parseError && !killSwitchState)
          {
            goMoDelayTime = 1000;
            
            int motorCount = (userCmd.argCount - 2) / 4;
            
            for (m = 0; m < MOTOR_COUNT; m++)
            {
              motors[m].gomoMoveTime[0] = 0.0f;
            }

            int32_t destPositions[MOTOR_COUNT];
            for (m = 0; m < motorCount; m++)
            {
              int offset = 2 + m * 4;
              motor = userCmd.args[offset] - 1;
              if (!isValidMotor(motor))
              {
                parseError = true;
                break;
              }
              destPositions[m] = setupBlur(motor, userCmd.args[0], userCmd.args[1], userCmd.args[offset + 1], userCmd.args[offset + 2], userCmd.args[offset + 3]);
            }
            goMoReady = (goMoDelayTime >= 0);
            if (goMoReady)
            {
              for (m = 0; m < motorCount; m++)
              {
                int offset = 2 + m * 4;
                motor = userCmd.args[offset] - 1;
                setupMotorMove(motor, destPositions[m]);
              }
            }
            sendMessage(MSG_BF, 0);

          }
          break;

        case CMD_GO:
          parseError = motorMoving || (userCmd.argCount > 0) || !goMoReady;
          if (!parseError && !killSwitchState)
          {
            for (m = 0; m < MOTOR_COUNT; m++)
            {
               if (motors[m].gomoMoveTime[0] != 0)
               {
                 int j;
                 for (j = 0; j < P2P_MOVE_COUNT; j++)
                 {
                     motors[m].moveTime[j] = motors[m].gomoMoveTime[j];
                     motors[m].movePosition[j] = motors[m].gomoMovePosition[j];
                     motors[m].moveVelocity[j] = motors[m].gomoMoveVelocity[j];
                     motors[m].moveAcceleration[j] = motors[m].gomoMoveAcceleration[j];
                 }
                 motors[m].destination = motors[m].gomoMovePosition[4]; // TODO change this!
                 motors[m].currentMove = 0;
                 bitSet(motorMoving, m);
               }
            }
            updateMotorVelocities();
            noInterrupts();
            velocityUpdateCounter = VELOCITY_UPDATE_RATE - 1;
            interrupts();
            sendMessage(MSG_GO, 0);            
          }
          break;
          
        case CMD_JM:
          parseError = (userCmd.argCount < 2 || !isValidMotor(motor));
          if (!parseError && !killSwitchState)
          {
            float speedAdjustment = 1.0f;
            float maxVelocity = motors[motor].maxVelocity;
            float maxAcceleration = motors[motor].maxAcceleration;
            
            if (userCmd.argCount == 3)
            {
              speedAdjustment = userCmd.args[2] * 0.0001f;
              motors[motor].maxVelocity = fmax(4.0f, (motors[motor].maxVelocity * speedAdjustment));
              motors[motor].maxAcceleration = fmax(4.0f, (motors[motor].maxAcceleration * speedAdjustment));
            }
            int32_t destination = 0;
            if (jogMotor(motor, userCmd.args[1], &destination))
            {
              if (!bitRead(motorMoving, motor) || destination != motors[motor].destination)
              {
                setupMotorMove(motor, destination);
              }
            }
            motors[motor].maxVelocity = maxVelocity;
            motors[motor].maxAcceleration = maxAcceleration;

            sendMessage(MSG_JM, motor);
          }
          break;

        case CMD_IM:
          parseError = (userCmd.argCount != 2 || !isValidMotor(motor));
          if (!parseError && !killSwitchState)
          {
            inchMotor(motor, userCmd.args[1]);
            sendMessage(MSG_IM, motor);
          }
          break;
          
        default:
          parseError = true;
          break;
      }
      
      if (killSwitchState)
      {
        sendMessage(MSG_SA, 0);
      }
      if (parseError)
      {
        SERIAL_DEVICE.print("parse error\r\n");
      }
    }
  }
}


/*
 *
 * Serial transmission.
 *
 */
void sendMessage(byte msg, byte motorIndex)
{
#if defined(BOARD_UNO) || defined(BOARD_MEGA)

  int i = (unsigned int)(txMsgBuffer.head + 1) % TX_MSG_BUF_SIZE;

  if (i != txMsgBuffer.tail)
  {
    txMsgBuffer.buffer[txMsgBuffer.head].msg = msg;
    txMsgBuffer.buffer[txMsgBuffer.head].motor = motorIndex;
    txMsgBuffer.head = i;
    
    if (!*txBufPtr)
      nextMessage();
  }

#else
  int i;
  
  switch (msg)
  {
    case MSG_HI:
      SERIAL_DEVICE.print("hi ");
      SERIAL_DEVICE.print(DFMOCO_VERSION);
      SERIAL_DEVICE.print(" ");
      SERIAL_DEVICE.print(MOTOR_COUNT);
      SERIAL_DEVICE.print(" ");
      SERIAL_DEVICE.print(DFMOCO_VERSION_STRING);
      SERIAL_DEVICE.print("\r\n");
      break;
    case MSG_MM:
      SERIAL_DEVICE.print("mm ");
      SERIAL_DEVICE.print(motorIndex + 1);
      SERIAL_DEVICE.print(" ");
      SERIAL_DEVICE.print(motors[motorIndex].destination);
      SERIAL_DEVICE.print("\r\n");
      break;
    case MSG_MP:
      SERIAL_DEVICE.print("mp ");
      SERIAL_DEVICE.print(motorIndex + 1);
      SERIAL_DEVICE.print(" ");
      SERIAL_DEVICE.print(motors[motorIndex].position);
      SERIAL_DEVICE.print("\r\n");
      break;
    case MSG_MS:
      SERIAL_DEVICE.print("ms ");
      for (i = 0; i < MOTOR_COUNT; i++)
        SERIAL_DEVICE.print(bitRead(motorMoving, i) ? '1' : '0');
      SERIAL_DEVICE.print("\r\n");
      break;
    case MSG_PR:
      SERIAL_DEVICE.print("pr ");
      SERIAL_DEVICE.print(motorIndex + 1);
      SERIAL_DEVICE.print(" ");
      SERIAL_DEVICE.print((uint16_t)motors[motorIndex].maxVelocity);
      SERIAL_DEVICE.print("\r\n");
      break;
    case MSG_SM:
      SERIAL_DEVICE.print("sm ");
      SERIAL_DEVICE.print(motorIndex + 1);
      SERIAL_DEVICE.print("\r\n");
      break;
    case MSG_SA:
      SERIAL_DEVICE.print("sa\r\n");
      break;
    case MSG_BF:
      SERIAL_DEVICE.print("bf ");
      SERIAL_DEVICE.print(goMoDelayTime);
      SERIAL_DEVICE.print("\r\n");
    case MSG_GO:
      SERIAL_DEVICE.print("go\r\n");
      break;
    case MSG_JM:
      SERIAL_DEVICE.print("jm ");
      SERIAL_DEVICE.print(motorIndex + 1);
      SERIAL_DEVICE.print("\r\n");
      break;
    case MSG_IM:
      SERIAL_DEVICE.print("im ");
      SERIAL_DEVICE.print(motorIndex + 1);
      SERIAL_DEVICE.print("\r\n");
      break;
  }
#endif
}

#if defined(BOARD_UNO) || defined(BOARD_MEGA)
void nextMessage()
{
  char *bufPtr;
  int i;
  
  if ((TX_MSG_BUF_SIZE + txMsgBuffer.head - txMsgBuffer.tail) % TX_MSG_BUF_SIZE)
  {
    byte msg = txMsgBuffer.buffer[txMsgBuffer.tail].msg;
    byte motorIndex = txMsgBuffer.buffer[txMsgBuffer.tail].motor;
    txMsgBuffer.tail = (unsigned int)(txMsgBuffer.tail + 1) % TX_MSG_BUF_SIZE;

    switch (msg)
    {
      case MSG_HI:
        sprintf(txBuf, "hi %d %d %s\r\n", DFMOCO_VERSION, MOTOR_COUNT, DFMOCO_VERSION_STRING);
        break;
      case MSG_MM:
        sprintf(txBuf, "mm %d %ld\r\n", motorIndex + 1, motors[motorIndex].destination);
        break;
      case MSG_MP:
        sprintf(txBuf, "mp %d %ld\r\n", motorIndex + 1, motors[motorIndex].position);
        break;
      case MSG_MS:
        sprintf(txBuf, "ms ");
        bufPtr = txBuf + 3;
        for (i = 0; i < MOTOR_COUNT; i++)
          *bufPtr++ = bitRead(motorMoving, i) ? '1' : '0';
        *bufPtr++ = '\r';
        *bufPtr++ = '\n';
        *bufPtr = 0;
        break;
      case MSG_PR:
        sprintf(txBuf, "pr %d %u\r\n", motorIndex + 1, (uint16_t)motors[motorIndex].maxVelocity);
        break;
      case MSG_SM:
        sprintf(txBuf, "sm %d\r\n", motorIndex + 1);
        break;
      case MSG_SA:
        sprintf(txBuf, "sa\r\n");
        break;
      case MSG_BF:
        sprintf(txBuf, "bf %d\r\n", goMoDelayTime);
        break;
      case MSG_GO:
        sprintf(txBuf, "go\r\n");
        break;
      case MSG_JM:
        sprintf(txBuf, "jm %d\r\n", motorIndex + 1);
        break;
      case MSG_IM:
        sprintf(txBuf, "im %d\r\n", motorIndex + 1);
        break;
    }
    
    txBufPtr = txBuf;
  }
}
#endif

boolean jogMotor(int motorIndex, int32_t target, int32_t * destination)
{
  Motor *motor = &motors[motorIndex];
  // ideally send motor to distance where decel happens after 2 seconds
  float vi = (motor->dir ? 1 : -1) * VELOCITY_CONVERSION_FACTOR * motor->nextMotorMoveSpeed;
  
  int dir = (target > motor->position) ? 1 : -1;
  // if switching direction, just stop
  if (motor->nextMotorMoveSpeed && (motor->dir ? 1 : -1) * dir < 0)
  {
    stopMotor(motorIndex);
    return false;
  }
  if (target == motor->position)
  {
    return false;
  }
  
  float maxVelocity = motor->maxVelocity;
  float maxAcceleration = motor->maxAcceleration;
  
  
  // given current velocity vi
  // compute distance so that decel starts after 0.5 seconds
  // time to accel
  // time at maxvelocity
  // time to decel
  float accelTime = 0, atMaxVelocityTime = 0;
  if (fabsf(vi) < maxVelocity)
  {
    accelTime = (maxVelocity - fabsf(vi)) / maxAcceleration;
    if (accelTime < 0.5f)
    {
      atMaxVelocityTime = 0.5f - accelTime;
    }
    else
    {
      accelTime = 0.5f;
    }
  }
  else
  {
    atMaxVelocityTime = 0.5f;
  }
  float maxVelocityReached = fabsf(vi) + maxAcceleration * accelTime;

  int32_t delta = fabsf(vi) * accelTime + (0.5f * maxAcceleration * accelTime * accelTime)
                  + atMaxVelocityTime * maxVelocityReached
                  + 0.5f * (maxVelocityReached * maxVelocityReached) / maxAcceleration; // = 0.5 * a * t^2 -> t = (v/a)
  
  int32_t dest = motor->position + dir * delta;
  
  // now clamp to target
  if ( (dir == 1 && dest > target) || (dir == -1 && dest < target) )
  {
    dest = target;
  }
  *destination = dest;
  return true;
}

void inchMotor(int motorIndex, int32_t target)
{
  Motor *motor = &motors[motorIndex];
  // ideally send motor to distance where decel happens after 2 seconds
  
  // if switching direction, just stop
  int dir = (target > motor->destination) ? 1 : -1;
  
  if (motor->nextMotorMoveSpeed)// && motor->dir * dir < 0)
  {
    stopMotor(motorIndex);
    return;
  }

  int32_t dest = motor->destination + dir * 2;
  
  // now clamp to target
  if ( (dir == 1 && dest > target) || (dir == -1 && dest < target) )
  {
    dest = target;
  }
  //setupMotorMove(motorIndex, dest);
  
  int i;

  for (i = 0; i < P2P_MOVE_COUNT; i++)
  {
    motor->moveTime[i] = 0;
    motor->moveVelocity[i] = 0;
    motor->moveAcceleration[i] = 0;
  }
  motor->currentMoveTime = 0;
  motor->moveTime[0] = 0.01f;
  motor->movePosition[0] = motor->position;
  motor->movePosition[1] = motor->position + dir * 2;
  motor->currentMove = 0;
 
  motor->destination = dest;

  if ( dest != motor->position )
  {
    bitSet(motorMoving, motorIndex);
  }
}

void calculatePointToPoint(int motorIndex, int32_t destination)
{
  Motor *motor = &motors[motorIndex];
  
  int i, moveCount;
  int moveProgrammed = 0;

  moveCount = 0;

  for (i = 0; i < P2P_MOVE_COUNT; i++)
  {
    motor->moveTime[i] = 0;
    motor->moveVelocity[i] = 0;
    motor->moveAcceleration[i] = 0;
  }
  motor->currentMoveTime = 0;
  motor->movePosition[0] = motor->position;

  float maxA = motor->maxAcceleration;
  float maxV = motor->maxVelocity;
  if (maxV == 0.0f)
  {
    maxV = 100.0f;
  }
  if (maxA == 0.0f)
  {
    maxA = maxV;
  }
  float maxAInverse = 1.0f / maxA;

  float tmax = maxV * maxAInverse;
  float dmax = maxV * tmax;
  
  float dist = fabsf(destination - motor->position);
  int dir = (destination > motor->position) ? 1 : -1;
  
  if (motor->nextMotorMoveSpeed > 5) // we need to account for existing velocity
  {
    float vi = (motor->dir ? 1 : -1) * VELOCITY_CONVERSION_FACTOR * motor->nextMotorMoveSpeed;
    float ti = fabsf(vi / maxA);
    float di = 0.5f * maxA * ti * ti;
    
    if (vi * dir < 0) // switching directions
    {
      motor->moveTime[moveCount] = ti;
      motor->moveAcceleration[moveCount] = dir * maxA;
      motor->moveVelocity[moveCount] = vi;
      moveCount++;
      
      dist += di;
    }
    else if (dist < di) // must decelerate and switch directions
    {
      motor->moveTime[moveCount] = ti;
      motor->moveAcceleration[moveCount] = -dir * maxA;
      motor->moveVelocity[moveCount] = vi;
      moveCount++;

      dist = (di - dist);
      dir = -dir;
    }
    else // further on in same direction
    {
       float deltaV = fabsf(vi) - maxV;

      if (deltaV > 0)
      {
          float tDecel = fabsf(deltaV * maxAInverse);

          motor->moveTime[moveCount] = tDecel;
          motor->moveAcceleration[moveCount] = -dir * maxA;
          motor->moveVelocity[moveCount] = vi;

          float dDecel = fabsf(vi * tDecel + 0.5f * -dir * maxA * tDecel * tDecel);

          dist -= dDecel;

          dist -= 0.5f * dmax;

          float tconst = dist / maxV;

          moveCount++;
          motor->moveTime[moveCount] = tconst;
          motor->moveAcceleration[moveCount] = 0;

          moveCount++;
          motor->moveTime[moveCount] = tmax;
          motor->moveAcceleration[moveCount] = dir * -maxA;

          moveProgrammed = 1;
      }
      else
      {
        dist += di;
        motor->movePosition[0] -= dir * di;
        motor->currentMoveTime = ti;
      }
    }
  }

  if (!moveProgrammed)
  {
    float t = tmax;
    if (dist <= dmax)
    {
      t = sqrtf(dist * maxAInverse);
    }
      
    motor->moveTime[moveCount] = t;
    motor->moveAcceleration[moveCount] = dir * maxA;
    
    if (dist > dmax)
    {
      moveCount++;
      dist -= dmax;
      float tconst = dist / maxV;
      motor->moveTime[moveCount] = tconst;
      motor->moveAcceleration[moveCount] = 0;
    }

    moveCount++;
    motor->moveTime[moveCount] = t;
    motor->moveAcceleration[moveCount] = dir * -maxA;
  }

  for (i = 1; i <= moveCount; i++)
  {
    float t = motor->moveTime[i - 1];
    motor->movePosition[i] = (int32_t)(motor->movePosition[i - 1] + motor->moveVelocity[i - 1] * t + 0.5f * motor->moveAcceleration[i - 1] * t * t);
    motor->moveVelocity[i] = motor->moveVelocity[i - 1] + motor->moveAcceleration[i - 1] * t;
  }
  motor->movePosition[moveCount + 1] = destination;
  for (i = 0; i <= moveCount; i++)
  {
    motor->moveAcceleration[i] *= 0.5f; // pre-multiply here for later position calculation
  }
  motor->currentMove = 0;
  
  return;

}

int32_t setupBlur(int motorIndex, int exposure, int blur, int32_t p0, int32_t p1, int32_t p2)
{
  Motor *motor = &motors[motorIndex];
  int i;
  
  float b = blur * 0.001f;
  float expTime = exposure * 0.001f;
  
  p0 = p1 + b * (p0 - p1);
  p2 = p1 + b * (p2 - p1);

  for (i = 0; i < P2P_MOVE_COUNT; i++)
  {
    motor->gomoMoveTime[i] = 0;
    motor->gomoMoveVelocity[i] = 0;
    motor->gomoMoveAcceleration[i] = 0;
  }
  
  motor->gomoMovePosition[1] = p0;
  motor->gomoMoveTime[1] = expTime * 0.5f;
  motor->gomoMoveVelocity[1] = (float)(p1 - p0) / (expTime * 0.5f);

  motor->gomoMovePosition[2] = p1;
  motor->gomoMoveTime[2] = expTime * 0.5f;
  motor->gomoMoveVelocity[2] = (float)(p2 - p1) / (expTime * 0.5f);

  if (fabsf(motor->gomoMoveVelocity[1]) > MAX_VELOCITY || fabsf(motor->gomoMoveVelocity[2]) > MAX_VELOCITY)
    goMoDelayTime = -1; // can not reach this speed

  // v = a*t -> a = v / t
  float accelTime = 1.0f;
  float a = motor->gomoMoveVelocity[1] / accelTime;
  float dp = 0.5f * a * accelTime * accelTime;
  float sp = p0 - dp; // starting position

  motor->gomoMovePosition[0] = sp;
  motor->gomoMoveTime[0] = accelTime;
  motor->gomoMoveAcceleration[0] = 0.5f * a; // pre-multiplied

  a = motor->gomoMoveVelocity[2] / accelTime;
  dp = 0.5f * a * accelTime * accelTime;
  float fp = p2 + dp;

  motor->gomoMovePosition[3] = p2;
  motor->gomoMoveTime[3] = accelTime;
  motor->gomoMoveVelocity[3] = motor->gomoMoveVelocity[2];
  motor->gomoMoveAcceleration[3] = -0.5f * a; // pre-multiplied

  motor->gomoMovePosition[4] = fp;

  return (int32_t)sp;
}
