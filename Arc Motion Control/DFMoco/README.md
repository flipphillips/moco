**Simple Motion Control with Dragonframe (DFMoco)**

**Overview **

This sketch turns an Arduino or equivalent development board into a
multi-axis

motion control signal generator. It is for use with the Arc motion
control system in

Dragonframe 4. It generates step and direction signals, which can be
sent to

stepper motor drivers.

Note: This is NOT a REAL TIME solution.

This is a simple, low cost, motion control solution for stop motion and
time-lapse.

Dragonframe will move all of your motors to position for frame 1, then
shoot, then

move all of your motors to position for frame 2.

If you need real-time moves you should check out the DMC-32:

<https://www.dragonframe.com/product/dmc-32/>

**Choosing a Development Board**

Throughout this document we refer to Arduino because it is the most
well-known

small development board. You can actually use a variety of boards:

To control up to four axes, you can use:

\- Arduino Uno

\- Arduino Duemilanove

\- Arduino 101

To control up to eight axes, you can use:

\- Arduino Mega

\- Arduino Mega 2560

**Wiring the Arduino for Motion Control**

The Arduino running the DFMoco sketch will generate step and direction
signals for stepper motors.

If you already have stepper motor drivers, you can take these signals
and wire them into a connector for those drivers.

If you do not have stepper motor drivers, and need to move relatively
small stepper motors, you can wire the Arduino signals to an Big Easy
Driver or comparable board. You will use one Big Easy Driver for each
stepper motor.

[[https://www.sparkfun.com/products/10735](http://www.sparkfun.com/products/10267)]{.underline}

**Kill Switch**

It is recommended to incorporate a pushbutton kill switch, especially
for larger rigs. This will stop all motors and bypasses any
communication issues between the computer and the Arduino.

The DFMoco sketch expects you to have a kill switch attached to PIN 2
(interrupt 0).

If you do NOT want a kill switch, you must comment out this line in the
code

    #define KILL_SWITCH_INTERRUPT 0

You comment it out by add two forward slashes at the start, so it looks
like this

    //#define KILL_SWITCH_INTERRUPT 0

You can reference the schematic (but not the code) on this page if you
are not sure how to connect a pushbutton:

<https://docs.arduino.cc/built-in-examples/digital/Button>

**Step/Direction Pin Configuration**

Channel 1

PIN 4 step

PIN 5 direction

Channel 2

PIN 6 step

PIN 7 direction

Channel 3

PIN 8 step

PIN 9 direction

Channel 4

PIN 10 step

PIN 11 direction

Channel 5

PIN 28 step

PIN 29 direction

Channel 6

PIN 30 step

PIN 31 direction

Channel 7

PIN 32 step

PIN 33 direction

Channel 8

PIN 34 step

PIN 35 direction

Install the Arduino Software

If you haven\'t already done so, you will need to install the Arduino
software:

Go to <https://www.arduino.cc/en/software> and download the Arduino
Software for your OS.

**Loading the DFMoco Program**

Once the Arduino software is installed, and your development board is
wired, you need to load the DFMoco program

(called a sketch in Arduino terminology) onto the board.

1\. Launch the IDE that you previously installed.

2\. Open \"DFMoco.ino\" located in this folder.

3\. Set your specific board using the \"Tools\" menu, \"Board\" submenu.

4\. \"Verify\" the sketch by pressing the play button on the top left of
the Arduino program.\
(Command-R on Mac, Control-R on Windows)

5\. \"Upload\" the sketch by pressing the button with a right arrow
pointing to some dots.\
(Command-U on Mac, Control-U on Windows)

**Connecting Dragonframe and the Arduino Board**

Your board is ready to go. Now you can start using it with Dragonframe:

1\. Start Dragonframe.

2\. Create a new scene or open a previous one.

3\. Select **Connections\...** from the **Scene** menu.

4\. Press **Add Connection** and choose **DFMoco Arduino** as the device
type.

5\. Select **ArcMoco #1 (or #2, #3, #4).**

6\. Choose the appropriate serial port.

7\. Press the **Connect** button.

8\. Refer to the Dragonframe User Guide, \"Motion Control\" chapter, and
to our online tutorials for further instructions.
