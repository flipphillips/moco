**Real-Time Motion Control with Dragonframe (dmc-lite)**

**Overview**

This sketch turns an Arduino Giga R1 or Portenta H7 into a multi-axis
motion control signal generator. It is for use with the Arc motion
control system in

Dragonframe 4 and newer. It generates step and direction signals, which
can be sent to stepper motor drivers.

This has many of the features of our DMC-32 device:

<https://www.dragonframe.com/product/dmc-32/>

Note that the Arduinos are still hobby boards, and we provide this code
as a convenience for do-it-yourselfers. We expect you to have a decent
level of comfort with basic circuitry if you attempt to use it.

**Choosing a Development Board**

The Arduino Giga R1 closely resembles the Arduino Mega 2560 in terms of
size. It has pin headers that make it easy to wire to drivers or other
inputs and outputs.

The Arduino Portenta H7 is a much smaller board. The default pinout in
our sketch uses the high-density J2 port. This means you need a breakout
board to connect to the signals.

**Wiring the Arduino for Motion Control**

The Arduino running the **dmc-lite** sketch will generate step and
direction signals for stepper motors. Note that these signals are 3.3V
logic level. If your driver needs 5V signals, you may need to add
voltage stepper circuitry. That is beyond the scope of our advice.

If you already have stepper motor drivers, you can take these signals
and wire them into a connector for those drivers.

The best stepper motor drivers are from Geckodrive. However, you can
find many less expensive ones at SparkFun.

**Kill Switch / E-Stop / Emergency Stop**

It is recommended to incorporate a pushbutton kill switch, especially
for larger rigs. This will stop all motors and bypasses any
communication issues between the computer and the Arduino.

The dmc_m7/config.h file has instructions for enabling this feature.

You can reference the schematic (but not the code) on this page if you
are not sure how to connect a pushbutton:

<https://docs.arduino.cc/built-in-examples/digital/Button>

**Step/Direction Pin Configuration**

The dmc_m4/config.h file contains the pin assigments for all step and
direction signals. They are different for the Giga R1 and the Portenta
H7.

Install the Arduino Software

If you haven\'t already done so, you will need to install the Arduino
software:

Go to <https://www.arduino.cc/en/software> and download the Arduino
Software for your OS.

**Loading the dmc-lite Program**

Once the Arduino software is installed, and your development board is
wired, you need to load the dmc-lite program (called a sketch in Arduino
terminology) onto the board.

The Giga R1 and Portenta H7 are dual-core devices. You will load
different programs onto each core.

First, load dmc_m7 onto the main core:

1\. Launch the Arduino IDE.

2\. Open \"dmc_m7/dmc_m7.ino\".

3\. Set your specific board using the \"Tools\" menu, \"Board\" submenu.

4\. In the Tools menu, set the \"Target core\" to \"Main core\".

5\. In the Tools menu, set the \"Flash split\" to \"1.5MB M7 + 0.5MB
M4\"

6\. \"Upload\" the sketch by pressing the button with a right arrow.\
(Command-U on Mac, Control-U on Windows)

Then, load the dmc_m4 sketch onto the second core:

1\. Launch the Arduino IDE.

2\. Open \"dmc_m4/dmc_m4.ino\".

3\. Set your specific board using the \"Tools\" menu, \"Board\" submenu.

4\. In the Tools menu, set the \"Target core\" to \"M4 Co-processor\".

5\. In the Tools menu, set the \"Flash split\" to \"1.5MB M7 + 0.5MB
M4\"

6\. \"Upload\" the sketch by pressing the button with a right arrow.\
(Command-U on Mac, Control-U on Windows)

**Connecting Dragonframe and the Arduino Board**

Your board is ready to go. Now you can start using it with Dragonframe:

1\. Start Dragonframe.

2\. Create a new scene or open a previous one.

3\. Select **Connections\...** from the **Scene** menu.

4\. Press **Add Connection** and choose **dmc-lite** (or any DMC device)
****as the device type.

5\. Select **ArcMoco #1 (or #2, #3, #4).**

**6.** Choose the appropriate serial port.

7\. Press the **Connect** button.

8\. Refer to the Dragonframe User Guide, \"Motion Control\" chapter, and
to our online tutorials for further instructions.
