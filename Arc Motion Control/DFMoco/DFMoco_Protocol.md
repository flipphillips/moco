**DFMoco Protocol**

Version 2.0.0

Messages are given from perspective of DFMoco device.

Motor number starts at 1.

All positions are in steps. Dragonframe converts between step count and
units.

**Protocol**

Serial Port (Comm)

Speed: 57600 Baud

Data bits: 8

Stop bits: 1

Parity: 0

**Message Format**

Messages are in ASCII text, with trailing \\r\\n

**Messages**

**Hello (hi) - initiate communication**

IN: hi

OUT: hi \<major version\> \<number of axes\> \<full version string\>

EXAMPLE: hi 1 8 1.2.7

**Motor Status (ms) - check moving status of all motors**

IN: ms

OUT: ms \<0 or 1 for each motor, 1 = moving\>

EXAMPLE: ms 10000000

**Move Motor (mm) - move a motor to a position**

IN: mm \<motor\> \<step position\>

EXAMPLE: mm 1 250

OUT: mm \<motor\> \<target step position\>

OUT2: mp \<motor\> \<step position\>

EXAMPLE: mm 1 250 (if motor needs to move)

EXAMPLE: mp 1 250 (if motor was already there)

**Motor Position Query (mp) - request motor position**

IN: mp \<motor\>

OUT: mp \<motor\> \<step position\>

**Stop Motor (sm) - stop a motor fairly quickly**

IN: sm \<motor\>

EXAMPLE: sm 1

OUT: sm \<motor\>

EXAMPLE: sm 1

**Stop All (sa) - stop all motors fairly quickly**

IN: sa

OUT: sa

**Jog Motor (jm) - move the motor at a reasonable speed (pulse rate)
towards destination**

IN: jm \<motor\> \<destination\> \[\<velocity\>\]

OUT: jm \<motor\>

Note: **velocity **will be passed as a third argument if the **major
version **in the **hi **response is 2 or greater. It is a positive
number between 0 and 10000, with 10000 meaning to run at 100% of **pulse
rate**.

**Inch Motor (im) - move the in very small increments towards
destination**

IN: im \<motor\> \<destination\>

OUT: im \<motor\>

**Pulse Rate (pr) - set the maximum steps/second of motor**

IN: pr \<motor\> \<steps/second\>

EXAMPLE: pr 1 20000

OUT: pr \<motor\> \<steps/second\>

**Zero Motor (zm) - resets the motor\'s current position to zero (no
movement)**

IN: zm \<motor\>

OUT: zm \<motor\>

**New Position (np) - sets the motor\'s current position to a new value
(no movement)**

IN: np \<motor\> \<position\>

OUT: np \<motor\> \<position\>

**Go-Motion (Blur):**

This is complicated, and not required.

The goal is to capture a single frame, with the camera moving over a
certain area while the shutter is open.

IN: bf \<exposure time ms\> \<blur 0-1000\> \[ch#1 number\] \[ch#1 p0\]
\[ch#1 p1\] \[ch#1 p2\] \[ch#2 number\] \[ch#2 p0\] \[ch#2 p1\] \[ch#2
p2\] \[etc\]

OUT: bf 1000

Explanation:

The exposure time is how long the camera will be exposing the image.

The blur amount is in the range 0-1000, where 500 would correspond to a
50% or 180 degree blur setting (standard).

Then, for each motor that should move you receive four parameters:

1\. A channel number.

2\. A previous frame position value.

3\. A current frame position value.

4\. A next frame position value.

Each motor\'s movement should be centered around p1, such that:

The distance that is traveled is in relation to the blur percent (with
500 being 50%). 100% would mean the exposure happens for the entire
distance of p0-\>p1-\>p2.

For 50% (the more standard case), the exposure happens at half the
distance between p1 and the previous and next positions.

In mathematical terms, the exposure happens between these positions:

\[p1 - (p1 - p0) \* blurPercent\] -\> \[p1\] -\> \[p1 + (p2 - p1) \*
blurPercent\]

And the exposure time should be evenly split between the two segments.

So it should take 1/2 of the exposure time to travel \[p1 - (p1 - p0) \*
blurPercent\] -\> \[p1\] ,

and 1/2 of the exposure time to travel \[p1\] -\> \[p1 + (p2 - p1) \*
blurPercent\]

Furthermore, there should be a ramp up to speed before the exposure
starts, and a slow down after it stops.

The ramp up to speed should take exactly 1 second, even if the device
does not need to move. This is so that Dragonframe knows when to trigger
the camera.

After the device calculates the go-motion move, it starts sending all
motors into the initial position.

Dragonframe will check for motor movement, and when everything has
stopped, it will issue a \"go\" command:

IN: go

OUT: go

At this point the rig should immediately start the go-motion move as
described above.

**Notes**

Any time a motor is moving, the device should send occasional \"mp\"
messages.

Dragonframe will keep requesting motor status (ms) until the movement is
finished.
