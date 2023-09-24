# MultiBASIC040 PRT - Power/Reset/Timer
This is a microcontroller for handling soft power, reset timing, and a programmable interval timer. Target is the Microchip/Atmel ATMega328P (Arduino Uno R3). It is written using the Arduino libraries to make it easier to get up and running, but does not use the Arduino I/O routines.

The main program loop is a set of state machines handling system power, system reset, and CPU read/write accesses to the internal registers.

There are three internal registers:

| A[1:0] | Register                 |
| :----: | :---                     |
| 00     | Status/Control register  |
| 01     | [not used]               |
| 10     | timer interval low byte  |
| 11     | timer interval high byte |


## Interval Timer
The timer interval is specified in microseconds using registers 2 & 3. This value is only applied to the timer on the rising edge of the TimerEnable bit. 