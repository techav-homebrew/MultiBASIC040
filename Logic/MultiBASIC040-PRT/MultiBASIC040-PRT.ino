#include <avr/io.h>
//#include <TimerOne.h>
// copy library to project directory instead
#include "TimerOne/TimerOne.h"

// this bit is verbose but helps minimize silly mistakes
#define atxPwrOkPort  PINC
#define atxPwrOkPin   0
#define atxPwrOkRead  (atxPwrOkPort & (1<<atxPwrOkPin))
#define atxPwrOkInit  DDRC &= ~(1<<atxPwrOkPin)

#define atxPwrOnPort  PORTC
#define atxPwrOnPin   1
#define atxPwrOnSet   atxPwrOnPort |= (1<<atxPwrOnPin)
#define atxPwrOnClear atxPwrOnPort &= (1<<atxPwrOnPin)
#define atxPwrOnInit  DDRC |= (1<<atxPwrOnPin)

#define cpuTSnPort    PINC
#define cpuTSnPin     2
#define cpuTSnRead    (cpuTSnPort & (1<<cpuTSnPin))
#define cpuTSnInit    DDRC &= ~(1<<cpuTSnPin)

#define cpuTAnPort    PORTC
#define cpuTAnPin     3
#define cpuTAnSet     cpuTAnPort |= (1<<cpuTAnPin)
#define cpuTAnClear   cpuTAnPort &= (1<<cpuTAnPin)
#define cpuTAnInit    DDRC |= (1<<cpuTAnPin)

#define busRSTnPort   PORTC
#define busRSTnPin    4
#define busRSTnSet    busRSTnPort |= (1<<busRSTnPin)
#define busRSTnClear  busRSTnPort &= (1<<busRSTnPin)
#define busRSTnInit   DDRC |= (1<<busRSTnPin)

#define busRWnPort    PINC
#define busRWnPin     5
#define busRWnRead    (busRWnPort & (1<<busRWnPin))
#define busRWnInit    DDRC &= ~(1<<busRWnPin)

#define cpuA0Port     PINB
#define cpuA0Pin      0
#define cpuA0Read     (cpuA0Port & (1<<cpuA0Pin))
#define cpuA0Init     DDRB &= ~(1<<cpuA0Pin)

#define cpuA1Port     PINB
#define cpuA1Pin      1
#define cpuA1Read     (cpuA1Port & (1<<cpuA1Pin))
#define cpuA1Init     DDRB &= ~(1<<cpuA1Pin)

#define timeIRQnPort  PORTB
#define timeIRQnPin   2
#define timeIRQnSet   timeIRQnPort |= (1<<timeIRQnPin)
#define timeIRQnClear timeIRQnPort &= (1<<timeIRQnPin)
#define timeIRQnInit  DDRB |= (1<<timeIRQnPin)

#define timeIAKnPort  PINB
#define timeIAKnPin   3
#define timeIAKnRead  (timeIAKnPort & (1<<timeIAKnPin))
#define timeIAKnInit  DDRB &= ~(1<<timeIAKnPin)

#define swPWRnPort    PINB
#define swPWRnPin     4
#define swPWRnRead    (swPWRnPort & (1<<swPWRnPin))
#define swPWRnInit    DDRB &= ~(1<<swPWRnPin); PORTB |= (1<<swPWRnPin)

#define swRSTnPort    PINB
#define swRSTnPin     5
#define swRSTnRead    (swRSTnPort & (1<<swRSTnPin))
#define swRSTnInit    DDRB &= ~(1<<swRSTnPin); PORTB |= (1<<swRSTnPin)

#define busDataPort   PORTD
#define busDataHiZ    DDRD = 0; PORTD = 0;
#define busDataDrive  DDRD = 0xff;

// define states for state machines
#define sPowerOff     0   // main power is off, waiting for button press
#define sPowerOn      1   // main power is on, waiting for button or soft off
#define sPowerOnDeb   2   // main power is off, debounce button press
#define sPowerOffHld  3   // main power is on, time button hold for hard off

#define sResetOff     0   // main power is off, waiting for power on
#define sResetOn      1   // main power is on, waiting for button press
#define sResetDeb     2   // main power is on, debounce button press
#define sResetHld     3   // main power is on, time reset pulse

#define sCycleOff     0   // main power is off, waiting for power on
#define sCycleOn      1   // main power is on, waiting for CPU cycle start
#define sCycleRd      2   // cpu read cycle in progress, waiting for end
#define sCycleWr      3   // cpu write cycle in progress, waiting for en

// define status/control register bits
#define regSoftPower  0   // power on when 1; power off when 0
#define regTimeEnable 1   // timer enabled when 1; timer disabled when 0

// some more helpful shortcuts
#define POWER_IS_ON   (regStatCtrl & (1<<regSoftPower))
#define BTN_PRESSED   0
#define BTN_RELEASED  1
#define FALSE 0
#define TRUE 1
#define CPUADDRREAD   (0 | (cpuA1Read << 1) | (cpuA0Read << 0))
#define NOP   __asm__ __volatile__ ("nop\n\t")

// timing constants
#define POWER_DEBOUNCE  5     // cycles to count for debouncing power button
#define POWER_HARD_OFF  32000 // cycles to count for hard power off
#define RESET_DEBOUNCE  5     // cycles to count for debouncing reset button
#define RESET_TIME      50    // cycles to count for reset pulse

#define TIMER_DEFAULT   2000  // default timer interval - 2000 microseconds

// set initial states for state machines
uint8_t powerState = sPowerOff;
uint8_t resetState = sResetOff;
uint8_t cycleState = sCycleOff;

// set up some cycle counters
uint16_t powerCount = 0;
uint16_t resetCount = 0;

// set up our CPU-accessible registers
uint8_t volatile regStatCtrl = 0;
uint16_t volatile regTimer = TIMER_DEFAULT;

// timer interrupt service routine
void timerISR(void) {
  // first stop the timer
  Timer1.stop();

  // assert the interrupt request, wait a cycle, then deassert
  timeIRQnClear;
  NOP;
  timeIRQnSet;

  // that's all we need to do here
}

void setup() {
  noInterrupts();

  // initialize our I/O pins
  busDataHiZ;
  atxPwrOkInit;
  atxPwrOnInit;
  cpuTSnInit;
  cpuTAnInit;
  busRSTnInit;
  busRWnInit;
  cpuA0Init;
  cpuA1Init;
  timeIRQnInit;
  timeIAKnInit;
  swPWRnInit;
  swRSTnInit;

  // initialize the timer and configure its ISR
  Timer1.initialize(TIMER_DEFAULT);
  Timer1.stop();
  Timer1.attachInterrupt(timerISR);
}

void loop() {
  // service Power state machine first:
  switch(powerState) {
    case sPowerOff:
      // wait for the power button to be pressed
      regStatCtrl &= ~(1<<regSoftPower);
      powerCount = 0;
      if(swPWRnRead == BTN_PRESSED) {
        // power button has been pressed, go to debounce
        powerState = sPowerOnDeb;
      } else {
        // keep waiting for button press
        powerState = sPowerOff;
      }
      break;
    case sPowerOn:
      // wait for either soft power off or power button to be pressed
      powerCount = 0;
      if(!POWER_IS_ON) {
        // soft power off
        powerState = sPowerOff;
      } else if(swPWRnRead == BTN_PRESSED) {
        // power button has been pressed, go to hold
        powerState = sPowerOffHld;
      } else {
        // keep waiting
        powerState = sPowerOn;
      }
      break;
    case sPowerOnDeb:
      powerCount++;
      if(swPWRnRead == BTN_PRESSED) {
        // button is still being pressed
        if(powerCount > POWER_DEBOUNCE) {
          powerState = sPowerOn;
        } else {
          powerState = sPowerOnDeb;
        }
      } else {
        // button was released early
        powerState = sPowerOff;
      }
      break;
    case sPowerOffHld:
      powerCount++;
      if(swPWRnRead == BTN_PRESSED) {
        // button is still being held
        if(powerCount > POWER_HARD_OFF) {
          powerState = sPowerOff;
        } else {
          powerState = sPowerOffHld;
        }
      } else {
        // button was released early
        powerState = sPowerOn;
      }
      break;
    default:
      powerState = sPowerOff;
      break;
  }

  // service Reset state machine next:
  switch(resetState) {
    case sResetOff:
      resetCount = 0;
      if(POWER_IS_ON) {
        // system has been powered on, do a reset cycle
        resetState = sResetHld;
      } else {
        resetState = sResetOff;
      }
      break;
    case sResetOn:
      resetCount = 0;
      if(!POWER_IS_ON) {
        // system has been powered off, go to off
        resetState = sResetOff;
      } else if(swRSTnRead == BTN_PRESSED) {
        // reset button has been pressed, do debounce
        resetState = sResetDeb;
      } else {
        // else hold here
        resetState = sResetOn;
      }
      break;
    case sResetDeb:
      resetCount++;
      if(swRSTnRead == BTN_PRESSED) {
        // button is still pressed
        if(resetCount > RESET_DEBOUNCE) {
          // debounce time expired, do reset cycle
          resetState = sResetHld;
        } else {
          // keep waiting
          resetState = sResetDeb;
        }
      } else {
        // button was released early
        resetState = sResetOn;
      }
      break;
    case sResetHld:
      resetCount++;
      if((atxPwrOkRead == LOW) && (resetCount > RESET_TIME)) {
        // reset time met and power supply is ready
        resetState = sResetOn;
      } else {
        // still holding
        resetState = sResetHld;
      }
      break;
    default:
      resetState = sResetOff;
      break;
  }

  // service CPU Cycle state machine last:
  switch(cycleState) {
    case sCycleOff:
      if(POWER_IS_ON) {
        // system has been powered on
        cycleState = sCycleOn;
      } else {
        // wait for system power on 
        cycleState = sCycleOff;
      }
      break;
    case sCycleOn:
      if(!POWER_IS_ON) {
        // power has been turned off
        cycleState = sCycleOff;
      } else if(cpuTSnRead == LOW) {
        // CPU is starting a bus cycle directed at us
        if(busRWnRead == LOW) {
          // CPU is starting a write cycle
          cycleState = sCycleWr;
        } else {
          cycleState = sCycleRd;
        }
      } else {
        // wait for CPU to start a bus cycle
        cycleState = sCycleOn;
      }
      break;
    case sCycleRd:
      // wait for CPU to end bus cycle
      if(cpuTSnRead == LOW) {
        // still waiting
        cycleState = sCycleRd;
      } else {
        cycleState = sCycleOn;
      }
      break;
    case sCycleWr:
      // wait for CPU to end bus cycle
      if(cpuTSnRead == LOW) {
        // still waiting
        cycleState = sCycleWr;
      } else {
        cycleState = sCycleOn;
      }
      break;
    default:
      cycleState = sCycleOff;
      break;
  }

  // set outputs based on the states above

  // reset output
  if(resetState == sResetHld) {
    busRSTnClear;
  } else {
    busRSTnSet;
  }

  // power on output
  if(powerState == sPowerOn || powerState == sPowerOffHld) {
    atxPwrOnSet;
  } else {
    atxPwrOnClear;
  }

  // transfer acknowledge
  if(cycleState == sCycleRd || cycleState == sCycleWr) {
    cpuTAnClear;
  } else {
    cpuTAnSet;
  }

  // data bus
  if(cycleState == sCycleRd) {
    // bus to output
    busDataDrive;
    switch (CPUADDRREAD) {
      case 0:
        // CPU reading status/control register
        busDataPort = regStatCtrl;
        break;
      case 1:
        // nothing happens here
        busDataPort = 0xff;
        break;
      case 2:
        // CPU reading timer low word
        busDataPort = (uint8_t)(regTimer & 0xff);
        break;
      case 3:
        // CPU reading timer high word
        busDataPort = (uint8_t)((regTimer >> 8) & 0xff);
        break;
      default:
        break;
    }
  } else if(cycleState = sCycleWr) {
    // bus read cycle
    busDataHiZ;
    switch (CPUADDRREAD) {
      case 0:
        // CPU writing status/control register
        uint8_t readData = busDataPort;
        if((regStatCtrl & (1<<regTimeEnable) == 0) && (readData & (1<<regTimeEnable) == 1)) {
          // this is rising edge of the timer enable bit, configure & start the timer
          Timer1.setPeriod(regTimer);
          Timer1.start();
        } else if(readData & (1<<regTimeEnable) == 0) {
          // user is clearing the timer enable bit, make sure timer is stopped.
          Timer1.stop();
        }
        regStatCtrl = readData;
        break;
      case 1:
        // nothing happens here
        break;
      case 2:
        // CPU writing timer low word
        regTimer = (regTimer & 0xff00) | ((uint16_t)(busDataPort) & 0x00ff);
        break;
      case 3:
        // CPU writing timer high word
        regTimer = (regTimer & 0x00ff) | ((uint16_t)(busDataPort) & 0xff00);
        break;
      default:
        break;
    }
  } else {
    // bus to Hi-Z
    busDataHiZ;
  }
}
