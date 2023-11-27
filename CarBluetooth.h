#ifndef LCDDISPLAY

// ****************** FEATURE FLAGS ************************************
#define WINDOW                       0	     // activates the intelligent window roll up/down feature
#define LCDDISPLAY                   1       // activates bluetooth display's serial port
#define MOTIONSENSOR                 0	     // activates motion sensor authentication feature
#define IRSENSOR                     1       // activates the infrared motion sensor
#define TURBOTIMER                   1	     // activates turbo timer feature
#define BLUETOOTH                    0       // activates the bluetooth headset input mechanism
#define REPORTMEMORY                 0       // activates a report of available mem out the serial port
#define SLEEPMODE                    1       // activates sleep mode
#define WATCHDOG                     1       // activates the watchdog timer used for micro reset

// ****************** DEPENDANCIES *************************************
#define AUTHENTICATION (MOTIONSENSOR | BLUETOOTH) // activates the user authentication mechanism
#if !MOTIONSENSOR
  #define BLUETOOTH                0         // if the motion sensor isn't around, no point having the bluetooth
#endif // #if !MOTIONSENSOR

#if MOTIONSENSOR
  #define IRSENSOR                 0         // the IR sensor and motion sensor won't both be used
#endif // #if MOTIONSENSOR

// ****************** INPUT PIN ASSIGNMENTS ****************************
#define tachIn                       1       // INTERRUPT - Tachometer, pin 3
#define switchIn                     0       // INTERRUPT - Power Button, pin 2
                                             // NOTE - This is one of the few pins that will wake micro
#define brakeIn                      48      // brake switch input

#if MOTIONSENSOR
  #define motionIn                   0	     // motion sensor analog input - Used for IR sensor too!
#endif // #if MOTIONSENSOR

#if WINDOW
  #define windowIn                   1	     // window sensor will be a hall effect to this analog in
#endif // #if WINDOW

#if TURBOTIMER
  #define tpsIn                      2	     // tps is a 0-5 volt pot analog input
#endif // #if TURBOTIMER

#if BLUETOOTH
  #define bluetoothLEDIn             2       // INTERRUPT - Bluetooth LED input, pin 21
                                             // pin that looks for the bluetooth headset LED flash
#endif // if BLUETOOTH

#if AUTHENTICATION
  #define alarmLEDPin                4       // INTERRUPT - Alarm LED input, pin 19, which is Serial1's Rx line
                                             // pin that monitors alarm LED for status
#endif // #if AUTHENTICATION

#if IRSENSOR
  #define IRSensorIn                 15       // ANALOG IR sensor input
#endif // #if IRSENSOR

// ****************** OUTPUT PIN ASSIGNMENTS ***************************
#define accRelay                     26      // accessory key position relay output
#define ignRelay                     24      // ignition key position relay output
#define starterRelay                 22      // starter key position relay output
#define greenLED                     13      // power button's green LED
#define amberLED                     11      // power button's amber LED
#define backlightLED                 9       // power button's backlight LED - must be PWM for dimming

#if WINDOW
  #define windowOut                  31      // Output that energizes the window motor
  #define windowPowerOut             33      // Output that energizes the power window relay
#endif // #if WINDOW

#if LCDDISPLAY
  #define LCDPowerPin                47      // pin that controls LCD power
#endif // #if LCDDISPLAY

#if AUTHENTICATION
  #define alarmPin                   39      // pin that toggles the alarm fob connection to arm / disarm alarm
#endif // #if AUTHENTICATION

#if MOTIONSENSOR
  #define motionPowerPin             42      // pin that uses darlington pair to power 12 volt motion sensor
                                             // setup this way to turn off microwave radiation while driving
                                             // USED FOR IR SENSOR TOO
#endif // #if MOTIONSENSOR

#if IRSENSOR
  #define IRLEDOutput                53      // IR LED's output power (feeds both IR LED and sensor)
#endif // #if IRSENSOR

// ****************** PROCESS VARIABLE ASSIGNMENTS **********************
#define relayOn                      HIGH    // tracks which polarity a pin needs to go to turn the relays "on"
                                             // this is assuming NPN or N-Channel transistors are used

#define relayOff                     LOW     // tracks which polarity a pin needs to go to turn the relays "off"

#define LEDOn                        HIGH    // tracks which polarity a pin needs to go to turn the LEDs "on"
                                             // this will allow flexibility for installation

#define LEDOff                       LOW     // tracks which polarity a pin needs to go to turn the LEDs "off"

#define startSequenceTimeout         8000    // number of ms the starter is allowed to crank without successful engine start
#define malfunctionTimeout           2000    // timeout while waiting for a good tach signal while cranking in ms, defaulted to 2s
#define engineNotRunningTachTimeout  1500    // no tach signals in this timeframe (in ms) means engine is stopped
#define tachTimerDeltaThreashold     24      // bother with the expensive floating point math if tach varies by 400 RPM
#define crankingThreashold           300     // engine speed treashold above which engine is considered running
#define fuelPumpPrimeTimeout         2000    // timeout waiting for the EMS to initialize and prime the fuel system
#define errorRecoveryTimeout         10000   // global timeout that forces the user to cool the starter before another attempt
#define engineRunningUpdateLimit     500     // amount of time allowed before verification if engine is running in milliseconds
#define debugSerialCharacter         's'     // character that turns on serial port debug messages
#define splashScreenResetCharacter   'r'     // character that runs the function to reset the LCD splash screen
#define memoryReportCharacter        'm'     // character that turns on memory reporting in the debug section
#define disableSleepModeCharacter    'd'     // character that disables sleep mode
#define resetPowerStateCounter       10      // limits the number of times the main loop can iterate with the powerState = HIGH to solve race conditions

#if WINDOW
  #define windowtimeout              4000    // backup timeout to kill the window relay should the threashold never be tripped in ms
#endif // #if WINDOW

#if TURBOTIMER
  #define turbotimeout               60000   // maximum time car is allowed to idle in worst-case turbo temp scenario in ms
#endif // #if TURBOTIMER

#if LCDDISPLAY
  #define lcdpowertimeout            20000   // max time allowed for the LCD to be powered up but the key off in ms
  #define LCDBootupTimeout           2000    // LCD screen boot up time in ms
#endif // #if LCDDISPLAY

#if MOTIONSENSOR
  #define motionThreashold           600     // motion detection threashold (higher # = more sensitive)
  #if !BLUETOOTH                             // no bluetooth present:
    #define timeForAuthentication    3000    // user must trigger the above times in this many ms
    #define eventsForAuthentication  5       // user must trigger this many times
  #endif // #if !BLUETOOTH
#endif // #if MOTIONSENSOR

#if AUTHENTICATION
  #define alarmLEDTimeout            2000    // timeout waiting for alarm LED to change state before it is considered solidly on / off
  #define powerBacklightTimeout      1500.0  // time it takes to dim the power button backlight from off to on or vice versa
  #define minTimeBetweenFobEvents    3000    // minimum allowed time between fob button press events
  #define fobButtonPressDuration     500     // duration the button is held down in ms
  #define alarmLEDTimeout            5000    // timeout after which the LED is no longer considered blinking
#endif // #if AUTHENTICATION

#if BLUETOOTH
  #define bluetoothLEDTimeout        5000    // timeout after which the LED is no longer considered blinking
#endif // #if BLUETOOTH

#if SLEEPMODE
  #define sleepTimeout               30000   // time until system sleeps in ms
#endif // #if SLEEPMODE

#if IRSENSOR
  #define IRTriggerThreashold        50      // difference in A/D readings sample-to-sample to trigger system
#endif // #if IRSENSOR

#endif // #ifndef LCDDISPLAY
