// ****************** FEATURE FLAGS ************************************
#define WINDOW                       0	     // activates the intelligent window roll up/down feature
#define LCDDISPLAY                   1       // activates bluetooth display's serial port
#define MOTIONSENSOR                 0	     // activates motion sensor authentication feature
#define IRSENSOR                     0       // activates the infrared motion sensor
#define TURBOTIMER                   1	     // activates turbo timer feature
#define BLUETOOTH                    0       // activates the bluetooth headset input mechanism
#define REPORTMEMORY                 0       // activates a report of available mem out the serial port
#define SLEEPMODE                    0       // activates sleep mode
#define WATCHDOG                     0       // activates the watchdog timer used for micro reset
#define CANBUS                       0       // activates the CAN bus communication capability
#define TIMEMAINLOOP                 0       // bit bangs an output so an oscillscope can be used to time main loop

// ****************** DEPENDANCIES *************************************
#define AUTHENTICATION (MOTIONSENSOR | BLUETOOTH) // activates the user authentication mechanism
#if !MOTIONSENSOR
  #define BLUETOOTH                  0       // if the motion sensor isn't around, no point having the bluetooth
#endif // #if !MOTIONSENSOR

#if MOTIONSENSOR
  #define IRSENSOR                   0       // the IR sensor and motion sensor won't both be used
#endif // #if MOTIONSENSOR

// ****************** INPUT PIN ASSIGNMENTS ****************************
#define tachIn                       1       // INTERRUPT - Tachometer
#define powerIn                      0       // INTERRUPT - Power Button
                                             // NOTE - This is one of the few pins that will wake micro
#define brakeIn                      2       // brake switch input

#if MOTIONSENSOR
  #define motionIn                   0	     // (pin 14) motion sensor analog input - Used for IR sensor too!
#endif // #if MOTIONSENSOR

#if WINDOW
  #define windowIn                   1	     // (pin 15) window sensor will be a hall effect to this analog in
#endif // #if WINDOW

#if TURBOTIMER
  #define tpsIn                      2	     // (pin 16) tps
#endif // #if TURBOTIMER

#if BLUETOOTH
  #define bluetoothLEDIn             6       // INTERRUPT - Bluetooth LED input
                                             // pin that looks for the bluetooth headset LED flash
#endif // if BLUETOOTH

#if AUTHENTICATION
  #define alarmLEDPin                11      // INTERRUPT - Alarm LED input, which is Serial1's Rx line
                                             // pin that monitors alarm LED for status
#endif // #if AUTHENTICATION

#if IRSENSOR
  #define IRSensorIn                 3       // (pin 17) ANALOG IR sensor input
#endif // #if IRSENSOR

#if TIMEMAINLOOP
  #define mainLoopTimerPin           20      // main loop timer oscillscope connection
#endif // #if TIMEMAINLOOP

// ****************** OUTPUT PIN ASSIGNMENTS ***************************
#define accRelay                     12      // accessory key position relay output
#define ignRelay                     18      // ignition key position relay output
#define starterRelay                 19      // starter key position relay output
#define greenLED                     20      // power button's green LED
#define amberLED                     21      // power button's amber LED
#define backlightLED                 22      // power button's backlight LED - must be PWM for dimming
#define statusLED                    13      // controls the board's LED

#if WINDOW
  #define windowOut                  25      // Output that energizes the window motor
  #define windowPowerOut             24      // Output that energizes the power window relay
#endif // #if WINDOW

#if LCDDISPLAY
  #define LCDPowerPin                23      // pin that controls LCD power
#endif // #if LCDDISPLAY

#if AUTHENTICATION
  #define alarmPin                   26      // pin that toggles the alarm fob connection to arm / disarm alarm
#endif // #if AUTHENTICATION

#if MOTIONSENSOR
  #define motionPowerPin             27      // pin that uses darlington pair to power 12 volt motion sensor
                                             // setup this way to turn off microwave radiation while driving
                                             // USED FOR IR SENSOR TOO
#endif // #if MOTIONSENSOR

#if IRSENSOR
  #define IRLEDOutput                28      // IR LED's output power (feeds both IR LED and sensor)
#endif // #if IRSENSOR

// ****************** PROCESS VARIABLE ASSIGNMENTS **********************
#define RELAY_ON                     HIGH    // tracks which polarity a pin needs to go to turn the relays "on"
                                             // this is assuming NPN or N-Channel transistors are used

#define RELAY_OFF                    LOW     // tracks which polarity a pin needs to go to turn the relays "off"

#define POWER_ON                     LOW     // tracks how the power switch electrically turns "on"
#define POWER_OFF                    HIGH

#define BRAKE_ON                     HIGH    // tracks how the brake switch electrically turns "on"
#define BRAKE_OFF                    LOW

#define LEDOn                        HIGH    // tracks which polarity a pin needs to go to turn the LEDs "on"
                                             // this will allow flexibility for installation

#define LEDOff                       LOW     // tracks which polarity a pin needs to go to turn the LEDs "off"

#if RELAY_ON // RELAY_ON is set to HIGH, which = 1
  #define KEY_OFF                      0       // bit field tracking relays: (accessory, ignition, start)
  #define KEY_ACC                      4       // (1,0,0)
  #define KEY_ON                       6       // (1,1,0)
  #define KEY_START                    3       // (0,1,1)
#else
  #define KEY_OFF                      0       // bit field tracking relays: (accessory, ignition, start)
  #define KEY_ACC                      3       // (0,1,1)
  #define KEY_ON                       1       // (0,0,1)
  #define KEY_START                    4       // (1,0,0)
#endif

#define startSequenceTimeout         8000    // number of ms the starter is allowed to crank without successful engine start
#define malfunctionTimeout           2000    // timeout while waiting for a good tach signal while cranking in ms, defaulted to 2s
#define engineNotRunningTachTimeout  2000    // no tach signals in this timeframe (in ms) means engine is stopped
#define tachTimerDeltaThreashold     75      // bother with the expensive floating point math if tach varies by 400 RPM
#define crankingThreashold           400     // engine speed treashold above which engine is considered running
#define fuelPumpPrimeTimeout         2000    // timeout waiting for the EMS to initialize and prime the fuel system
#define errorRecoveryTimeout         10000   // global timeout that forces the user to cool the starter before another attempt
#define engineRunningUpdateLimit     500     // amount of time allowed before verification if engine is running in milliseconds
#define debugSerialCharacter         's'     // character that turns on serial port debug messages
#define splashScreenResetCharacter   'r'     // character that runs the function to reset the LCD splash screen
#define memoryReportCharacter        'm'     // character that turns on memory reporting in the debug section
#define disableSleepModeCharacter    'd'     // character that disables sleep mode
#define resetPowerStateCounter       10      // limits the number of times the main loop can iterate with the powerState = HIGH to solve race conditions
#define LCDBacklightTimeout          10000   // the amount of time the LCD back light is allowed to be on when the engine (and thus alternator) isn't turning

#if WINDOW
  #define windowtimeout              4000    // backup timeout to kill the window relay should the threashold never be tripped in ms
#endif // #if WINDOW

#if TURBOTIMER
  #define turbotimeout               30000   // maximum time car is allowed to idle in worst-case turbo temp scenario in ms
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
