#include "CarBluetooth_Teensy.h"

#if WATCHDOG
  #include <avr/wdt.h>
#endif // #if WATCHDOG

#if SLEEPMODE
  #include <avr/power.h>
  #include <avr/sleep.h>
#endif // #if SLEEPMODE

#if CANBUS
  #include <FlexCAN.h>
#endif // #if CANBUS

// ****************** OBJECT INSTANTIATIONS **************
#if CANBUS
  FlexCAN CANbus(500000);
#endif // #if CANBUS

// ****************** DEBUG SWITCHES *********************
bool MASTERDEBUG = 0;                                 // master switch for serial debug messages
int tempvar = 0;;
int previoustempvar = tempvar;

// ****************** DEBUG VARIABLES ********************
bool brakeStatus = 0;                                 // tracks the state of the brake pedal
bool previousBrake = 0;                               // allows for updating the switch positions only when they change
char previousKeyState = 'o';                          // allows for updating the key state only when they change
int previousEngineSpeed = 1000;                       // allows for updating the engine speed only when it changes
bool previousEngineRunning = 0;                       // tracks the previous value of engineRunning
long debugprinttimer = millis();

// ****************** PROCESS VARIABLE ASSIGNMENTS *********************
int serialData;                                       // tracks the read data from the UART0 serial port
long looptimer = 0;                                   // tracks when millis() rolls over for reset of all timer variables
long tachTimestamp = 0;                               // tracks when the tachometer signal goes low using an interrupt
long previousTachTimeStamp = 0;                       // tracks the previous time the tachometer ISR fired
int tachPeriod = 0;                                   // tracks the difference in tach timestamps
int previoustachPeriod = 0;                           // tracks the previous difference in tach timestamps
bool tachEvent = 0;                                   // tracks if a tach signal has ever happened, handles initial startup scenario
int engineSpeed;                                      // tracks the engine speed in crank revs / minute
long errorRecoverTimer = 0;                           // tracks the timeout waiting for the starter to cool off
bool allowEngineStart = 1;                            // tracks if the user is allowed an engine start
bool engineRunning = 0;                               // tracks if the engine is above thraeshold speed to be considered running
bool millisoverflow = 0;                              // tracks if the user has been notified that millis() overflowed
static unsigned long last_interrupt_time = 0;         // timestamps the previous power button interrupt
int EngineStart = 0;                                  // tracks if the engine started successfully or not
                                                      // 0 = haven't attempted to start yet
                                                      // 1 = attempted to start, timeout reached without engine firing
                                                      // 2 = attempted to start, MALFUNCTION DETECTED
                                                      // 3 = attempted to start, was successful
                                                      // 4 = start sequence aborted with subsequent pressing of the power button

char keyPosition = 'o';                               // tracks the current key position
                                                      // 'o' = off
                                                      // 'a' = accessory
                                                      // 'i' = ignition
                                                      // 's' = start

volatile int powerState = LOW;                        // tracks the recognized state of the power switch
int powerButtonCounter = 0;                           // counter that is used to reset the power button during certain race conditions
char previousKeyPosition = 'o';                       // tracks changes in the key position

#if TURBOTIMER
  int tpszerocal = 0;                                 // calibration for recognizing when the throttle is shut
  long zerotpstimer = 0;                              // timer that keeps track of how long the engine has been at idle for adequate turbo cooling
#endif // #if TURBOTIMER

#if LCDDISPLAY
  bool LCDPower = 0;                                  // tracks if the LCD is powered up
  bool delayLCDPrint = 0;                             // tracks if the LCD print needs to be delayed for startup screen
  bool LCDBackLightStatus = 0;                        // tracks the state of the LCD back light
  long lcdPowerTimer = 0;                             // tracks how long the LCD has been powered up with the key off
  char* lcdline1;                                     // tracks the LCD message, line 1
  char* lcdline2;                                     // tracks the LCD message, line 2
#endif // #if LCDDISPLAY

#if AUTHENTICATION
  bool userAuthenticated = 1;                         // is the user authenticated?
  bool previousUserAuthenticated = 1;                 // tracks if the user authentication changed
  bool previousUserAuthenticatedDebug = 1;            // tracks the user authentication for debug output
  bool motionSensorPower = 0;                         // tracks if the motion sensor is powered up
  bool previousMotionSensorPower = 0;                 // tracks if the motion sensor power has been changed
  int dimPowerButton = B00;                           // tracks if the power button back light is being dimmed or not
                                                      // B00 = off
                                                      // B01 = get brighter
                                                      // B10 = get dimmer
                                                      // B11 = on
  long dimTimer = 0;                                  // tracks the progress of the dimmer

  long fobButtonPressTimestamp = 0;                   // tracks when the fob button was pressed to prevent repeated pressings
  long alarmTimestamp = 0;                            // timestamps the alarm interrupt event
  int alarmState = 0;                                 // tracks the current state of the alarm. This also tracks the state of the door locks
                                                      // 0 = disarmed
                                                      // 1 = armed
                                                      // 2 = pending armed
  int previousAlarmState = 0;                         // tracks the state of the alarm for debug purposes
#endif // #if AUTHENTICATION

#if MOTIONSENSOR
  int motionSensorTriggerEventCount = 0;              // tracks how often the motion sensor is triggered
  int previousMotionSensorTriggerEventCount = 0;      // tracks the previous motion sensor counter for debug
  #if !BLUETOOTH
    long motionSensorTimer = 0;                       // times the motion sensor's trigger events when bluetooth isn't present in the system
  #endif // #if !BLUETOOTH
#endif // #if MOTIONSENSOR

#if BLUETOOTH
  long bluetoothTimestamp = 0;                        // timestamps the bluetooth interrupt event
  bool bluetoothState = 0;                            // tracks the current state of the bluetooth. 
                                                      // 0 = disassociated
                                                      // 1 = associated
  bool previousBluetoothState = 0;                    // tracks the state of the bluetooth for debug purposes
#endif // #if BLUETOOTH

#if REPORTMEMORY
  int previousMemory = 0;                             // tracks changes in the available memory for debug purposes
  bool reportMemory = 0;                              // tracks if the memory should be reported; this is expensive processor-wise
#endif // #if REPORTMEMORY

#if SLEEPMODE
  long sleepTimer = millis();                         // tracks how long before sleep mode is entered
  bool sleepModeEnabled = 1;                          // allows user to turn off sleep mode during debug
#endif // #if SLEEPMODE

#if CANBUS
  static CAN_message_t msg,rxmsg;
#endif // #if CANBUS

#if TIMEMAINLOOP
  int mainLoopTimerPinState = 1;
#endif // #if TIMEMAINLOOP

// ***************************************** SETUP ******************************************
void setup() 
{
  Serial.begin(9600);                                 // UART0 uses the USB port
  setInputPinPullups();                               // sets pull-ups on all input pins for power consumption reduction
  pinMode(accRelay, OUTPUT);
  pinMode(ignRelay, OUTPUT);
  pinMode(starterRelay, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(amberLED, OUTPUT);
  pinMode(backlightLED, OUTPUT);
  pinMode(brakeIn, INPUT);
  attachInterrupt(switchIn, powerButtonISR, LOW);     // push button interrupt
  attachInterrupt(tachIn, tachISR, RISING);           // tachometer

  #if LCDDISPLAY
    Serial2.begin(9600);                              // UART2 for LCD message screen
    pinMode(LCDPowerPin, OUTPUT);                     // pin that controls LCD power
    digitalWrite(LCDPowerPin, relayOn);               // turn on LCD display
    LCDPower = 1;
    delay(750);
    backlightOn();
    clearLCD();
    delay(750);
    delayLCDPrint = 1;
    splashScreen("Initializing","System");
    delay(250);
    clearLCD();
    printLCD("Initialization","Complete");
  #endif // #if LCDDISPLAY
  
  #if WINDOW
    pinMode(windowOut, OUTPUT);                       // output for window control relay
    pinMode(windowPowerOut, OUTPUT);                  // output for energizing the window circuit
  #endif // #if WINDOW
  
  #if BLUETOOTH
    attachInterrupt(alarmLEDPin, alarmISR, FALLING);  // alarm LED fires an interrupt for timing the LED
    attachInterrupt(bluetoothLEDIn, btISR, FALLING);  // alarm LED fires an interrupt for timing the LED
  #endif // #if BLUETOOTH
  
  #if AUTHENTICATION
    pinMode(alarmPin, OUTPUT);                        // pin that toggles the alarm fob connection to arm / disarm alarm
  #endif // #if AUTHENTICATION
  
  #if MOTIONSENSOR
    pinMode(motionPowerPin, OUTPUT);
    powerMotionSensor(1);                             // power up the sensor on bootup
  #endif // #if MOTIONSENSOR
  
  #if WATCHDOG
    wdt_enable(WDTO_8S);                              // watchdog timer set to 8 seconds
  #endif // #if WATCHDOG
  
  #if SLEEPMODE
    sleepTimer = millis();
  #endif // #if SLEEPMODE

  #if TIMEMAINLOOP
    pinMode(mainLoopTimerPin, OUTPUT);
    digitalWrite(mainLoopTimerPin, mainLoopTimerPinState);
  #endif // #if TIMEMAINLOOP

/*extern volatile unsigned long timer0_millis;
noInterrupts();
timer0_millis = 0xFFFFB1DF; // 20 seconds before rollover
interrupts();*/
} // void setup()

/*
########################################################
TO DO:
- IRSENSOR Control Code
  -> Use a timed coded tap sequence?
  -> Normalize against averaged background light

- Sleep Mode Interactions
  -> Figure out which features don't jive with sleep mode

- LCD Bugs:
  -> Display cuts out during turbo timer timedown
  -> Display cuts when the key is to accessory (could be sleep mode?)
########################################################
*/

// ***************************************** MAIN LOOP ******************************************

void loop() {
  if (millis() < looptimer) // millis() rolled over
  {
    noInterrupts();
    millisoverflow = 1;
    previousTachTimeStamp = millis();
    errorRecoverTimer = millis();
    tachTimestamp = millis();
    #if TURBOTIMER
      zerotpstimer = millis();
    #endif // #if TURBOTIMER
    #if LCDDISPLAY
      lcdPowerTimer = millis();
    #endif // #if LCDDISPLAY
    #if AUTHENTICATION
      dimTimer = millis();
      alarmTimestamp = millis();
    #endif // #if AUTHENTICATION
    #if BLUETOOTH
      long bluetoothTimestamp = millis();
    #endif // #if BLUETOOTH
    #if MOTIONSENSOR && !BLUETOOTH
      motionSensorTimer = millis();
    #endif // #if MOTIONSENSOR
    #if SLEEPMODE
      sleepTimer = millis();
    #endif // #if SLEEPMODE
    interrupts();
  }
  looptimer = millis();

  #if TIMEMAINLOOP
    mainLoopTimerPinState = !mainLoopTimerPinState;
    digitalWrite(mainLoopTimerPin, mainLoopTimerPinState);
  #endif

  if (Serial.available() > 0)
  {
    serialData = Serial.read();
    Serial.flush();
  }
  else // no serial data available
    serialData = 0;

  if (powerState == HIGH) {
    powerButtonCounter++;
    if (powerButtonCounter >= resetPowerStateCounter) 
      powerState = LOW; // reset the power button's state if the main loop has iterated resetPowerStateCounter times with powerState = HIGH
  }
  else
    powerButtonCounter = 0;

  calculateEngineSpeed();
  bool brakeStatus = digitalRead(brakeIn);

  #if WATCHDOG
    wdt_reset();          // Pat the dog
  #endif // #if WATCHDOG

  if (allowEngineStart == 0 && (millis() - errorRecoverTimer >= errorRecoveryTimeout)) {
    allowEngineStart = 1; // allowEngineStart gets set to zero if the starter is at risk of overheat. Force a wait of 10 seconds.
    if (MASTERDEBUG) {
      Serial.println("Flipping allowEngineStart back to 1");
    }
    #if LCDDISPLAY
      printLCD("Starter","Re-Enabled");
    #endif // #if LCDDISPLAY
  }
  
  #if SLEEPMODE
    if (brakeStatus) sleepTimer = millis();
  #endif // #if SLEEPMODE
  
  if (brakeStatus == HIGH && powerState == HIGH && allowEngineStart == 1) {
    if (engineRunning == 0) {
      EngineStart = start_sequence();
      keyPosition = 's';
      switch (EngineStart)
      {
        case 1:
        case 2:
          allowEngineStart = 0;
          errorRecoverTimer = millis();
          keyPosition = 'i';
          engineRunning = 0;
          #if LCDDISPLAY
            printLCD("ERROR OCCURED","PLEASE WAIT");
          #endif // #if LCDDISPLAY
          break;

        case 3:
          keyPosition = 'i';
          engineRunning = 1;
          #if LCDDISPLAY
            printLCD("ENGINE STARTED","SUCCESSFULLY");
          #endif // #if LCDDISPLAY
          break;

        case 4:
          keyPosition = 'a';
          engineRunning = 0;
          #if LCDDISPLAY
            printLCD("ENGINE START","ABORTED");
          #endif // #if LCDDISPLAY
          break;
      }
    }
    else if (engineRunning == 1) // if the engine is running and the user wants to kill the motor without killing the accessories, use brake pedal
      keyPosition = 'a';
    powerState = LOW;
    LED_relay_control(keyPosition);
  }
  if (brakeStatus == LOW && powerState == HIGH) { // power switch is pressed, brake is NOT pressed
    switch (keyPosition) 
    {
      case 'o':                // key is off, go to accessory
        keyPosition = 'a';
        break;
      case 'a':                // key is at accessory, go to ignition
        keyPosition = 'i';
        break;
      case 'i':                // key is at ignition, turn off
        #if TURBOTIMER
          if (engineRunning == 1) {timeDownTurbo();}
        #endif // #if TURBOTIMER
        keyPosition = 'o';
        break;
    }
    powerState = LOW;
  }
  if (keyPosition != previousKeyPosition)
    LED_relay_control(keyPosition);
  
  if (serialData == debugSerialCharacter) 
  {
    #if SLEEPMODE
      sleepTimer = millis(); // reset the timer to give the user the chance to disable sleep mode
    #endif // if SLEEPMODE
    MASTERDEBUG = !MASTERDEBUG;
    Serial.print("Master Debug ");
    switch (MASTERDEBUG)
    {
      case 0:
        Serial.println("Disabled");
        break;
      case 1:
        Serial.println("Enabled");
        break;
    }
    #if SLEEPMODE
      Serial.println("Use 'd' to disable sleep mode");
    #endif // #if SLEEPMODE
  }
    
  if (MASTERDEBUG) {debug_function();}
  
  #if TURBOTIMER
    if (engineRunning == 1) {
      tpsCheck();
    }
  #endif // #if TURBOTIMER

  #if BLUETOOTH
    if (!engineRunning) {findBluetoothState();} // manipulates bluetoothState global
  #endif // #if BLUETOOTH
  
  #if MOTIONSENSOR
    if ((keyPosition == 'a') || (keyPosition == 'i')) 
    {
      powerMotionSensor(0); // if the user is authenticated, turn off the proximity sensor so it won't emit the radiation
      motionSensorTriggerEventCount = 0;
    }
    else // keyPosition is 'o'
    {
      findMotionState();
      #if !BLUETOOTH
        powerMotionSensor(1);
      #endif // #if !BLUETOOTH
    }
    
    #if !BLUETOOTH // this whole section is shooting for a certain number of triggers in a certain time in the absence of a bluetooth authentication mechanism
      if (millis() - motionSensorTimer < timeForAuthentication)
      {
        if (motionSensorTriggerEventCount > eventsForAuthentication)
        {
          userAuthenticated = !userAuthenticated;
          motionSensorTriggerEventCount = 0;
        }
      }
      else
      {
        motionSensorTriggerEventCount = 0;
      }
    #endif // #if !BLUETOOTH

    #if BLUETOOTH // this section is devoted to having both the motion sensor and bluetooth for authentication
      if ((motionSensorTriggerEventCount > 1) && (bluetoothState)) // bluetooth is associated and the motion sensor was triggered
      {
        userAuthenticated = 1;
        motionSensorTriggerEventCount = 0;
      }
      if (!bluetoothState) 
      {
        userAuthenticated = 0;
        motionSensorTriggerEventCount = 0;
        powerMotionSensor(0);
      }
      else
      {
        powerMotionSensor(1);
      }
    #endif // #if BLUETOOTH
  #endif // #if MOTIONSENSOR

  #if AUTHENTICATION
    if (engineRunning) 
    {
      userAuthenticated = 1;
      previousUserAuthenticated = 1;      
    }
    if (userAuthenticated != previousUserAuthenticated)
    {
      if (userAuthenticated)
      {
        powerUpLCD();
        printLCD("Driver","Authenticated");
        switch (dimPowerButton)
        {
           case B00:
             dimPowerButton = B01;
             break;
           case B10: // if the dimmer is going down and the user becomes re-authenticated, re-brighten.
             dimPowerButton = B01;
             break;
        }
      }
      
      else // !userAuthenticated
      {
        powerUpLCD();
        printLCD("Driver Not","Authenticated");
        switch (dimPowerButton)
        {
          case B11:
            dimPowerButton = B10;
            break;
          case B01:
            dimPowerButton = B00;
            break;
        }
      }
      dimTimer = millis();
    }
    
    if (dimPowerButton == B01 || dimPowerButton == B10)
    {
      powerButtonDim();
    }
    
    findAlarmState();

    if (userAuthenticated && (alarmState == 1 || alarmState == 2)) // user is allowed in, but the doors are still locked
    {
      pressAlarmButton();
    }
    if (!userAuthenticated && (alarmState == 0)) // user is not allowed in, but the doors are still open
    {
      pressAlarmButton();
    }
    /*NOTE - Not handling the case of alarmState == 2 because that state shouldn't be interrupted by
             the user still staying in bluetooth range*/

    previousUserAuthenticated = userAuthenticated;
  #endif // #if AUTHENTICATION

  #if LCDDISPLAY
    printDelayedMessages(); // Handles any messages that are in a queue to be printed as a result of the LCD needing a few seconds to power up
    if (engineRunning > 0) {
      digitalWrite(LCDPowerPin, relayOn);
      LCDPower = 1;
    }
    else if (millis() - lcdPowerTimer > lcdpowertimeout) // shut off LCD a certain amount of time after engine shuts off
    {
      digitalWrite(LCDPowerPin, relayOff);
      LCDPower = 0;
    }
    if (serialData == splashScreenResetCharacter) {setSplashScreen("   TOYOTA MR2   ","      TURBO");}
  #endif // #if LCDDISPLAY

  #if REPORTMEMORY
    if (serialData == memoryReportCharacter)
    {
      switch (MASTERDEBUG)
      {
        case 1:
          reportMemory = !reportMemory;
          break;
        case 0:
          reportMemory = 0;
          break;
      }
    }
  #endif // #if REPORTMEMORY
  
  #if SLEEPMODE
    if (serialData == disableSleepModeCharacter)
    {
      sleepModeEnabled = !sleepModeEnabled;
      sleepTimer = millis();
      if (MASTERDEBUG)
      {
        switch (sleepModeEnabled)
        {
          case 0:
            Serial.println("Sleep mode disabled");
            break;
          case 1:
            Serial.println("Sleep mode enabled");
            break;
        }
      }
    }
    if (sleepModeEnabled) 
    {
      if ((keyPosition == 'o') && (allowEngineStart) && (millis() - sleepTimer > sleepTimeout))
      {
        sleepNow();
      }
    }
  #endif // #if SLEEPMODE

  previousKeyPosition = keyPosition;
} // void loop()

// ***************************************** START SEQUENCE ******************************************
int start_sequence()
{
  #if LCDDISPLAY
    powerUpLCD();
    #if TURBOTIMER
      printLCD("Starting Engine","CLOSE THROTTLE");
      zerotpstimer = millis();
    #else
      printLCD("Starting Engine","");
    #endif // #if TURBOTIMER
  #endif // #if LCDDISPLAY
  #if SLEEPMODE
    sleepTimer = millis();
  #endif // #if SLEEPMODE
  powerState = LOW; // set the power button low for possible panic start sequence exit
  long crankingTimer;

  if (MASTERDEBUG) {Serial.println("Entering Start Sequence.");}

  digitalWrite(starterRelay, relayOff); // Make sure the starter motor has completely spun down before cranking
  digitalWrite(accRelay, relayOn); // Turn on the accessory key position and allow the EMS to initialize
  digitalWrite(ignRelay, relayOn); // Turn on the ignition key position and allow the EMS to initialize
  long temp = millis();
  while (millis() - temp < fuelPumpPrimeTimeout)
  {
    #if WATCHDOG
      wdt_reset();          // Pat the dog
    #endif // #if WATCHDOG
    if (powerState == HIGH) // if power button pressed during startup sequence, abort startup!
    {
      powerState = LOW;
      return EngineStart = 4;
    }
    #if LCDDISPLAY
      printDelayedMessages();
    #endif // #if LCDDISPLAY
  }
  
  if (MASTERDEBUG) {Serial.println("EMS primed, proceeding with startup");}
  
  #if LCDDISPLAY
    printLCD("Fuel System","Primed, Starting");
  #endif // #if LCDDISPLAY
  
  digitalWrite(accRelay, relayOff); // Turn off all accessories to allow max battery power available for starter
  digitalWrite(starterRelay, relayOn);
  long timestamp = millis();
  while (millis() - timestamp <= malfunctionTimeout)
  {
    #if WATCHDOG
      wdt_reset();          // Pat the dog
    #endif // #if WATCHDOG
    calculateEngineSpeed();
    #if LCDDISPLAY
      printDelayedMessages();
    #endif // #if LCDDISPLAY
    if (engineSpeed > 0) 
      break;
    if (powerState == HIGH) // if power button pressed during startup sequence, abort startup!
    {
      powerState = LOW;
      return EngineStart = 4;
    }
  }
  if (engineSpeed == 0) {                                  // no tach signal - malfunction detected - disengage starter and shut down!
    digitalWrite(starterRelay, relayOff);
    digitalWrite(accRelay, relayOff);
    digitalWrite(ignRelay, relayOff);
    
    if (MASTERDEBUG) {Serial.println("MALFUNCTION DETECTED");}
    
    EngineStart = 2;
  }
  crankingTimer= millis();
  while (millis() - crankingTimer < startSequenceTimeout) {
    #if WATCHDOG
      wdt_reset();          // Pat the dog
    #endif // #if WATCHDOG
    calculateEngineSpeed();
    #if LCDDISPLAY
      printDelayedMessages();
    #endif // #if LCDDISPLAY
    if (powerState == HIGH) { // if power button pressed during startup sequence, abort startup!
      powerState = LOW;
      EngineStart = 4;
      break;
    }
    if (engineSpeed == 0)
      break;
    if (engineSpeed < crankingThreashold)  // below engine firing threashold, engine is still cranking
      EngineStart = 1;
    if (engineSpeed >= crankingThreashold) // above engine firing threashold, release starter, start successful
    {
      digitalWrite(starterRelay, relayOff);
      digitalWrite(accRelay, relayOn);
      digitalWrite(ignRelay, relayOn);
      
      if (MASTERDEBUG) {Serial.println("engine started");}
      
      #if TURBOTIMER // calibrate the idle TPS reading
        int currentTPS = analogRead(tpsIn);
        if (currentTPS <= 3) {currentTPS = 4;}
        tpszerocal = 1.3 * currentTPS;
        if (MASTERDEBUG)
        {
          Serial.print("Calibrated TPS is ");
          Serial.println(tpszerocal);
        }
      #endif // #if TURBOTIMER
      
      EngineStart = 3;
      break;
    }
  }
  if (EngineStart == 1) // above while loop timed out without firing the engine, shut off everything, the starter is getting very hot!
  {
    digitalWrite(starterRelay, relayOff);
    digitalWrite(accRelay, relayOff);
    digitalWrite(ignRelay, relayOff);
    keyPosition = 'o';
    if (MASTERDEBUG) {Serial.println("cranking timeout reached without engine firing up");}
  }  
  return EngineStart;
} // int start_sequence()

// ***************************************** LED AND RELAY CONTROL ******************************
void LED_relay_control(char keyState) 
{
  switch (keyState)
  {
    case 'o':
      digitalWrite(greenLED,LEDOff);
      digitalWrite(amberLED,LEDOff);
      // -------------------------------------
      digitalWrite(starterRelay, relayOff);
      digitalWrite(ignRelay, relayOff);
      digitalWrite(accRelay, relayOff);
      engineRunning = 0;
      #if LCDDISPLAY
        powerUpLCD();
        printLCD("Key Position is","OFF");
      #endif // #if LCDDISPLAY
      #if SLEEPMODE
        sleepTimer = millis();
      #endif // #if SLEEPMODE
      break;

    case 'a':
      digitalWrite(greenLED,LEDOff);
      digitalWrite(amberLED,LEDOn);
      // -------------------------------------
      digitalWrite(starterRelay, relayOff);
      digitalWrite(accRelay, relayOn);
      digitalWrite(ignRelay, relayOff);
      engineRunning = 0;
      #if LCDDISPLAY
        powerUpLCD();
        printLCD("Key Position is","ACCESSORY");
      #endif // #if LCDDISPLAY
      break;

    case 'i':
      digitalWrite(greenLED,LEDOn);
      digitalWrite(amberLED,LEDOff);
      // -------------------------------------
      digitalWrite(starterRelay, relayOff);
      digitalWrite(accRelay, relayOn);
      digitalWrite(ignRelay, relayOn);
      #if LCDDISPLAY
        powerUpLCD();
        printLCD("Key Position is","IGNITION");
      #endif // #if LCDDISPLAY
      break;
  }
  #if LCDDISPLAY
    lcdPowerTimer = millis();
  #endif // #if LCDDISPLAY
  if (MASTERDEBUG && (keyState != previousKeyState)) {
    Serial.print("keyPosition is ");
    Serial.println(keyState);
  }
  previousKeyState = keyState;
} // void LED_relay_control

// ******************************* INTERRUPT TIMIMG CALCULATIONS ******************************
void calculateEngineSpeed()
{
  if (!tachEvent || (millis() - previousTachTimeStamp > engineNotRunningTachTimeout)) // no tach signal for engineNotRunningTachTimeout milliseconds
  {
    engineRunning = 0;
    engineSpeed = 0;
    tachEvent = 0;
    return;
  }
  
  if (engineRunning == 1) {
    tachPeriod = tachTimestamp - previousTachTimeStamp;
    //if ((tachPeriod - previoustachPeriod) > tachTimerDeltaThreshold) { // only bother with float math if necessary
      engineSpeed = 60000.00 / tachPeriod;
      //previoustachPeriod = tachPeriod;
    //}
  }
} // void calculateEngineSpeed()

#if AUTHENTICATION
  void findAlarmState()
  {
  /*
  Alarm State:
   0 = disarmed
   1 = armed, doors are locked
   2 = pending armed
   */
    if (millis() - alarmTimestamp > alarmLEDTimeout)
    {
      switch (digitalRead(alarmLEDPin))
      {
        case LOW: // the circuit is pull-low, so 'LOW' means the LED is OFF
          alarmState = 0;
          return;
        case HIGH: // the circuit is pull-low, so 'HIGH' means the LED is ON
          alarmState = 2;
          return;
      }
    }
    else
      {alarmState = 1;}
  } // void findAlarmState()
#endif // #if AUTHENTICATION

#if BLUETOOTH
  void findBluetoothState()
  {
    /*
    * Bluetooth LED blinks when it is associated, and is off when disassociated.
    * Since the circuit is pull-high, the LED being off actually means the input is 'HIGH'.
    */
    if (millis() - bluetoothTimestamp > bluetoothLEDTimeout) // Either solid or off - not associated
    {
      bluetoothState = 0;
      return;
    }
    else // is associated
      {bluetoothState = 1;}
  } // void findBluetoothState()
#endif // #if BLUETOOTH

// ******************************* INTERRUPT EVENT HANDLERS ******************************
void powerButtonISR() // fires when power button voltage goes low
{
  #if WATCHDOG
    wdt_enable(WDTO_8S); // sets watchdog timeout
  #endif // #if WATCHDOG
  if (millis() - last_interrupt_time > 80) // If interrupts come faster than 30ms, assume it's a bounce and ignore
  {
    powerState = HIGH;
  }
  last_interrupt_time = millis();
} // void powerButtonISR()

void tachISR() // fires when tach goes low on each ignition event
{
  previousTachTimeStamp = tachTimestamp;
  tachTimestamp = millis();
  if (engineRunning == 0) {
    previousTachTimeStamp = tachTimestamp;
  }
  engineRunning = 1;
  tachEvent = 1;
} // void tachISR()

#if AUTHENTICATION
  void alarmISR() // fires when alarm LED goes low
  {
    alarmTimestamp = millis();
  } // void alarmISR()
#endif // #if AUTHENTICATION

#if BLUETOOTH
  void btISR() // fires when Bluetooth LED input voltage goes low
  {
    bluetoothTimestamp = millis();
  } // void btISR()
#endif // #if BLUETOOTH

// ***************************************** DEBUG SECTION **********************************
void debug_function() {
  int temp = engineSpeed - previousEngineSpeed;
  if (millis() - debugprinttimer > 1000) { // print this every second
    Serial.print("Engine Speed is ");
    Serial.println(engineSpeed);
    Serial.print("tachPeriod is ");
    Serial.println(tachPeriod);
    debugprinttimer = millis();
  }
  if (abs(temp) >= 100) // report the engine speed if it has deviated more than 10 RPM
  {
    Serial.print("Engine Speed is ");
    Serial.println(engineSpeed);
    Serial.print("delta tach time is ");
    Serial.println(tachPeriod);
    previousEngineSpeed = engineSpeed;
  }
  if (millisoverflow)
  {
    Serial.println("Milliseconds overflowed!");
    millisoverflow = 0;
  }

  #if MOTIONSENSOR
    if (motionSensorTriggerEventCount != previousMotionSensorTriggerEventCount)
    {
      Serial.print("Proximity Sensor Trigger count is ");
      Serial.println(motionSensorTriggerEventCount);
    }
    if (motionSensorPower != previousMotionSensorPower)
    {
      Serial.print("Motion Sensor Power is ");
      Serial.println(motionSensorPower);
    }
    previousMotionSensorPower = motionSensorPower;
    previousMotionSensorTriggerEventCount = motionSensorTriggerEventCount;
  #endif // #if MOTIONSENSOR

  bool brake = digitalRead(brakeIn);
  if (brake != previousBrake) {
    Serial.print("Brake is ");
    Serial.println(brake);
  }
  previousBrake = brake;
  if (MASTERDEBUG && previousEngineRunning != engineRunning){
    Serial.print("engineRunning is ");
    Serial.println(engineRunning);
    previousEngineRunning = engineRunning;
  }
  if (EngineStart == 4) {
    Serial.println("PANIC during startup");
    EngineStart = 0;
  }

  #if AUTHENTICATION
    if (userAuthenticated != previousUserAuthenticatedDebug)
    {
      Serial.print("User Authenticated is ");
      Serial.println(userAuthenticated);
    }
    if (alarmState != previousAlarmState)
    {
      Serial.print("Alarm State is ");
      Serial.println(alarmState);
    }
    previousUserAuthenticatedDebug = userAuthenticated;
    previousAlarmState AUTHENTICATION= alarmState;
  #endif // #if AUTHENTICATION

  #if BLUETOOTH
    if (bluetoothState != previousBluetoothState)
    {
      Serial.print("Bluetooth State is ");
      Serial.println(bluetoothState);
    }
    previousBluetoothState = bluetoothState;
  #endif // #if BLUETOOTH

  #if REPORTMEMORY
    if (reportMemory)
    {
      int memory = availableMemory();
      if (memory != previousMemory)
      {
        Serial.print("Available Memory is ");
        Serial.println(memory);
      }
      previousMemory = memory;
    }
  #endif // #if REPORTMEMORY
} // void debug_function()

// ***************************************** MOTION SENSOR *****************************
#if MOTIONSENSOR
  void findMotionState()
  {
    if (analogRead(motionIn) < motionThreashold)
    {
      motionSensorTriggerEventCount ++;
      #if !BLUETOOTH
        if (motionSensorTriggerEventCount == 1)
        {
          motionSensorTimer = millis();
        }
      #endif // #if !BLUETOOTH
    }
  } // int findMotionSize()
  
  void powerMotionSensor(bool power)
  {
    if (power)
    {
      if (motionSensorPower) return; // if it's already powered up, don't bother toggling the power pin
      digitalWrite(motionPowerPin, relayOn);
      motionSensorPower = 1;
      motionSensorTriggerEventCount = 0;
    }
    if (!power)
    {
      if (!motionSensorPower) return; // if it's already powered down, don't bother toggling the power pin
      digitalWrite(motionPowerPin, relayOff);
      motionSensorPower = 0;
    }
  } // void powerMotionSensor()
#endif // #if MOTIONSENSOR

// ***************************************** WINDOW SENSOR ******************************
#if WINDOW
  void windowSensorHandler()
  {
    int windowvoltage = 0;          // tracks the voltage from the window hall effect sensor
    long windowtimer = 0;           // tracks the time elapsed the window has been moving
  } // void windowSensorHandler()
#endif // #if WINDOW

// ***************************************** TURBO TIMER *******************************
#if TURBOTIMER
  void timeDownTurbo()
  {
    powerState = LOW;
    if (MASTERDEBUG) {Serial.println("Turbo Timer Active.");}
    while(millis() - zerotpstimer <= turbotimeout - 1000) // It looks funny when "0 seconds until shut down" is displayed
    {
      if (powerState == HIGH)
      {
        powerState = LOW;
        return;
      }
      #if LCDDISPLAY
        char strTimeRemaining[5];
        float timeRemaining = (turbotimeout - (millis() - zerotpstimer)) / 1000.00;
        itoa(timeRemaining,strTimeRemaining,10);
        strcat(strTimeRemaining," seconds");
        printLCD("Turbo Timer",strTimeRemaining);
        delay(500);
      #endif // #if LCDDISPLAY
    }
  } // void timeDownTurbo()
  
  void tpsCheck() {
    int tpsvoltage = analogRead(tpsIn);
    if (tpsvoltage >= tpszerocal) {
      zerotpstimer = millis();
    }
  } // void tpsCheck()
#endif // #if TURBOTIMER 

// ***************************************** LCD ******************************************
#if LCDDISPLAY
  void printLCD(char line1[], char line2[])
  {
    if (sizeof(line1) > 16 || sizeof(line2) > 16) // NOTE - Each line can only be 16 characters long!!
      Serial2.print("INTERNAL ERROR  String too long");

    if (delayLCDPrint)  // wait for boot screen
    {
      lcdline1 = line1; // set the message to be printed into globals so we can come back after the boot up and print them.
      lcdline2 = line2;
      return;
    }
    
    clearLCD();
    Serial2.print(line1);
    goTo(16); // set cursor to second line
    Serial2.print(line2);
  } // void printLCD()
  
  void splashScreen(char line1[], char line2[])
  {
    clearLCD();
    // write line 1
    for(int t=0;t<16;t++) 
    {
      if (line1[t] == '\0') break;
      goTo(t);
      Serial2.print(line1[t]);
      delay(50);
    }
    // write line 2
    for(int t=0;t<16;t++) 
    {
      if (line2[t] == '\0') break;
      goTo(t+16);
      Serial2.print(line2[t]);
      delay(50);
    }
  } // void splashScreen()
  
  void clearLCD() 
  {
    Serial2.write(0xFE);
    Serial2.write(0x01);
  } // void clearLCD()
  
  void backlightOn() 
  {
    if (LCDBackLightStatus == 1)
      return;
    LCDBackLightStatus = 1;
    Serial2.write(0x7C);
    Serial2.write(157);
  } // void backlightOn()
  
  void backlightOff() 
  {
    Serial2.write(0x7C);
    Serial2.write(128);
  } // void backlightOff() 
  
  void goTo(int position) 
  {
    if (position < 16) 
    { 
      serCommand(); //command flag
      Serial2.write((position+128));
    } 
    else if (position < 32) 
    {
      serCommand(); //command flag
      Serial2.write((position+48+128));
    } 
    else 
    { 
      goTo(0); 
    }
  } // void goTo()
  
  void serCommand() 
  {
    Serial2.write(0xFE);
  } // void serCommand()
  
  void setSplashScreen(char splashLine1[], char splashLine2[])
  {
    //splashLine1 = "   TOYOTA MR2   ";
    //splashLine2 = "      TURBO";
    clearLCD();
    Serial2.print(splashLine1);
    Serial2.print(splashLine2);
    delay(1000);
    Serial2.write(0x7C);
    delay(100);
    Serial2.write(0x0A);
    delay(1000);
  } // void setSplashScreen()

  void powerUpLCD()
  {
    if (!LCDPower)
    {
      digitalWrite(LCDPowerPin,relayOn);
      LCDPower = 1;
      delayLCDPrint = 1;
      lcdPowerTimer = millis();
    }
  } // void powerUpLCD()
  
  void printDelayedMessages()
  /* Handles any messages that are in a queue to be printed as a result of the LCD needing a few seconds to power up*/
  {
    if ((millis() - lcdPowerTimer > LCDBootupTimeout) && delayLCDPrint == 1) // LCD boot up is finished, go ahead and display originally-intended message
    {
      delayLCDPrint = 0;
      printLCD(lcdline1, lcdline2);
    }
  } // void printDelayedMessages()
#endif // #if LCDDISPLAY

// ***************************************** USER AUTHENTICATION ******************************************
#if AUTHENTICATION
  void powerButtonDim()
  {
    long deltatime = millis() - dimTimer;
    if (deltatime < powerBacklightTimeout) // timeout hasn't expired, move the LED intensity up or down
    {
      float temp = 0.00;
      switch (dimPowerButton)
      {
        case B10:
          temp = 1.00 - (deltatime / powerBacklightTimeout);
          break;
        case B01:
          temp = deltatime / powerBacklightTimeout;
          break;
      }
      int intensity = temp * 255.00;
      analogWrite(backlightLED, intensity);
    }

    else
    {
      switch (dimPowerButton)
      {
        case B01:
          digitalWrite(backlightLED, relayOn);
          dimPowerButton = B11;
          break;
        case B10:
          digitalWrite(backlightLED, relayOff);
          dimPowerButton = B00;
          break;
      }
    }
  } // void powerButtonDim()

  void pressAlarmButton() 
  {
    if (millis() - fobButtonPressTimestamp < minTimeBetweenFobEvents) {return;} // Fob button being pressed too quickly - Button press disallowed.
    else
    {
      if (engineRunning) {return;}
      if (MASTERDEBUG) {Serial.println("Pressing Alarm Button");}
      digitalWrite(alarmPin, relayOn);
      delay(fobButtonPressDuration);
      digitalWrite(alarmPin, relayOff);
      fobButtonPressTimestamp = millis();
    }
  } // void pressAlarmButton()
#endif // #if AUTHENTICATION

#if REPORTMEMORY
  int availableMemory() {
    int size = 8192; // Use 2048 with ATmega328
    int *buf;
  
    while ((buf = (int *) malloc(--size)) == NULL)
      ;
  
    free(buf);
  
    return size;
  } // int availableMemory()
#endif // #if REPORTMEMORY

#if SLEEPMODE
  void sleepNow() 
  {
    if (MASTERDEBUG) {Serial.println("Entering sleep mode");}
    #if WATCHDOG
      wdt_disable(); // disables the timer entirely
    #endif // #if WATCHDOG
    #if LCDDISPLAY
      digitalWrite(LCDPowerPin, relayOff);
      LCDPower = 0;
    #elif MOTIONSENSOR
      powerMotionSensor(0);
    #endif
    delay(100);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();        // enables the sleep bit in the mcucr register
                           // so sleep is possible. just a safety pin 
    sleep_mode();          // here the device is actually put to sleep!!
    sleep_disable();       // first thing after waking from sleep:
                           // disable sleep...
    #if WATCHDOG
      wdt_enable(WDTO_8S);
    #endif // #if WATCHDOG
    #if LCDDISPLAY
      digitalWrite(LCDPowerPin, relayOn);
      LCDPower = 1;
      delayLCDPrint = 1;
    #endif
    sleepTimer = millis();
  }
#endif // #if SLEEPMODE

void setInputPinPullups()
{
  bool setPullup = 0;
  int inputPins = 46;
  #if defined (__AVR_ATmega328P__)
    inputPins = 13;
  #elif defined (__AVR_ATmega1280__)
    inputPins = 52;
  #endif
  int exceptionPins[] = {
    backlightLED,
    amberLED,
    greenLED,
    starterRelay,
    ignRelay,
    accRelay,
    brakeIn,
    statusLED,
    #if WINDOW
      windowOut,windowPowerOut,
    #endif
    #if LCDDISPLAY
      LCDPowerPin,
    #endif
    #if AUTHENTICATION
      alarmPin,
    #endif
    #if MOTIONSENSOR
      motionPowerPin,
    #endif
    #if IRSENSOR
      IRLEDOutput,
    #endif
  };
  for (int pin = 0; pin <= inputPins; pin++)
  {
    setPullup = 1;
    for (int excidx = 0; excidx < sizeof(exceptionPins)/sizeof(int); excidx++)
    {
      if (exceptionPins[excidx] == pin)
      {
        setPullup = 0;
        break;
      }
    }
    if (setPullup)
    {
      pinMode(pin, INPUT);
      digitalWrite(pin, HIGH);
    }
  }
}
