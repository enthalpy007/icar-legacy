#include "TimerThree.h"
#include <string.h>
#include "Engine_Simulator_Serial.h"

bool debug = 1;
bool accIn = 0;                             // is the accessory relay energized?
bool ignIn = 0;                             // is the ignition relay energized?
bool startIn = 0;                           // is the starter relay energized?
bool forceEnginetoRunningState = 0;         // is the engine forced to run regardless of inputs?
int engineSpeed = 0;                        // track the engine speed
int previousEngineSpeed = 0;                // tracks the previous engine speed for serial reporting purposes
long tachPeriod;                            // tracks the period of the tach signal in microseconds
bool valueStatus = LOW;                     // value of the status LED
long previousMillisStatus = 0;              // will store last time LED was updated
long crankToStartCounter = 0;               // tracks the amount of time the motor has been cranking
int serialData = normalEngine;              // tracks the serial serialData, which will only be one character
                                            // 'm' = malfunction, no tach signal will be presented
                                            // 'c' = cranking indefinitely, engine won't fire up
                                            // 'n' = normal engine operation
char keyPosition = 'o';                     // tracks the position of the key
                                            // 'o' = off
                                            // 'a' = accessory
                                            // 'i' = ignition
                                            // 's' = start
                                            // 'e' = error!
char* printableserialData[] = {
  "normal",                                 // normal operation
  "no tach",                                // no tach signal shall ever be presented
  "cranking indefinitely"                   // the tach signal will not be allowed to exceed cranking RPM
};
char previousKeyPosition;                   // tracks the previous key's position to see if it changed
int operatingEngineSpeed = idleEngineSpeed; // tracks the speed of the engine when it's allowed to run
bool tachstate;                             // flag for the timer ISR to flip the tach output pin

int alarmState = 0;                         // tracks the current state of the alarm. This also tracks the state of the door locks
                                            // 0 = disarmed
                                            // 1 = armed
                                            // 2 = pending armed
long alarmTimer = 0;                        // bit banging the alarm LED
bool alarmFobIn = 0;                        // tracks if the car controller has "pressed" the alarm fob button
bool previousAlarmFobIn = 0;                // tracks the previously recorded alarm fob button status
bool alarmLEDStatus = 0;                    // tracks the alarm LED's state

bool bluetoothStateIn = 0;                  // input pin hooked to a switch that tracks the state of the bluetooth
                                            // 1 = associated
                                            // 0 = unassociated
bool bluetoothLEDState = 0;                 // tracks the LED's output state for bit banging
long bluetoothLEDTimer = 0;                 // timer for bit banging the output LED

#if timeMainLoop
  long timeStamp = 0;
  long previousTimeStamp = 0;
  int deltaTimeStamp = 0;
  int previousDeltaTimeStamp = 0;
#endif

void setup()
{
  pinMode(tachOut, OUTPUT);
  pinMode(statusOut, OUTPUT);
  pinMode(alarmLEDOut, OUTPUT);
  pinMode(bluetoothOut, OUTPUT);
  digitalWrite(bluetoothOut, LOW); // tie it low so the engine controller's input doesn't float
  Serial.begin(9600);
  Timer3.initialize();
  Timer3.attachInterrupt(timerisr);
}

void loop()
{
  #if timeMainLoop
    calculateLoopTime();
  #endif
  readInputs();
  if (Serial.available() > 0)
  {
    serialData = Serial.read();
  } else {
    serialData = 0;
  }
  figureOutKeyPosition();
  figureOutEngineSpeed();
  processTachOutput();
  processStatusLED();
  figureOutBluetoothState();
  figureOutAlarmState();
  reportSerialData();
  resetPreviousVariables();
}

#if timeMainLoop
  void calculateLoopTime() {
    timeStamp = millis();
    deltaTimeStamp = timeStamp - previousTimeStamp;
    if (deltaTimeStamp - previousDeltaTimeStamp > 5) {
      Serial.println(deltaTimeStamp);
    }
    previousTimeStamp = timeStamp;
    previousDeltaTimeStamp = deltaTimeStamp;
  }
#endif

void readInputs() {
  accIn = digitalRead(accessoryIn); // A HIGH value means the accessory position was turned ON
  ignIn = digitalRead(ignitionIn); // A HIGH value means the ignition position was turned ON
  startIn = digitalRead(starterIn); // A HIGH value means the start position was turned ON
  alarmFobIn = digitalRead(alarmFobInput); // A LOW value means the button is being PRESSED
  bluetoothStateIn = digitalRead(bluetoothStateInput); // A LOW value means UNASSOCIATED
}

void reportSerialData() {
  if (keyPosition != previousKeyPosition) {
    Serial.print("key position is now ");
    if (keyPosition == 'o') Serial.println("off");
    if (keyPosition == 'a') Serial.println("accessory");
    if (keyPosition == 'i') Serial.println("ignition");
    if (keyPosition == 's') Serial.println("start");
    if (keyPosition == 'e') Serial.println("ERROR!");
  }
  if (keyPosition == 's' && previousKeyPosition != 's' && engineSpeed > crankingEngineSpeed)
    Serial.println("ERROR!!  ENGINE RUNNING, GRINDING STARTER!!");
  if (previousEngineSpeed != engineSpeed) {
    Serial.print("engine speed is ");
    Serial.println(engineSpeed);
  }
  if (serialData != 0) {
    switch (serialData) {
      case normalEngine:
        Serial.print("simulator state is ");
        Serial.println(printableserialData[0]);
        break;
      case malfunction:
        Serial.print("simulator state is ");
        Serial.println(printableserialData[1]);
        break;
      case crankIndefinitely:
        Serial.print("simulator state is ");
        Serial.println(printableserialData[2]);
        break;
      case tachCruise:
        operatingEngineSpeed = cruiseEngineSpeed;
        Serial.print("engine speed is changed to ");
        Serial.println(operatingEngineSpeed);
        break;
      case tachIdle:
        operatingEngineSpeed = idleEngineSpeed;
        Serial.print("engine speed is changed to ");
        Serial.println(operatingEngineSpeed);
        break;
      case forceEngineRunning:
        forceEnginetoRunningState = !forceEnginetoRunningState;
        Serial.print("engine forced state is ");
        Serial.println(forceEnginetoRunningState);
        break;
    }
  }
}

void figureOutEngineSpeed() {
  if (serialData == malfunction || keyPosition == 'o' || keyPosition == 'a') { // malfunction or engine stall - no tach
    engineSpeed = 0;
  }
  else if (serialData == crankIndefinitely && keyPosition == 's') // engine cranking indefinitely but won't start
    engineSpeed = crankingEngineSpeed;
  else if (serialData == crankIndefinitely && keyPosition == 'i' && previousKeyPosition == 's') // unsuccessful start, engine should spin down after releasing starter
    engineSpeed = 0;
  else if (keyPosition == 'i' && engineSpeed == 0) // the engine is about to fire up, reset the cranking timer
    crankToStartCounter = millis();
  else if ((keyPosition == 'i') && (engineSpeed > 0)) // the engine is successfully running, keep the tach going
    engineSpeed = operatingEngineSpeed;
  else if (keyPosition == 's') { // engine cranking during normal start period
    if (millis() - crankToStartCounter < crankToStartTime) // engine cranking timer hasn't expired, engine only cranking
      engineSpeed = crankingEngineSpeed;
    else if (millis() - crankToStartCounter >= crankToStartTime) // engine cranking timer DID expire, fire and idle engine
      engineSpeed = operatingEngineSpeed;
  }
  if (forceEnginetoRunningState > 0) {
    engineSpeed = operatingEngineSpeed;
  }
}

void figureOutKeyPosition() {
  if (accIn == LOW && ignIn == LOW && startIn == LOW)
    keyPosition = 'o';
  if (accIn == HIGH && ignIn == LOW && startIn == LOW)
    keyPosition = 'a';
  //if (accIn == LOW && ignIn == HIGH && startIn == LOW)
    //keyPosition = 'e';
  if (accIn == LOW && ignIn == LOW && startIn == HIGH)
    keyPosition = 'e';
  if (accIn == HIGH && ignIn == HIGH && startIn == LOW)
    keyPosition = 'i';
  if (accIn == LOW && ignIn == HIGH && startIn == HIGH)
    keyPosition = 's';
  if (accIn == HIGH && ignIn == HIGH && startIn == HIGH)
    keyPosition = 'e';
}

void processTachOutput() {
  if ((engineSpeed > 0) && (previousEngineSpeed != engineSpeed)) {
    tachPeriod = 1000000 / (engineSpeed / 15); // convert engine speed to tachometer period in microseconds assuming a 4-cylinder 4-cycle
    Timer3.pwm(9, 512, tachPeriod); // pwm pin 10 (don't ask me why), "512" = 50% duty cycle
  }
}

void processStatusLED() {
  if (keyPosition == 'i' || keyPosition == 's')
    digitalWrite(statusOut, HIGH);
  else if (keyPosition == 'a') {
    if (millis() - previousMillisStatus > statusInterval) { // flop LED every statusInterval
      if (valueStatus == LOW)
        valueStatus = HIGH;
      else
        valueStatus = LOW;
      digitalWrite(statusOut, valueStatus);
      previousMillisStatus = millis(); 
    }
  }
  else if (keyPosition == 'e') {
    if (millis() - previousMillisStatus > errorStatusInterval) { // flop LED every errorStatusInterval
      if (valueStatus == LOW)
        valueStatus = HIGH;
      else
        valueStatus = LOW;
      digitalWrite(statusOut, valueStatus);
      previousMillisStatus = millis(); 
    }
  }
  else // key position is off
    digitalWrite(statusOut, LOW);
}

void figureOutBluetoothState() {
  if (!bluetoothStateIn) {bluetoothLEDState = 0;} // unassociated, turn off output
  if ((millis() - bluetoothLEDTimer > bluetoothTimeout) && bluetoothStateIn) // time's up, flip the output only if associated
  {
    bluetoothLEDState = !bluetoothLEDState;
    bluetoothLEDTimer = millis();
  }
  digitalWrite(bluetoothOut, bluetoothLEDState);
}

void figureOutAlarmState() {
  /*
  Alarm State:
   0 = disarmed
   1 = armed, doors are locked
   2 = pending armed
   */
  if (engineSpeed == 0)
  {
    if (alarmFobIn && !previousAlarmFobIn) // only trigger on rising edge of alarm fob input
    {
      switch (alarmState)
      {
        case 0:
          alarmState = 1;
          alarmTimer = millis();
          break;
        case 1:
          alarmState = 0;
          alarmLEDStatus = LOW;
          break;
      }
    }
    
    if ((alarmState == 1) && (millis() - alarmTimer > alarmTimeout)) // time the bit banging of the output LED if alarm is armed
    {
      alarmLEDStatus = !alarmLEDStatus;
      alarmTimer = millis();
    }
  }
  else {alarmLEDStatus = 0;}
    
  digitalWrite(alarmLEDOut, alarmLEDStatus);
}

void resetPreviousVariables() {
  previousKeyPosition = keyPosition;
  previousAlarmFobIn = alarmFobIn;
  previousEngineSpeed = engineSpeed;
}

void timerisr() {
  tachstate = !tachstate;
  if (engineSpeed > 0)
    digitalWrite(tachOut, tachstate); // toggle the value of pin 10
  else
    digitalWrite(tachOut, HIGH); // typical EMSes have pull-up resistors with pull-low outputs
}
