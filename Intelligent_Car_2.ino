#include "CarBluetooth_Teensy_2.h"

// ****************** DEBUG SWITCHES *********************
bool MASTERDEBUG = 0;                                 // master switch for serial debug messages

// ****************** PROCESS VARIABLE ASSIGNMENTS *********************
int previousEngineSpeed = 1000;                       // allows for updating the engine speed only when it changes
bool previousEngineRunning = 0;                       // tracks the previous value of engineRunning
int serialData;                                       // tracks the read data from the UART0 serial port
long millisRollover = 0;                              // tracks when millis() rolls over for reset of all timer variables
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
int relayPosition = KEY_OFF;                          // tracks the state of the relays
char previousRelayPosition = relayPosition;           // tracks changes in the state of the relays

/*
 * http://stackoverflow.com/questions/1371460/state-machines-tutorials
 * http://www.gedan.net/2008/09/08/finite-state-machine-matrix-style-c-implementation/
 * http://www.gedan.net/2009/03/18/finite-state-machine-matrix-style-c-implementation-function-pointers-addon/
 * MAIN SYSTEM STATES
 *                  power button with brake        power button withOUT brake     user authenticated          user deauthenticated
 * init             unauth, nil action             unauth, nil action             auth, nil action            unauth, nil action
 * unauth           auth, nil action               auth, nil action               auth, nil action            unauth, nil action
 * auth             start, start                   acc, acc relay on              auth, nil action            unauth, nil action
 * acc              start, start                   ign, ign/acc relays on         acc, nil action             acc, nil action
 * ign eng on       timer, acc/ign relays on       timer, acc/ign relays on       ign eng on, nil action      ign eng on, nil action
 * ign eng off      acc, ign/start relays off      off, all relays off            ign eng off, nil action     ign eng off, nil action
 * timer            acc, acc relay on              off, all relays off            timer, nil action           timer, nil action
 * start            acc, acc relay on/start abort  acc, acc relay on/start abort  start, nil action           start, nil action
 * *****************************************************************************
 * START SEQUENCE STATES
 *                  start request received         crank tach                     running tach                timer expires
 * not starting     prime, start prime timer       not starting, nil action       not starting, nil action    not starting, nil action
 * prime            prime, nil action              prime, nil action              not starting, nil action    crank no tach, display/start crank timer
 * crank no tach    crank no tach, nil action      crank, nil action              not starting, nil action    tach error, display / start hot starter timer
 * crank with tach  crank with tach, nil action    crank w/ tach, nil action      cal tps, nil action         hot starter, display / start hot starter timer
 * hot starter      hot starter, nil action        hot starter, nil action        not starting, nil action    not starting, display
 * tach error       tach error, nil action         crank with tach, nil action    not starting, nil action    not starting, nil action
 */

/*transition stateTransition[8][4] = {
  { {UNAUTH, NIL},    {UNAUTH, NIL},    {UNAUTH, NIL},      {UNAUTH, NIL} },       // INIT
  { {AUTH, NIL},      {AUTH, NIL},      {AUTH, NIL},        {UNAUTH, NIL} },       // UNAUTH
  { {START, START},   {ACC, KEY_ACC},   {AUTH,NIL},         {UNAUTH, NIL}, },      // AUTH
  { {START, START},   {IGN, KEY_IGN},   {ACC, NIL},         {ACC, NIL}, },         // ACC
  { {TIMER, KEY_IGN}, {TIMER, KEY_IGN}, {IGN_ENG_ON, NIL},  {IGN_ENG_ON, NIL}, },  // IGN ENG ON
  { {ACC, KEY_ACC},   {OFF, KEY_OFF},   {IGN_ENG_OFF, NIL}, {IGN_ENG_OFF, NIL}, }, // IGN ENG OFF
  { {ACC, KEY_ACC},   {OFF, KEY_OFF},   {TIMER, NIL},       {TIMER, NIL}, },       // TIMER
  { {ACC, KEY_ACC},   {ACC, KEY_ACC},   {START, NIL},       {START, NIL}, },       // START
}

transition startTransition[6][4] = {
  { {PRIME, START_PRIME_TIMER}, {NOT_START, NIL},  {NOT_START, NIL},     {NOT_START, NIL} },              // NOT STARTING
  { {PRIME, NIL},               {PRIME, NIL},      {NOT_START, NIL},     {CRANK_NO_TACH, CRANK_TIMER} },  // PRIME
  { {CRANK_NO_TACH, NIL},       {CRANK, NIL],      {NOT_START, NIL},     {TACH_ERROR, HOT_START_TIMER} }, // CRANK NO TACH
  { {CRANK_TACH, NIL},          {CRANK_TACH, NIL}, {NOT_START, CAL_TPS}, {HOT_START, HOT_START_TIMER} },  // CRANK WITH TACH
  { {HOT_START, NIL},           {HOT_START, NIL},  {NOT_START, NIL},     {NOT_START, DISPLAY_ERROR} },    // HOT STARTER
  { {TACH_ERROR, NIL},          {CRANK_TACH, NIL}, {NOT_START, NIL},     {NOT_START, NIL} },              // TACH ERROR
}*/

enum systemState {
  INIT,
  DEAUTH,
  AUTH,
  ACC,
  IGN_ENG_ON,
  IGN_ENG_OFF,
  TIMER,
  START,
};

enum startupState {
  NOT_STARTING,
  PRIME,
  CRANK_NO_TACH,
  CRANK_WITH_TACH,
  HOT_STARTER,
  TACH_ERROR,
};

enum {
  WITHBRAKE,
  NOBRAKE,
  AUTHENTICATED,
  DEAUTHENTICATED,
} event;

/*enum {
  NIL,
  ACC0_IGN0_START0,
  ACC1_IGN0_START0,
  ACC1_IGN1_START0,
  ACC0_IGN1_START1,
} action;*/

/*typedef struct {
  state nextState;
  action actionToDo;
} transition;*/

// ************ I/O Data Structures ************
struct inputData
{
  bool brakeSwitch;
  bool powerSwitch;
  int  tps;
  bool userAuthenticated;
};
inputData inputStates = {POWER_OFF, POWER_OFF, 0, 1};
inputData previousInputStates = {POWER_OFF, POWER_OFF, 0, 1};

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
  pinMode(powerIn, INPUT);
  attachInterrupt(tachIn, tachISR, RISING);           // tachometer
}

// ***************************************** MAIN LOOP ******************************************
void loop() {
  if (millis() < millisRollover) // millis() rolled over
  {
    noInterrupts();
    millisoverflow = 1;
    previousTachTimeStamp = millis();
    errorRecoverTimer = millis();
    tachTimestamp = millis();
    interrupts();
  }
  millisRollover = millis();

  readInputs();

  if (Serial.available() > 0) {
    serialData = Serial.read();
    Serial.flush();
  }
  else // no serial data available
    serialData = 0;

  if (engineRunning > 0) {
    calculateEngineSpeed();
  }

  if (hasEventHappened() == 1) {
    handleStateTransition();
  }
  
  setRelayStates();

  resetPreviousValues();
} // void loop()

int hasEventHappened() {
  if ((inputStates.powerSwitch == POWER_ON) || (inputStates.userAuthenticated != previousInputStates.userAuthenticated)) { return 1; }
  return 0;
}

void handleStateTransition() {
  
}

// ************** SET RELAY STATES BASED ON STRUCT *************
void setRelayStates() {
  /* Relay mask is (accessory, ignition, start)*/
  digitalWrite(accRelay, ((relayPosition >> 2) & 1));
  digitalWrite(ignRelay, ((relayPosition >> 1) & 1));
  digitalWrite(starterRelay, relayPosition & 1);
}

void readInputs() {
  inputStates.brakeSwitch = digitalRead(brakeIn);
  inputStates.powerSwitch = digitalRead(powerIn);
  #if TURBOTIMER
    if (relayPosition & 2) { // won't ever care about the TPS unless the ignition relay is on (2nd bit in the mask)
      inputStates.tps = analogRead(tpsIn);
    }
  #endif // #if TURBOTIMER
}

/**************** PROCESSOR-INTENSIVE CALCULATIONS ***************/
void calculateEngineSpeed()
{
  if (!tachEvent || (millis() - previousTachTimeStamp > engineNotRunningTachTimeout)) // no tach signal for engineNotRunningTachTimeout milliseconds
  {
    engineRunning = 0;
    engineSpeed = 0;
    tachEvent = 0;
    return;
  }
  tachPeriod = tachTimestamp - previousTachTimeStamp;
  if (engineRunning && ((tachPeriod - previoustachPeriod) > tachTimerDeltaThreashold)) // only bother with float math if necessary
  {
    engineSpeed = 60000.00 / tachPeriod;
    if (engineSpeed > crankingThreashold) {engineRunning = 1;}
  }
  previoustachPeriod = tachPeriod;
} // void calculateEngineSpeed()

/**************** INTERRUPT HANDLERS ******************/
void tachISR() // fires when tach goes low on each ignition event
{
  engineRunning = 1;
  tachEvent = 1;
  previousTachTimeStamp = tachTimestamp;
  tachTimestamp = millis();
} // void tachISR()

void setInputPinPullups()
{
  bool setPullup = 0;
  int inputPins = 46;
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

// ***************** MAIN LOOP CLEANUP FUNCTIONS *****************
void resetPreviousValues() {
  previousInputStates.brakeSwitch = inputStates.brakeSwitch;
  previousInputStates.powerSwitch = inputStates.powerSwitch;
  previousInputStates.tps = inputStates.tps;
  previousInputStates.userAuthenticated = inputStates.userAuthenticated;
  previousRelayPosition = relayPosition;
}

