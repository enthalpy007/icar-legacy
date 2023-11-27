// ************ PIN ASSIGNMENTS **************
#define accessoryIn         48  // accessory input pin maps to pin 12, HIGH means ON
#define ignitionIn          50  // ignition input pin maps to pin 18, HIGH means ON
#define starterIn           52  // starter input pin maps to pin 19, HIGH means ON
#define alarmFobInput       49  // alarm fob input LOW value means button being PRESSED
#define bluetoothStateInput 22  // bluetooth input a LOW value means UNASSOCIATED
#define alarmLEDOut         12  // pin for alarm LED output
#define bluetoothOut        47  // output pin for bluetooth LED
#define statusOut           13  // status LED output
#define tachOut             2   // tach output (PWM)

// ************ SERIAL COMANDS ***************
#define malfunction         'm' // malfunction, no tach signal will be presented
#define crankIndefinitely   'c' // crank indefinitely, engine won't fire up
#define normalEngine        'n' // normal engine operation
#define forceEngineRunning  'f' // forces the engine to a running state regardless of inputs
#define tachCruise          '3' // cruise tach of 2000 RPM
#define tachIdle            '1' // idle tach of 1000 RPM

// ************ TIMING VALUES ****************
#define crankToStartTime   2000 // time it takes to fire the motor
#define errorStatusInterval 200 // interval at which to pulse the LED (milliseconds) IN ERROR STATE
#define statusInterval     1000 // interval at which to pulse the LED (milliseconds)
#define alarmTimeout        500 // timeout for flipping the alarm LED output
#define bluetoothTimeout   1000 // bluetooth output LED switching period (in ms)

// ************ PROCESS VARIABLES ************
#define crankingEngineSpeed 300 // cranking RPM
#define idleEngineSpeed    1000 // idle RPM
#define cruiseEngineSpeed  3000 // cruise RPM
#define timeMainLoop          1 // run the calculation for main loop timing (serial port approximation)
