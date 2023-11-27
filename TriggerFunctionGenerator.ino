#define NETeethPerGTooth 24
#define NEOutput 7
#define GOutput 8
#define LEDPIN 13
#define crankTimeout 5000 // microseconds per NE tooth
#define toothDuration 2000 // NE and G tooth duration in us
#define toothOutputOn HIGH
#define toothOutputOff LOW
#define ShutDownTime 30000 // 30 seconds in ms
int toothOutputState = 0;
long crankTimer = 0;
long toothTimer = 0;
int NETeethSinceLastG = 0;
void setup()
{
  //Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT); // LED
  digitalWrite(LEDPIN, HIGH);
  pinMode(NEOutput, OUTPUT); // NE
  digitalWrite(NEOutput, toothOutputOff);
  pinMode(GOutput, OUTPUT); // G
  digitalWrite(GOutput, toothOutputOff);
  crankTimer = micros();
}

void loop()
{
  if (toothOutputState == 0 && (micros() - crankTimer > crankTimeout))
  {
    //Serial.println("1");
    digitalWrite(NEOutput, toothOutputOn);
    crankTimer = micros();
    toothTimer = crankTimer;
    NETeethSinceLastG++;
    toothOutputState = 1;
    if (NETeethSinceLastG >= NETeethPerGTooth)
    {
      digitalWrite(GOutput, toothOutputOn);
      toothOutputState = 1;
      NETeethSinceLastG = 0;
    }
  }
  if (toothOutputState == 1 && (micros() - toothTimer > toothDuration))
  {
    //Serial.println("0");
    digitalWrite(NEOutput, toothOutputOff);
    digitalWrite(GOutput, toothOutputOff);
    toothOutputState = 0;
  }
  if (millis() > ShutDownTime)
  {
    digitalWrite(LEDPIN, LOW);
    digitalWrite(NEOutput, toothOutputOff);
    digitalWrite(GOutput, toothOutputOff);
    for (;;){}
  }
}
