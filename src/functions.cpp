#include <Arduino.h>
#include "functions.h"

float pressureSensor()
{
  int pressure = 0;
  float sensorValue = analogRead(A0); // reading analogue value at pin A0

  pressure = 1.1843 * sensorValue - 2.5289; // Linear relationship between the analog values and output pressure
  int divisible = 50;
  if (pressure % divisible < 5 || pressure > 480)
  {
    int val = map(pressure, 0, 1023, 0, 375);
    Serial.print("z0.val=");
    Serial.print(val);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }

  if (pressure < 6)
  { // too low of a pressure to capture - assuming 0 due to conversion error
    return 0;
  }
  else
  {
    return pressure;
  }
}

float inletPressureSensor()
{
  float pressureSend = 0;

  float sensorValue = analogRead(A1);
  pressureSend = sensorValue;

  if (sensorValue < 1)
  { // too low of a pressure to capture - assuming 0 due to conversion error
    return 0;
  }
  else
  {
    return pressureSend;
  }
}

void pressureAdjustment(float errorValue, int springDirPin, int springStepPin, int input, int output, int setpoint)
{
  Serial.println(" Entered automatic adjustment");
  Serial.print("errorValue : ");
  Serial.println(errorValue);
  if (errorValue > 0)
  { // Setting the direction for the motor (Current output pressure is greater than the setpoint)
    digitalWrite(springDirPin, LOW);
    digitalWrite(input, LOW);
    digitalWrite(output, HIGH);
    delay(200);
    Serial.println("Reversal"); // unwinds the spring housing for two rotations
    for (int i = 0; i <= 800; i++)
    { // 800 value is based on the step/rev setting
      digitalWrite(springStepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(springStepPin, LOW);
      delayMicroseconds(500);
    }
    delay(500);
  }
  else
  {
    Serial.println("Tightening"); // if the error value is less than 0 (Current output is lower than the setpoint)
    digitalWrite(springDirPin, HIGH);

    while (errorValue < -1)
    { // keeps winding the driver until the error is under the value of 1
      digitalWrite(springStepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(springStepPin, LOW);
      delayMicroseconds(500);

      float pressure_check = pressureSensor();
      errorValue = pressure_check - setpoint; // checking output pressure and comparing to error to see if condition of being within 1 is met
    }
  }

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  delay(500);

  Serial.println("Exiting adjustment --> back to loop function");
}

bool shutOffFunction(int spindleDirPin, int spindleStepPin, int input, int output, int shutLimit, int preShutLimit)
{

  bool coarse_tighten = true;

  Serial.print("t2.pco=63015");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("t2.txt=\"Spindle driver in action\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  if (coarse_tighten == true)
  {
    digitalWrite(spindleDirPin, HIGH);
    digitalWrite(input, LOW);
    digitalWrite(output, HIGH);
    delay(600);
    for (int j = 0; j < 9; j++)
    { // driver winds the spindle down. Value of 9 based on multiple tests to see number of rotations needed to complete shut off
      for (int i = 0; i <= 400; i++)
      {
        digitalWrite(spindleStepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(spindleStepPin, LOW);
        delayMicroseconds(500);
      }
    }
    Serial.println("Coarse tightening completed");
    coarse_tighten = false;
  }

  // The following code tests the functionality of the spindle to check for any pressure creep despite shut off
  Serial.print("t2.pco=63015");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("t2.txt=\"Testing shut off functionality\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  delay(500);

  float preSample = 0;
  float postSample = 0;

  preSample = pressureSensor();
  Serial.print("The outlet pressure after shut off (pre) : ");
  Serial.println(preSample);

  Serial.println("Checking for pressure creep after shut off");
  delay(10000);
  Serial.print("The outlet pressure after shut off (post) : ");
  postSample = pressureSensor();
  Serial.println(postSample);
  Serial.print("Difference : ");
  Serial.println(abs(preSample - postSample));

  if (abs(preSample - postSample) >= shutLimit || preSample > preShutLimit)
  {
    // Serial.println("Pressure leaked after shut off ---> shut off failed");
    Serial.print("t2.pco=63488");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.print("t2.txt=\"Shut off test failed\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    return false;
  }
  else
  {
    // Serial.println("Pressure not leaking after shut off ---> shut off passed");
    Serial.print("t2.pco=12000");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.print("t2.txt=\"Shut off test passed\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    return true;
  }
}

bool pressureDecay(int input, int output, int decayLim)
{
  float pressureTest = 0;
  float nextSample = 0;

  Serial.print("t2.pco=63015");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("t2.txt=\"Testing for leaks\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  delay(1000);

  digitalWrite(input, LOW); // Valve is now isolated and checked if it holds pressure

  pressureTest = inletPressureSensor(); // inlet pressure value used to check for leaks in the valve
  Serial.print("System pressure before test: ");
  Serial.println(pressureTest);
  delay(5000);
  nextSample = inletPressureSensor();
  Serial.print("System pressure after test: ");
  Serial.println(nextSample);

  digitalWrite(output, HIGH);
  delay(1000);

  Serial.print("Pressure decay: ");
  Serial.println(abs(nextSample - pressureTest));

  if (abs(nextSample - pressureTest) > decayLim)
  { // Any leaks above 50 are termed as leaks through the valve
    return false;
  }
  else
  {
    return true;
  }
}

bool springPressureHold(int springDirPin, int springStepPin, int input, int output, int holdingLim)
{
  float preSample = 0;
  float postSample = 0;

  digitalWrite(input, HIGH); // pressure held through the valve
  digitalWrite(output, LOW);
  delay(500);

  preSample = pressureSensor(); // outlet pressure value checked
  Serial.print("Pressure before pressure is adjusted : ");
  Serial.println(preSample);

  digitalWrite(springDirPin, HIGH); // winding the spring housing to check if its functioning correctly

  for (int i = 0; i <= 800; i++)
  {
    digitalWrite(springStepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(springStepPin, LOW);
    delayMicroseconds(500);
  }

  postSample = pressureSensor();
  Serial.print("Pressure after pressure is adjusted : ");
  Serial.println(postSample);

  if (abs(preSample - postSample) < holdingLim)
  {
    Serial.println("Test 2 : No pressure change noticed ---> pressure is not holding");
    return false;
  }
  else
  {
    Serial.println("Test 2 : Pressure change is considerable when spring is driven ---> pressure is holding");
    return true;
  }
}

bool springPressure(float pressureSetpoint, int creepLim)
{
  float postSample = 0;

  Serial.print("t2.pco=63015");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("t2.txt=\"Testing pressure creep\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.print("Pressure before delay to check for creep : ");
  Serial.println(pressureSetpoint); // pressureSetpoint variable is the mark at which the spring housing was set to
  delay(15000);

  postSample = pressureSensor(); // postSample is to check for any pressure creep once the output pressure has been set
  Serial.print("Pressure after delay to check for creep : ");
  Serial.println(postSample);

  if (abs(pressureSetpoint - postSample) > creepLim)
  {
    Serial.print("t2.pco=63488");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.print("t2.txt=\"Pressure creep detected\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    return false;
  }
  else
  {
    Serial.print("t2.pco=12000");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.print("t2.txt=\"Pressure is holding\"");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    return true;
  }
}

bool pulsingCheck(float currentPressure, int input, int output, int pulseValue, int pulseLim)
{

  digitalWrite(input, HIGH);

  int pulseNumber = 0;
  float newPressure = 0;
  bool holdingPressure = true;

  while (pulseNumber < pulseValue)
  { // three total pulses in the whole operation
    digitalWrite(output, LOW);
    delay(1500);
    newPressure = pressureSensor();
    Serial.print("Pressure after ");
    Serial.print(pulseNumber + 1);
    Serial.print(" pulse : ");
    Serial.println(newPressure);
    if (abs(currentPressure - newPressure) >= pulseLim)
    { // if the pressure after a pulse is greater than 20. valve fails the pulse test
      holdingPressure = false;
      // Serial.println("Pulsing failed");

      Serial.print("t2.pco=63488");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.print("t2.txt=\"Pulsing failed\"");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);

      return holdingPressure; // valve has passed the current leg of the pulsing operation
    }
    digitalWrite(output, HIGH);
    delay(500);
    pulseNumber += 1;
  }
  Serial.println("Pulsing passed");
  Serial.print("t2.pco=12000");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.print("t2.txt=\"Pulsing passed\"");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  return holdingPressure;
}

bool spindleReversal(int spindleDirPin, int spindleStepPin, int input, int output, int spindleUpButton, int spindleDownButton, int spindlePassButton, int spindleFailButton, int checkValue)
{

  int upCount = 0;

  /*
  Serial.println("Loosening for leak test");
  digitalWrite(spindleDirPin, LOW);
  for (int i = 0; i <= 400; i++) {
    digitalWrite(spindleStepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(spindleStepPin, LOW);
    delayMicroseconds(500);
  }
  */

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);

  bool spindleLeakState = false;
  /*
  while (spindleLeakState == false) {                           // operations stay within this loop until operator is not satisfied with the leak test

    // buttons to control the wind/unwind of the spindle and pass/fail
    bool spindleUpState = digitalRead(spindleUpButton);
    bool spindleDownState = digitalRead(spindleDownButton);
    bool spindlePass = digitalRead(spindlePassButton);
    bool spindleFail = digitalRead(spindleFailButton);


    if (spindleDownState == HIGH) {                             // drives the spindle down
      digitalWrite(spindleDirPin, HIGH);
      for (int i = 0; i <= 400; i++) {
        digitalWrite(spindleStepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(spindleStepPin, LOW);
        delayMicroseconds(500);
      }
      upCount--;
      Serial.println(upCount);
    }

    if (spindleUpState == HIGH) {                              // drives the spindle up
      digitalWrite(spindleDirPin, LOW);
      for (int i = 0; i <= 400; i++) {
        digitalWrite(spindleStepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(spindleStepPin, LOW);
        delayMicroseconds(500);
      }
      upCount++;                                               // variable keeps check of how many times the spindle was winded/unwinded
      Serial.println(upCount);
    }

    if (spindleFail == HIGH) {                                 // operator presses the fail button
      return false;
    }

    if (upCount >= checkValue) {
      if (spindlePass == HIGH) {                               // operator must check atleast 6 locations of unwinding before they can try pass the valve
        return true;
      }
    }
  }
  */

  digitalWrite(spindleDirPin, LOW);
  for (int i = 0; i <= 3; i++) // 3 complete revolutions
  {
    for (int j = 0; j <= 400; j++)
    {
      digitalWrite(spindleStepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(spindleStepPin, LOW);
      delayMicroseconds(500);
    }
  }

  while (spindleLeakState == false)
  {
    if (Serial.available())
    {
      delay(30);
      char leakCheck = (Serial.read());

      if (leakCheck == 'P')
      {
        Serial.println("Operator passed the valve in submerge test");
        return true;
      }

      if (leakCheck == 'F')
      {
        Serial.println("Operator failed the valve in submerge test");
        return false;
      }
    }
  }
}