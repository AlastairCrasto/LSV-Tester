#include <Arduino.h>
#include "functions.h"

float pressureSensor()
{
  int pressure = 0;
 //float sensorValue = analogRead(A4); // reading analogue value at pin A0

 // pressure = 1.1843 * sensorValue - 2.5289; // Linear relationship between the analog values and output pressure
  float sum = 0;
  
  for(int i = 0; i<10 ; i++){
    float sensorValue = analogRead(A4);
    float calibratedValue = 0.9578*sensorValue - 1.0058;
    sum += calibratedValue;
  }
   
   float avg = sum/10;
  pressure = avg;

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

  //float sensorValue = analogRead(A5);
  //pressureSend = sensorValue;
  float sum = 0;

  for(int i= 0 ; i<10 ; i++){
    float sensorValue = analogRead(A5);
    float calibratedValue = 0.97*sensorValue -3.5531;
    sum += calibratedValue;
  }

  float average = sum/10;

  pressureSend = average;

  if (pressureSend < 6)
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

bool shutOffFunction(int spindleForward, int spindleReverse, int input, int output, int shutLimit, int preShutLimit)
{

  bool coarse_tighten = true;

  Serial3.print("t2.pco=63015");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("t2.txt=\"Spindle driver in action\"");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  if (coarse_tighten == true)
  {
    
    //digitalWrite(spindleDirPin, HIGH);
    digitalWrite(output, HIGH);
    delay(500);
    digitalWrite(input, LOW);
    delay(500);

    digitalWrite(spindleForward,HIGH);
    delay(1500);
    digitalWrite(spindleForward,LOW);

    Serial.println("Coarse tightening completed");
    coarse_tighten = false;
  }

  // The following code tests the functionality of the spindle to check for any pressure creep despite shut off
  Serial3.print("t2.pco=63015");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("t2.txt=\"Testing shut off functionality\"");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  delay(500);

  float preSample = 0;
  float postSample = 0;

  preSample = pressureSensor();
  Serial.print("The outlet pressure after shut off (pre) : ");
  Serial.println(preSample);

  Serial.println("Checking for pressure creep after shut off");
  delay(8000);
  Serial.print("The outlet pressure after shut off (post) : ");
  postSample = pressureSensor();
  Serial.println(postSample);
  Serial.print("Difference : ");
  Serial.println(abs(preSample - postSample));

  if (abs(preSample - postSample) >= shutLimit || preSample > preShutLimit)
  {
    // Serial.println("Pressure leaked after shut off ---> shut off failed");
    Serial3.print("t2.pco=63488");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.print("t2.txt=\"Shut off test failed\"");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    return false;
  }
  else
  {
    // Serial.println("Pressure not leaking after shut off ---> shut off passed");
    Serial3.print("t2.pco=12000");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.print("t2.txt=\"Shut off test passed\"");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    return true;
  }
}

bool pressureDecay(int input, int output, int decayLim)
{
  float pressureTest = 0;
  float nextSample = 0;

  Serial3.print("t2.pco=63015");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("t2.txt=\"Testing for leaks\"");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

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

  Serial3.print("t2.pco=63015");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("t2.txt=\"Testing pressure creep\"");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  Serial.print("Pressure before delay to check for creep : ");
  Serial.println(pressureSetpoint); // pressureSetpoint variable is the mark at which the spring housing was set to
  delay(10000);

  postSample = pressureSensor(); // postSample is to check for any pressure creep once the output pressure has been set
  Serial.print("Pressure after delay to check for creep : ");
  Serial.println(postSample);

  if (abs(pressureSetpoint - postSample) > creepLim)
  {
    Serial3.print("t2.pco=63488");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.print("t2.txt=\"Pressure creep detected\"");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    return false;
  }
  else
  {
    Serial3.print("t2.pco=12000");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.print("t2.txt=\"Pressure is holding\"");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
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

      Serial3.print("t2.pco=63488");
      Serial3.write(0xff);
      Serial3.write(0xff);
      Serial3.write(0xff);
      Serial3.print("t2.txt=\"Pulsing failed\"");
      Serial3.write(0xff);
      Serial3.write(0xff);
      Serial3.write(0xff);

      return holdingPressure; // valve has passed the current leg of the pulsing operation
    }
    digitalWrite(output, HIGH);
    delay(500);
    pulseNumber += 1;
  }
  Serial3.println("Pulsing passed");
  Serial3.print("t2.pco=12000");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("t2.txt=\"Pulsing passed\"");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  return holdingPressure;
}

bool spindleReversal(int spindleForward, int spindleReverse, int spindle, int catridge, int input, int output, int submerge)
{

 // int upCount = 0;

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);

  bool spindleLeakState = false;


 digitalWrite(spindleReverse,HIGH);
 delay(400);
 digitalWrite(spindleReverse,LOW);
 delay(500);

 digitalWrite(spindle,LOW);
 digitalWrite(catridge,LOW);
 delay(2000);

  Serial3.print("t2.pco=63015");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print("t2.txt=\"Submerging - Please check for leaks\"");
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);


 digitalWrite(submerge,HIGH);
 delay(3000);
 

 //ADD THE SUBMERGING STEPS HERE


  while (spindleLeakState == false)
  {
    if (Serial3.available())
    {
      delay(30);
      char leakCheck = (Serial3.read());

      if (leakCheck == 'P')
      {
        Serial.println("Operator passed the valve in submerge test");
        digitalWrite(submerge,LOW);
        delay(3000);
         //digitalWrite(spindle,HIGH);
          digitalWrite(catridge,HIGH);
          delay(1000);

        return true;
        
      }

      if (leakCheck == 'F')
      {
        Serial.println("Operator failed the valve in submerge test");
        digitalWrite(submerge,LOW);
        delay(2000);
        return false;
      }
    }
  }
}