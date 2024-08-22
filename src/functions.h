#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <Arduino.h>

float pressureSensor();
float inletPressureSensor();
void pressureAdjustment(float errorValue,int springDirPin,int springStepPin,int input,int output,int setpoint);
bool shutOffFunction(int spindleDirPin,int spindleStepPin,int input, int output);
bool pressureDecay(int input, int output);
bool springPressureHold(int springDirPin,int springStepPin,int input,int output);
bool springPressure(float pressureSetpoint);
bool secondPulse(float currentPressure,int input,int output);
bool spindleReversal(int spindleDirPin,int spindleStepPin,int input, int output,int spindleUpButton, int spindleDownButton, int spindlePassButton, int spindleFailButton);

#endif