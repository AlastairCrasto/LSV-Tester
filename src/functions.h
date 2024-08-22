#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <Arduino.h>

// These functions calculate the pressure at the inlet and the outlet
float pressureSensor();
float inletPressureSensor();

// PreesureAdjustment set the spring pressure to allow an set output (350 or 500 kPa)
void pressureAdjustment(float errorValue,int springDirPin,int springStepPin,int input,int output,int setpoint);

// shutoffFunction winds the spindle to shut off the flow as well as tests the shut off functionality of the spindle in the LV Tester.
bool shutOffFunction(int spindleDirPin,int spindleStepPin,int input, int output);

//initial test carried out to see if there are any leaks through the valve
bool pressureDecay(int input, int output);

// Preliminary check of the spring housing to see if it functionality is in check
bool springPressureHold(int springDirPin,int springStepPin,int input,int output);

// Test carried out after setting the pressure to check for any pressure creep in the outlet
bool springPressure(float pressureSetpoint);

// Pulse check to see if the spring housing holds pressure
bool secondPulse(float currentPressure,int input,int output);

// Submerge test and used by operators to check for any leaks in the valve
bool spindleReversal(int spindleDirPin,int spindleStepPin,int input, int output,int spindleUpButton, int spindleDownButton, int spindlePassButton, int spindleFailButton);

#endif