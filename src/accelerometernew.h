#ifndef ACCELEROMETERNEW_H
#define ACCELEROMETERNEW_H

#include "LSM6DSOXSensor.h"
#include "graham_generator.h"

#define INT_1 D5  // Changed to D5 as requested

extern volatile bool motionDetected;
extern volatile int state;
extern volatile int prevstate;
extern volatile bool stateChanged;

//Interrupts.
volatile int mems_event = 0;

// Components
LSM6DSOXSensor AccGyr(&Wire, LSM6DSOX_I2C_ADD_L);

// MLC
ucf_line_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;

void INT1Event_cb();
void printMLCStatus(uint8_t status);

void setupLSM6DSOX() 
{
  AccGyr.begin();

  /* Feed the program to Machine Learning Core */
  ProgramPointer = (ucf_line_t *)graham_generator;
  TotalNumberOfLine = sizeof(graham_generator) / sizeof(ucf_line_t);
  Serial.println("Motion Intensity for LSM6DSOX MLC");
  Serial.print("UCF Number Line=");
  Serial.println(TotalNumberOfLine);

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
    if (AccGyr.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
      Serial.print("Error loading the Program to LSM6DSOX at line: ");
      Serial.println(LineCounter);
      while (1) {
        delay(1000);
      }
    }
  }

  Serial.println("Program loaded inside the LSM6DSOX MLC");
  Serial.println("State detection active...");

  // Note: Don't enable accelerometer here - let the main code handle it
  // This allows the original acceleration logging to work

  //Interrupts.
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  // Initialize state variables
  motionDetected = false;
  state = -1;
  prevstate = -1;
  stateChanged = false;
}

// Get immediate raw state (for debugging)
int getRawState() {
  uint8_t mlc_out[8];
  AccGyr.Get_MLC_Output(mlc_out);
  return mlc_out[0];
}

// Consensus-based state reading for stability
int getState() {
  static int lastStableState = -1;
  static int consensusBuffer[3] = {-1, -1, -1};
  static int bufferIndex = 0;
  static bool initialized = false;
  
  // Get current raw reading
  uint8_t mlc_out[8];
  AccGyr.Get_MLC_Output(mlc_out);
  int currentReading = mlc_out[0];
  
  // If first time or no stable state yet, use current reading
  if (!initialized) {
    lastStableState = currentReading;
    initialized = true;
  }
  
  // Add to circular buffer
  consensusBuffer[bufferIndex] = currentReading;
  bufferIndex = (bufferIndex + 1) % 3;
  
  // Check if all 3 values are the same
  if (consensusBuffer[0] == consensusBuffer[1] && 
      consensusBuffer[1] == consensusBuffer[2] && 
      consensusBuffer[0] != -1) {
    // All 3 readings agree - update stable state
    lastStableState = consensusBuffer[0];
  }
  
  // Never return -1, always return a valid state
  return (lastStableState == -1) ? currentReading : lastStableState;
}

int checkForStateChange()
{
  if (motionDetected) {
    motionDetected = false; // Reset flag
    
    LSM6DSOX_MLC_Status_t status;
    AccGyr.Get_MLC_Status(&status);
    if (status.is_mlc1) {
      uint8_t mlc_out[8];
      AccGyr.Get_MLC_Output(mlc_out);
      
      int newState = mlc_out[0];
      
      // Check if state actually changed
      if (newState != prevstate && newState != -1) {
        prevstate = state;
        state = newState;
        stateChanged = true;
        return newState;
      }
    }
  }
  return -1;
}

// Function to check and print state changes (call this in main loop)
void checkAndPrintStateChange() {
  // Read current state directly (don't rely only on interrupts)
  static int lastReportedState = -1;
  int currentState = getRawState();
  
  // Check if state has changed since last check
  if (currentState != lastReportedState && currentState != -1) {
    Serial.print("State Change Detected: ");
    Serial.print(lastReportedState);
    Serial.print(" -> ");
    Serial.println(currentState);
    
    Serial.print("Timestamp: ");
    Serial.println(millis());
    
    lastReportedState = currentState;
  }
  
  // Also check interrupt-based detection
  int newState = checkForStateChange();
  if (stateChanged) {
    stateChanged = false; // Reset flag
    Serial.print("Interrupt-based State Change: ");
    Serial.print(prevstate);
    Serial.print(" -> ");
    Serial.println(state);
  }
}

void INT1Event_cb() {
  motionDetected = true;
}

// Global variables (define these in your main file)
volatile bool motionDetected = false;
volatile int state = -1;
volatile int prevstate = -1;
volatile bool stateChanged = false;

#endif // ACCELEROMETERNEW_H