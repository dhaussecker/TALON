#ifndef ACCELEROMETERNEW_H
#define ACCELEROMETERNEW_H

#include "LSM6DSOXSensor.h"
#include "graham_generator.h"
#include <Notecard.h>

// External notecard instance (defined in main.cpp)
extern Notecard notecard;

#define INT_1 D5  // Changed to D5 as requested

extern volatile bool motionDetected;
extern volatile int state;
extern volatile int prevstate;
extern volatile bool stateChanged;

// Structure to store state change events
struct StateChangeEvent {
  int fromState;
  int toState;
  unsigned long timestamp;
};

// Storage for state change events (max 100 events between transmissions)
#define MAX_STATE_EVENTS 100
StateChangeEvent stateEvents[MAX_STATE_EVENTS];
int eventCount = 0;
unsigned long lastTransmission = 0;

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
  
  // Get initial state
  uint8_t mlc_out[8];
  AccGyr.Get_MLC_Output(mlc_out);
  state = mlc_out[0];
  prevstate = state;
  stateChanged = false;
  
  Serial.print("Initial MLC State: ");
  Serial.println(state);
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
    
    // Get current state directly
    uint8_t mlc_out[8];
    AccGyr.Get_MLC_Output(mlc_out);
    int newState = mlc_out[0];
    
    // Debug output
    Serial.print("Interrupt! Current state: ");
    Serial.print(state);
    Serial.print(", New state: ");
    Serial.print(newState);
    
    // Check if state actually changed from our last known state
    if (newState != state && newState != -1) {
      prevstate = state;  // Store previous state
      state = newState;   // Update current state
      stateChanged = true;
      Serial.println(" -> CHANGE DETECTED!");
      return newState;
    } else {
      Serial.println(" -> no change");
    }
  }
  return -1;
}

// Add a state change event to storage
void addStateChangeEvent(int fromState, int toState, unsigned long timestamp) {
  if (eventCount < MAX_STATE_EVENTS) {
    stateEvents[eventCount].fromState = fromState;
    stateEvents[eventCount].toState = toState;
    stateEvents[eventCount].timestamp = timestamp;
    eventCount++;
    
    Serial.print("State Change Stored: ");
    Serial.print(fromState);
    Serial.print(" -> ");
    Serial.print(toState);
    Serial.print(" at ");
    Serial.println(timestamp);
  } else {
    Serial.println("Warning: State event buffer full!");
  }
}

// Check for interrupt-based state changes and store them
void checkAndStoreStateChanges() {
  // First check if interrupt occurred
  int newState = checkForStateChange();
  
  // Then check if a state change was detected
  if (stateChanged) {
    stateChanged = false; // Reset flag
    addStateChangeEvent(prevstate, state, millis());
  }
}

// Send all stored state changes to Notehub
void sendStateChangesToCloud() {
  if (eventCount == 0) {
    Serial.println("No state changes to send");
    return;
  }
  
  Serial.print("Sending ");
  Serial.print(eventCount);
  Serial.println(" state changes to cloud...");
  
  // Create JSON note with all state changes
  J *req = notecard.newRequest("note.add");
  JAddStringToObject(req, "file", "states.qo");
  JAddBoolToObject(req, "sync", true);
  
  J *body = JAddObjectToObject(req, "body");
  if (body) {
    JAddNumberToObject(body, "event_count", eventCount);
    JAddNumberToObject(body, "collection_start", lastTransmission);
    JAddNumberToObject(body, "collection_end", millis());
    
    // Add events as an array
    J *events = JAddArrayToObject(body, "events");
    if (events) {
      for (int i = 0; i < eventCount; i++) {
        J *event = JCreateObject();
        if (event) {
          JAddNumberToObject(event, "from", stateEvents[i].fromState);
          JAddNumberToObject(event, "to", stateEvents[i].toState);
          JAddNumberToObject(event, "time", stateEvents[i].timestamp);
          JAddItemToArray(events, event);
        }
      }
    }
  }
  
  bool success = notecard.sendRequest(req);
  
  if (success) {
    Serial.print("Successfully sent ");
    Serial.print(eventCount);
    Serial.println(" state changes");
    
    // Reset for next collection period
    eventCount = 0;
    lastTransmission = millis();
  } else {
    Serial.println("Failed to send state changes");
  }
}

// Check if it's time to send state changes (every 5 minutes)
void checkStateTransmissionTimer() {
  const unsigned long FIVE_MINUTES = 5 * 60 * 1000; // 5 minutes in milliseconds
  
  if (millis() - lastTransmission >= FIVE_MINUTES) {
    sendStateChangesToCloud();
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