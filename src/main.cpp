#include <Arduino.h>
#include <Wire.h>
#include <Notecard.h>
#include <cstring>
#include <cstdlib>

#define usbSerial Serial

#define productUID "com.gmail.taulabtech:taulabtest"

// LSM6DSOX I2C addresses
#define LSM6DSOX_ADDRESS_LOW  0x6A
#define LSM6DSOX_ADDRESS_HIGH 0x6B

// LSM6DSOX register addresses
#define LSM6DSOX_WHO_AM_I     0x0F
#define LSM6DSOX_CTRL1_XL     0x10
#define LSM6DSOX_STATUS_REG   0x1E
#define LSM6DSOX_OUTX_L_A     0x28

// Expected WHO_AM_I value
#define LSM6DSOX_WHO_AM_I_VALUE 0x6C

Notecard notecard;

// Configuration
float current_odr = 26.0f;           // 26 Hz sampling rate
unsigned long sample_interval_ms;    // Calculated from ODR
unsigned long logging_duration = 10000;  // 10 seconds

// Sensor variables
uint8_t lsm6dsox_address = 0;
bool lsm6dsox_found = false;

// Data storage for batching
#define MAX_SAMPLES 300  // Adjust based on available memory
float ax_samples[MAX_SAMPLES];
float ay_samples[MAX_SAMPLES];
float az_samples[MAX_SAMPLES];
int collected_samples = 0;

// I2C communication functions
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(lsm6dsox_address);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(lsm6dsox_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  
  Wire.requestFrom(lsm6dsox_address, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool readMultipleRegisters(uint8_t reg, uint8_t* buffer, uint8_t count) {
  Wire.beginTransmission(lsm6dsox_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  
  Wire.requestFrom(lsm6dsox_address, count);
  for (uint8_t i = 0; i < count; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read();
    } else {
      return false;
    }
  }
  return true;
}

bool initLSM6DSOX() {
  uint8_t addresses[] = {LSM6DSOX_ADDRESS_LOW, LSM6DSOX_ADDRESS_HIGH};
  
  for (int i = 0; i < 2; i++) {
    lsm6dsox_address = addresses[i];
    
    uint8_t whoami = readRegister(LSM6DSOX_WHO_AM_I);
    
    if (whoami == LSM6DSOX_WHO_AM_I_VALUE) {
      Serial.print("LSM6DSOX found at address 0x");
      Serial.println(lsm6dsox_address, HEX);
      
      // Configure accelerometer: 26Hz, ±2g
      uint8_t ctrl_val = 0x20;  // 0010 0000 = 26Hz, ±2g
      
      if (writeRegister(LSM6DSOX_CTRL1_XL, ctrl_val)) {
        Serial.println("LSM6DSOX configured: 26Hz, ±2g");
        delay(100);
        lsm6dsox_found = true;
        return true;
      }
    }
  }
  
  Serial.println("LSM6DSOX not found!");
  return false;
}

bool isDataReady() {
  uint8_t status = readRegister(LSM6DSOX_STATUS_REG);
  return (status & 0x01);
}

bool readAcceleration(float &ax, float &ay, float &az) {
  if (!lsm6dsox_found) return false;
  
  uint8_t data[6];
  if (!readMultipleRegisters(LSM6DSOX_OUTX_L_A, data, 6)) return false;
  
  int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
  int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
  int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);
  
  // Convert to mg (milligravity)
  const float sensitivity_mg = 0.061035;
  ax = raw_x * sensitivity_mg;
  ay = raw_y * sensitivity_mg;
  az = raw_z * sensitivity_mg;
  
  return true;
}

void writeBinaryData() {
  // Send acceleration data as base64-encoded JSON note instead of binary storage
  // This is simpler and more reliable than the complex binary API
  
  Serial.println("Encoding acceleration data as base64...");
  
  // Calculate total size needed
  int total_size = collected_samples * 12;  // 3 floats * 4 bytes each
  
  // Create buffer with all data
  uint8_t* all_data = (uint8_t*)malloc(total_size);
  if (all_data == NULL) {
    Serial.println("Failed to allocate memory for data");
    return;
  }
  
  // Pack all samples into the buffer
  for (int i = 0; i < collected_samples; i++) {
    int offset = i * 12;
    memcpy(&all_data[offset], &ax_samples[i], 4);
    memcpy(&all_data[offset + 4], &ay_samples[i], 4);
    memcpy(&all_data[offset + 8], &az_samples[i], 4);
  }
  
  // Base64 encode the entire dataset
  int encodedLen = ((total_size + 2) / 3) * 4 + 1;
  char* encoded = (char*)malloc(encodedLen);
  if (encoded == NULL) {
    Serial.println("Failed to allocate memory for encoded data");
    free(all_data);
    return;
  }
  
  JB64Encode(encoded, (const char*)all_data, total_size);
  
  // Send as regular JSON note with base64 data
  J *req = notecard.newRequest("note.add");
  JAddStringToObject(req, "file", "sensors.qo");
  JAddBoolToObject(req, "sync", true);
  
  J *body = JAddObjectToObject(req, "body");
  if (body) {
    JAddStringToObject(body, "data", encoded);
    JAddNumberToObject(body, "samples", collected_samples);
    JAddNumberToObject(body, "format", 1);  // 1 = float32 ax,ay,az format
    JAddNumberToObject(body, "rate_hz", current_odr);
    JAddNumberToObject(body, "duration_ms", logging_duration);
    JAddNumberToObject(body, "timestamp", millis());
  }
  
  bool success = notecard.sendRequest(req);
  
  if (success) {
    Serial.print("Successfully sent ");
    Serial.print(collected_samples);
    Serial.println(" samples as base64 JSON note");
  } else {
    Serial.println("Failed to send data note");
  }
  
  // Clean up
  free(all_data);
  free(encoded);
}

void sendSamplesToCloud() {
  if (collected_samples == 0) {
    Serial.println("No samples to send");
    return;
  }
  
  Serial.println("Sending samples to cloud as JSON note...");
  
  // Send data using the simpler JSON approach
  writeBinaryData();
}

void log() {
  Serial.println("A_X [mg]\tA_Y [mg]\tA_Z [mg]");
  Serial.print("Logging for ");
  Serial.print(logging_duration / 1000);
  Serial.println(" seconds...");
  
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Reset sample collection
  collected_samples = 0;
  
  unsigned long start_time = millis();
  unsigned long last_sample = 0;
  
  while (millis() - start_time < logging_duration && collected_samples < MAX_SAMPLES) {
    if (millis() - last_sample >= sample_interval_ms) {
      if (isDataReady()) {
        float ax, ay, az;
        if (readAcceleration(ax, ay, az)) {
          // Store the sample in arrays
          ax_samples[collected_samples] = ax;
          ay_samples[collected_samples] = ay;
          az_samples[collected_samples] = az;
          
          // Also print to serial for monitoring
          Serial.print(ax, 1);
          Serial.print("\t");
          Serial.print(ay, 1);
          Serial.print("\t");
          Serial.println(az, 1);
          
          collected_samples++;
        }
      }
      last_sample = millis();
    }
  }
  
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Logging completed!");
  Serial.print("Total samples collected: ");
  Serial.println(collected_samples);
  Serial.print("Actual rate: ");
  Serial.print((float)collected_samples * 1000.0 / logging_duration, 2);
  Serial.println(" Hz");
  
  // Send all samples as a single note (1 credit)
  sendSamplesToCloud();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(2500);
  usbSerial.begin(115200);
  notecard.begin();
  notecard.setDebugOutputStream(Serial);
  
  {
    J *req = notecard.newRequest("hub.set");
    if (req != NULL) {
      JAddStringToObject(req, "product", productUID);
      JAddStringToObject(req, "mode", "continuous");
      notecard.sendRequest(req);
    }
  }
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("=== LSM6DSOX Serial Logger ===");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize sensor
  if (!initLSM6DSOX()) {
    Serial.println("ERROR: Failed to initialize LSM6DSOX!");
    while(1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }
  
  // Calculate sample interval
  sample_interval_ms = (unsigned long)(1000.0f / current_odr);
  
  Serial.print("Max samples per session: ");
  Serial.println(MAX_SAMPLES);
  Serial.println("Ready to start logging...");
  delay(2000);
  log();
}

void loop() {

}