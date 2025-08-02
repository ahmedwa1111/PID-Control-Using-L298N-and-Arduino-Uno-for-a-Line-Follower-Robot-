#include <Wire.h>         // Required for I2C communication
#include <SparkFunSX1509.h> 

// --- Motor Control Pins ---
int ena = 5;
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int enb = 10;

// --- PID Constants ---
int P;
int I;
int D;

float Kp = 0.09;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

// --- Button Pins  ---
int button_calibration = A3;
int button_start = 2;

// --- SX1509 I2C Array Setup ---
// SX1509 I2C Address (Confirmed as 0x3E)
const byte SX1509_ADDRESS = 0x3E; 
SX1509 io; // Create an SX1509 object

// Define the pins on the SX1509 connected to the QRE1113 sensors
// For SparkFun Line Follower Array, these are usually pins 0-7.
const byte SENSOR_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7};
const int SensorCount = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);
uint16_t sensorValues[SensorCount]; // Array to store raw digital readings (0 or 1)

// You'll adjust these based on your actual sensor behavior.
uint16_t sensorMaxValues[SensorCount]; // Max (e.g., black)
uint16_t sensorMinValues[SensorCount]; // Min (e.g., white)

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing Line Follower...");

  // --- Initialize SX1509 ---
  Wire.begin(); // Initialize I2C communication
  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("SX1509 not found at 0x3E. Check wiring or I2C address.");
    while (1); // Halt if the chip is not found
  }
  Serial.println("SX1509 initialized successfully.");

  // Set SX1509 pins as inputs for the QRE1113 sensors
  for (int i = 0; i < SensorCount; i++) {
    io.pinMode(SENSOR_PINS[i], INPUT_PULLUP); // Use INPUT_PULLUP for digital QRE1113 breakouts
  }

  // --- Motor and Button Pin Setup ---
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  pinMode(button_calibration, INPUT_PULLUP); // Use INPUT_PULLUP for buttons
  pinMode(button_start, INPUT_PULLUP);     // if no external pull-down resistors

  // --- Calibration Routine ---
  Serial.println("Press calibration button to start calibration...");
  while(digitalRead(button_calibration) == HIGH) {} // Wait for button press (active low if using pullup)
  Serial.println("Calibrating...");

  // Initialize min/max values for calibration
  for (int i = 0; i < SensorCount; i++) {
    sensorMinValues[i] = 1; // Assume 1 is 'black' for initial max/min
    sensorMaxValues[i] = 0; // Assume 0 is 'white' for initial max/min
  }

  // Calibrate for a short duration (e.g., 200 iterations for a few seconds)
  // You need to manually move the line over all sensors during calibration
  for (uint16_t i = 0; i < 200; i++) { // Reduced calibration time for quicker test
    for (int j = 0; j < SensorCount; j++) {
      int value = io.digitalRead(SENSOR_PINS[j]); // Read current sensor value
      
      // Update min/max (0 is likely white, 1 is black for SX1509 INPUT_PULLUP)
      if (value < sensorMinValues[j]) sensorMinValues[j] = value;
      if (value > sensorMaxValues[j]) sensorMaxValues[j] = value;
    }
    delay(50); // Small delay between readings
  }
  Serial.println("Calibration Complete.");
  Serial.print("Min values: "); for(int i=0; i<SensorCount; i++) Serial.print(sensorMinValues[i]); Serial.println();
  Serial.print("Max values: "); for(int i=0; i<SensorCount; i++) Serial.print(sensorMaxValues[i]); Serial.println();


  // --- Wait for Start Button ---
  Serial.println("Press start button to begin line following...");
  while(digitalRead(button_start) == HIGH) {} // Wait for button press
  Serial.println("Starting line follower!");
  delay(1000); // Small delay before starting
}

void loop() {
  PID_control();
}

// Function to read sensors and get line position, replacing qtr.readLineBlack
uint16_t readLineCustom(uint16_t *sensorValuesArray) {
  uint32_t sum = 0;
  uint32_t weightedSum = 0;
  uint16_t onLine = 0;

  // Read all sensors from the SX1509
  for (int i = 0; i < SensorCount; i++) {
    sensorValuesArray[i] = io.digitalRead(SENSOR_PINS[i]);
    // For QRE1113 digital breakouts with INPUT_PULLUP:
    // LOW (0) means reflection (white)
    // HIGH (1) means no reflection (black)

    
    // Normalize values if you want a more "analog" representation from digital sensors.

    
    int normalizedValue = sensorValuesArray[i]; // 0 for white, 1 for black

    // If your sensors read 1 for white and 0 for black, you'd invert:
    // int normalizedValue = 1 - sensorValuesArray[i];

    if (normalizedValue == 1) { // If sensor detects the black line
      weightedSum += (long)normalizedValue * i * 1000; // Multiply by sensor index (0-7) and a scaling factor
      sum += normalizedValue;
      onLine++;
    }
  }

  if (sum == 0) {
    // If no sensors see the line, predict based on last position or return a specific value.
    // A common strategy is to return the last known position or an extreme value.
    // For now, let's assume we return a value indicating off to one side if no sensors see the line.
    // If the robot was last going right, return far right, etc.
    // For simplicity, returning a value slightly off center if completely off.
    // In a real robot, you might want to return `lastPosition` or 0/7000.
    return lastError < 0 ? 0 : (SensorCount - 1 ) * 1000; // Returns 0 or 7000 if completely off line
  }

  // Calculate the weighted average position
  // The line position will range from 0 (far left) to (SensorCount-1)*1000 (far right)
  // For 8 sensors, 0 to 7000. Center is 3500.
  return weightedSum / sum;
}


void PID_control() {
  // Use your custom readLineCustom function
  uint16_t positionLine = readLineCustom(sensorValues);
  
  /// Debugging line position

  // Error calculation: 3500 is the center for an 8-sensor array (0 to 7000)
  int error = 3500 - positionLine;

  P = error;
  I = I + error; // Accumulate error for integral term
  D = error - lastError;
  lastError = error;

  // Basic anti-windup for integral term
  if (I > 10000) I = 10000;
  if (I < -10000) I = -10000;

  int motorSpeedChange = P * Kp + I * Ki + D * Kd;

  int motorSpeedA = 120+ motorSpeedChange; // Left motor 
  int motorSpeedB = 120 - motorSpeedChange; // Right motor 

  // --- Speed Clamping  ---
  if (motorSpeedA > 120) {
    motorSpeedA = 120;
  }
  if (motorSpeedB > 120) {
    motorSpeedB = 120;
  }
  if (motorSpeedA < -110) {
    motorSpeedA = -110;
  }
  if (motorSpeedB < -100) { 
    motorSpeedB = -100;
  }

 
  
  forward_movement(motorSpeedA, motorSpeedB);
}

void forward_movement(int speedA, int speedB) {
  // --- Left Motor ---
  if (speedA < 0) { // Reverse
    speedA = 0 - speedA; // Make speed positive
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else { // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2,LOW );
  }
  analogWrite(ena, speedA);

  // --- Right Motor ---
  if (speedB < 0) { // Reverse
    speedB = 0 - speedB; // Make speed positive
    digitalWrite(in3,LOW );
    digitalWrite(in4, HIGH);
  } else { // Forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4,LOW );
  }
  analogWrite(enb, speedB);
}
