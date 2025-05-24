#include <Wire.h>

#define I2C_SLAVE_ADDR 0x04

// Sensor Pins
#define leftgrey 32
#define leftblue 25
#define leftpurple 26
#define rightyellow 2
#define rightbrown 15
#define rightorange 0

// Manual Calibration Values for Each Sensor
const int sensorMin[6] = {500, 1450, 650, 640, 630, 3640};  // Pure white values
const int sensorMax[6] = {250, 870, 720, 840, 855, 865};  // Pure black values

// Sensor Positions (mm from center)
const float sensorPositions[6] = {-33, -21, -12, 11, 22, 31}; 

// PID Constants
float Kp = 0.90;
float Ki = 0.75;
float Kd = 0.80;
float baseSpeed = 170.0;
float centreAngle = 90.0;
float K = 0.5;

// PID Variables
float error = 0, previousError = 0, integral = 0;
float setpoint = 0;  // Centered position

// Motor & Servo Control
int servoAngle = 90;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Function to Read & Normalize Sensor Values
int getCalibratedValue(int rawValue, int sensorIndex) {
    return map(rawValue, sensorMin[sensorIndex], sensorMax[sensorIndex], 0, 1000);
}

// Function to Read Sensor Values
void readSensors(int *sensorValues) {
    sensorValues[0] = getCalibratedValue(analogRead(leftgrey), 0);
    sensorValues[1] = getCalibratedValue(analogRead(leftblue), 1);
    sensorValues[2] = getCalibratedValue(analogRead(leftpurple), 2);
    sensorValues[3] = getCalibratedValue(analogRead(rightyellow), 3);
    sensorValues[4] = getCalibratedValue(analogRead(rightbrown), 4);
    sensorValues[5] = getCalibratedValue(analogRead(rightorange), 5);
}

// Function to Compute Weighted Average (Line Position)
float getLinePosition(int *sensorValues) {
    float weightedSum = 0;
    float totalWeight = 0;
    for (int i = 0; i < 6; i++) {
        weightedSum += sensorValues[i] * sensorPositions[i];  
        totalWeight += sensorValues[i];
    }
    return (totalWeight == 0) ? 0 : (weightedSum / totalWeight);
}

// Function to Compute PID Correction
float computePID(float linePosition) {
    error = setpoint - linePosition;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// Function to Send Motor & Servo Commands via I2C
void sendMotorCommand(int leftMotorSpeed, int rightMotorSpeed, int servoAngle) {
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write((int16_t)leftMotorSpeed >> 8);
    Wire.write((int16_t)leftMotorSpeed & 0xFF);
    Wire.write((int16_t)rightMotorSpeed >> 8);
    Wire.write((int16_t)rightMotorSpeed & 0xFF);
    Wire.write((int16_t)servoAngle >> 8);
    Wire.write((int16_t)servoAngle & 0xFF);
    Wire.endTransmission();
}

// Function to Adjust Servo Angle & Motor Speeds
void adjustMotors(float correction) {
    servoAngle = centreAngle + correction;//Where u is the correction variable

    leftMotorSpeed = baseSpeed + (K * correction);
    rightMotorSpeed = baseSpeed - (K * correction);

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
    servoAngle = constrain(servoAngle, 45, 135); 

    sendMotorCommand(leftMotorSpeed, rightMotorSpeed, servoAngle);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
}

void loop() {
    int sensorValues[6];
    
    readSensors(sensorValues);
    
    float linePosition = getLinePosition(sensorValues);

    float correction = computePID(linePosition);

    adjustMotors(correction);

    delay(50);
}

