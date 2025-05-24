#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define I2C_SLAVE_ADDR 0x04

// MPU6050 object
Adafruit_MPU6050 mpu;

// Ultrasonic Sensor Pins
const int echoPin = 33;
const int trigPin = 5;

// Global Variables
float yaw = 0.0, gyroBiasZ = 0.0, gyroAngleZ = 0.0;
unsigned long prevTime = 0;

// Function to get distance from Ultrasonic Sensor
float getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

// Function to get yaw from MPU6050
float getYaw() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dt = (millis() - prevTime) / 1000.0;
    prevTime = millis();
    float correctedGyroZ = g.gyro.z - gyroBiasZ;
    gyroAngleZ += correctedGyroZ * dt * 180 / PI;
    yaw = 0.8 * (yaw + correctedGyroZ * dt * 180 / PI) + 0.2 * gyroAngleZ;
    return yaw;
}

// Function to send motor commands via I2C
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

void turnToAngle(float targetAngle) {
    float startYaw = getYaw();
    float targetYaw = startYaw + targetAngle;
    while (abs(getYaw() - targetYaw) > 2) {  //small error margin
        sendMotorCommand(0,0,0);//Car comes to a stop
        delay(300);
        sendMotorCommand(200, -200, 0);  // Rotate on the spot
        delay(50);
    }
    sendMotorCommand(0, 0, 90);  // Stop turning
    delay(200);
}

void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.begin(115200);
    Wire.begin();
    
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Found!");
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    
    // Calibrate Gyroscope
    float sum = 0.0;
    for (int i = 0; i < 500; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;
        delay(10);
    }
    gyroBiasZ = sum / 500;
}

void loop() {
    
    if (distance <= 10) {
        sendMotorCommand(0, 0, 90); // Stop
        delay(500);
        turnToAngle(-90); // Turn left 90° (anti-clockwise)
        delay(500);
        if (getDistance() <= 10) {
            turnToAngle(180); // If still blocked, turn 180° (to check right)
            delay(500);
            if (getDistance() <= 10) {
                turnToAngle(90); // If still blocked, turn 90° (back to original direction)
            }
        }
    }
    sendMotorCommand(200, 200, 90); // Move forward
    delay(200);
}

