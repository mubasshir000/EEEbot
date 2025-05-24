#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define LED_PIN 4  // Change if using a different LED pin

Adafruit_MPU6050 mpu;

// Variables for gyroscope readings
float gyroAngleZ = 0.0;
float gyroBiasZ = 0.0;
float yaw = 0.0;
unsigned long prevTime = 0;
int lastQuadrant = 0;
bool ledState = LOW;  // LED state

void setup() {
    Serial.begin(115200);
    while (!Serial);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, ledState);

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    Serial.println("MPU6050 Found!");
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    Serial.println("Calibrating Gyroscope...");
    int numSamples = 500;
    float sum = 0.0;

    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;
        delay(10);
    }

    gyroBiasZ = sum / numSamples;
    Serial.print("Gyro Bias Z:");
    Serial.println(gyroBiasZ);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float alpha = 0.80;
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    float correctedGyroZ = g.gyro.z - gyroBiasZ;
    gyroAngleZ += correctedGyroZ * dt * 180 / PI;
    yaw = alpha * (yaw + correctedGyroZ * dt * 180 / PI) + (1 - alpha) * gyroAngleZ;

    // Normalize yaw to be within 0° to 360°
    int normalizedYaw = (int)yaw % 360;
    if (normalizedYaw < 0) normalizedYaw += 360;

    // Determine the current quadrant (0-3 for 0°, 90°, 180°, 270°)
    int currentQuadrant = normalizedYaw / 90;

    // If a new quadrant is entered, toggle LED
    if (currentQuadrant != lastQuadrant) {
        lastQuadrant = currentQuadrant;
        ledState = !ledState;  // Toggle LED state
        digitalWrite(LED_PIN, ledState);
    }
}


