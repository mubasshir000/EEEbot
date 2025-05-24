// Define Ultrasonic Sensor Pins
const int echoPin = 33;
const int trigPin = 5;
const int ledPin = 4; // PWM-capable pin for LED brightness control

// Declare global variables
float duration;
float distance;

void setup() {
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
    pinMode(ledPin, OUTPUT);  // Sets the LED pin as an Output
    Serial.begin(115200);
}

void loop() {
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculates the distance
    distance = duration * 0.034 / 2;

    // Print distance to Serial Monitor
    Serial.print("Distance: ");
    Serial.println(distance);

    // Map distance (0 cm → 255 brightness, 100 cm → 0 brightness)
    int brightness = map(distance, 0, 100, 255, 0);
    
    // Constrain brightness between 0 and 255
    brightness = constrain(brightness, 0, 255);
    
    // Set LED brightness
    analogWrite(ledPin, brightness);
    Serial.println(brightness);

    delay(500);  // Wait before next measurement
}
