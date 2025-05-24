#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//Creating an MPU6050 object
Adafruit_MPU6050 mpu;

// replace the next variables with your SSID/Password combination
const char* ssid = "EEELab03";
const char* password = "EEEE1002";                

// add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.8.12";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// LED Pin and Ultrasonic Sensor Pins
const int ledPin = 4;
const int echoPin = 33;
const int trigPin = 5;

// Declare global/static variables
float duration;
float distance;
float gyroAngleZ = 0.0;
float gyroBiasZ = 0.0;  // Gyro bias to be calculated in setup()
float yaw = 0.0;
unsigned long prevTime = 0;  // Time tracking for dt

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(ledPin, OUTPUT); // Sets the ledPin as an Output
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

    while (!Serial);

  // Initialize I2C communication
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Calibrate Gyro Bias
  Serial.println("Calibrating Gyroscope...");
  int numSamples = 500;
  float sum = 0.0;

  for(int i = 0; i <numSamples; i++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(10);
  }

  gyroBiasZ = sum / numSamples; //Average bias
  Serial.print("Gyro Bias Z:");
  Serial.println(gyroBiasZ);
}

void setup_wifi() {
  delay(10);
  // we start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // if a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // wait 5 seconds before retrying
      delay(1000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

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
    
    // convert the value to a char array
    char distanceString[8];
    dtostrf(distance, 1, 2, distanceString);
    Serial.print("Distance: ");
    Serial.println(distanceString);
    client.publish("esp32/ultrasonicsensor", distanceString);

    // Read sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float alpha = 0.80;  // Complementary filter coefficient

  // Calculate dt (time difference in seconds)
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  // Subtract gyroscope bias
  float correctedGyroZ = g.gyro.z - gyroBiasZ;

  // Calculate angle from gyroscope
  gyroAngleZ += correctedGyroZ * dt * 180 / PI;  // Integrate to get angle

  // Apply complementary filter for yaw
  yaw = alpha * (yaw + correctedGyroZ * dt * 180 / PI) + (1 - alpha) * gyroAngleZ;

  // convert the value to a char array
    char yawString[8];
    dtostrf(yaw, 1, 2, yawString);
    Serial.print("Yaw: ");
    Serial.println(yawString);
    client.publish("esp32/rotationz", yawString);
  }
}
