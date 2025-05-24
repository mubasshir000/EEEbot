#include <Wire.h>

// Sensor Pins
#define leftgrey 32
#define leftblue 25
#define leftpurple 26
#define rightyellow 2
#define rightbrown 15
#define rightorange 0

void setup() {
    Serial.begin(115200);  // Start serial communication at 115200 baud
    
    pinMode(leftgrey, INPUT);
    pinMode(leftblue, INPUT);
    pinMode(leftpurple, INPUT);
    pinMode(rightyellow, INPUT);
    pinMode(rightbrown, INPUT);
    pinMode(rightorange, INPUT);
}

void loop() {
    // Read sensor values
    int greyValue = analogRead(leftgrey);
    int blueValue = analogRead(leftblue);
    int purpleValue = analogRead(leftpurple);
    int yellowValue = analogRead(rightyellow);
    int brownValue = analogRead(rightbrown);
    int orangeValue = analogRead(rightorange);
    
    // Print values to Serial Monitor
    Serial.print("Left Grey: "); Serial.print(greyValue);
    Serial.print(" | Left Blue: "); Serial.print(blueValue);
    Serial.print(" | Left Purple: "); Serial.print(purpleValue);
    Serial.print(" | Right Yellow: "); Serial.print(yellowValue);
    Serial.print(" | Right Brown: "); Serial.print(brownValue);
    Serial.print(" | Right Orange: "); Serial.println(orangeValue);
    
    delay(500);  // Delay for readability
}
