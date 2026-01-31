#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("hello");
  delay(2000); // 0.5 Hz
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}