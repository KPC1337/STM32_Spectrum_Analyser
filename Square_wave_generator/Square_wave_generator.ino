#include <Tone.h>

Tone tone1;
const int analogInPin = A5; 

void setup() {
  tone1.begin(13);  
  Serial.begin(115200);
}

void loop() { 
  int analogIn = analogRead(analogInPin);
  long int freq = map(analogIn, 0, 1023, 1, 80000);
  Serial.println(freq);
  tone1.play(freq);
  }
