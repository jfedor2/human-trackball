#include "DigiMouse.h"

#define PIN 0

void setup() {
  pinMode(PIN, INPUT);
  digitalWrite(PIN, HIGH);
  DigiMouse.begin();
}

int prevstate = 1;

void loop() {
  int state = digitalRead(PIN);
  if (state != prevstate) {
    if (state) {
      // release
      DigiMouse.setButtons(0);
    } else {
      // click left mouse button
      DigiMouse.setButtons(1);
    }
    DigiMouse.delay(50);
  }
  prevstate = state;
  DigiMouse.update();
}
