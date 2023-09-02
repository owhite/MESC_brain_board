#include <Throb.h>

#define LED_PIN    13    // I/O pin connected to the LED.

Throb throb(LED_PIN);

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

// Note: throb's PWM settings are based on counters.
//  the counters wont get bumped unless
//  it's called. It's not based on interrupts
// Delays in the loop will also slow the heartbeat
void loop() {
  int i;
  for(i=0;i<1000000;i++) {
    throb.pulse();
  }
  for(i=0;i<1000000;i++) {
    throb.stop();
  }
  for(i=0;i<1000000;i++) {
    throb.goDark();
  }
  for(i=0;i<1000000;i++) {
    throb.fullOn();
  }
}

