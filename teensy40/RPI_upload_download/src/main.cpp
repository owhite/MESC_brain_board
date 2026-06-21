#include <Arduino.h>

#define LED_PIN1 2
#define LED_PIN2 3
#define LED_PIN3 13

int speed = 200;
int sample_count = 0;
int seq = 0;
const int done_after_samples = 25;
bool done_emitted = false;

void emit_event(const char* level, const char* msg, int sample) {
  seq++;
  Serial.print("{\"seq\":");
  Serial.print(seq);
  Serial.print(",\"t_ms\":");
  Serial.print(millis());
  Serial.print(",\"level\":\"");
  Serial.print(level);
  Serial.print("\",\"msg\":\"");
  Serial.print(msg);
  Serial.print("\",\"sample\":");
  Serial.print(sample);
  Serial.println("}");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);

  delay(500);
  emit_event("INFO", "RUN_START", sample_count);
}

void loop() {
  sample_count++;
  emit_event("DATA", "UP", sample_count);

  if (!done_emitted && sample_count >= done_after_samples) {
    emit_event("INFO", "RUN_DONE", sample_count);
    done_emitted = true;
  }

  digitalWriteFast(LED_PIN1, HIGH);
  digitalWriteFast(LED_PIN2, HIGH);
  digitalWriteFast(LED_PIN3, HIGH);
  delay(speed);
  digitalWriteFast(LED_PIN1, LOW);
  digitalWriteFast(LED_PIN2, LOW);
  digitalWriteFast(LED_PIN3, LOW);
  delay(speed);
}


