#include "main.h"
#include "tone_player.h"

static TonePlayer g_tone;

void setup() {

  tone_init(&g_tone, SPEAKER_PIN);
  tone_start(&g_tone, /*freq_hz=*/2000, /*dur_ms=*/300, /*silence_ms=*/200);

}

void loop() {
  uint32_t now = micros();
  tone_update(&g_tone, now);

  // Polling examples
  if (tone_is_playing(&g_tone)) {
    // currently sounding
  } else if (tone_is_silence(&g_tone)) {
    // in post-tone quiet period
  } else if (tone_is_idle(&g_tone)) {
    // all done; could schedule another tone
  }

}
