#include <Arduino.h>
#include <Audio.h>

// Audio objects
AudioSynthWaveformSine sine1;
AudioOutputI2S i2s1;
AudioConnection patchCord1(sine1, 0, i2s1, 0);
AudioConnection patchCord2(sine1, 0, i2s1, 1);
AudioControlSGTL5000 audioShield;

// State
float currentFreq = 1000.0f; // Hz
float currentAmp = 0.5f;     // 0.0 - 1.0
bool outputOn = true;

void handleLine(const String &line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;
  if (s.equalsIgnoreCase("on")) {
    outputOn = true;
    sine1.amplitude(currentAmp);
    Serial.println("OK: on");
    return;
  }
  if (s.equalsIgnoreCase("off")) {
    outputOn = false;
    sine1.amplitude(0.0f);
    Serial.println("OK: off");
    return;
  }

  // Frequency: "f <value>"
  if (s.startsWith("f ") || s.startsWith("F ")) {
    float v = s.substring(2).toFloat();
    if (v > 0.0f && v < 200000.0f) {
      currentFreq = v;
      sine1.frequency(currentFreq);
      Serial.print("OK: frequency="); Serial.println(currentFreq);
    } else {
      Serial.println("ERR: invalid frequency");
    }
    return;
  }

  // Amplitude: "a <percent>" (0-100)
  if (s.startsWith("a ") || s.startsWith("A ")) {
    int p = s.substring(2).toInt();
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    currentAmp = p / 100.0f;
    if (outputOn) sine1.amplitude(currentAmp);
    Serial.print("OK: amplitude="); Serial.print(p); Serial.println(" %");
    return;
  }

  Serial.println("ERR: unknown command");
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) ; // wait for serial
  AudioMemory(12);
//  i2s1.begin();
  audioShield.enable();
  audioShield.volume(0.5);

  // Initialize default sine
  sine1.amplitude(1.0f); // start muted
  sine1.frequency(currentFreq);

  Serial.println("Teensy I2S Sine Generator");
  Serial.println("Commands:");
  Serial.println("  f <hz>    - set frequency (e.g. f 1000)");
  Serial.println("  a <pct>   - set amplitude percent 0-100 (e.g. a 50)");
  Serial.println("  on        - enable output");
  Serial.println("  off       - disable output");
  Serial.println();
}

String inputBuf;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    Serial.printf("Received char: '%c\n' (0x%02x)\n", c, (uint8_t)c);
    if (c == ';') {
      Serial.printf("Received command: '%s'\n", inputBuf.c_str());
      handleLine(inputBuf);
      inputBuf = String();
    } else {
      inputBuf += c;
      // prevent runaway
      if (inputBuf.length() > 200) inputBuf.remove(0, inputBuf.length() - 200);
    }
  }
}
