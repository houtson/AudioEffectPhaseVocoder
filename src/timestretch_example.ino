// timestretch_example.ino
//
// Plays a mono WAV file from SD with time-stretching controlled by serial
// keys or an optional potentiometer wired to an analogue pin.
//
// Hardware:
//   Teensy 4.0 + Audio Shield (SGTL5000)
//   Pot wiper → A0  (outer legs to 3.3 V and GND)
//
// Pot range maps to playback speed 0.5× … 1.5×.
//
// WAV requirements:
//   Mono, 16-bit PCM, 44100 Hz.
//
// --- PHASE STATUS ---
// Phase 3 (current): Time-stretching active via setStretch().
//                    p/q keys adjust stretch factor. Pot support optional.

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "effect_phaseVocoder.h"

// ---------------------------------------------------------------------------
// Audio graph — vocoder is 0-input/1-output, owns its own sample buffer
// ---------------------------------------------------------------------------
AudioEffectPhaseVocoder  vocoder;
AudioOutputUSB audioOut;
//AudioOutputI2S audioOut;
//AudioControlSGTL5000 codec;

AudioConnection patchLeft(vocoder, 0, audioOut, 0);
AudioConnection patchRight(vocoder, 0, audioOut, 1);

// ---------------------------------------------------------------------------
// Pin & timing config
// ---------------------------------------------------------------------------
static const int   POT_PIN      = A0;
static const float STRETCH_MIN  = 0.5f;   // 0.5 = half duration (faster)
static const float STRETCH_MAX  = 2.0f;   // 2.0 = double duration (slower)
static const int   POT_READ_MS  = 50;
static const float CONTROL_STEP = 0.05f;
static const bool  USE_POT      = false;
float stretch = 1.0f;

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void applyStretch(float newStretch)
{
    stretch = clampf(newStretch, STRETCH_MIN, STRETCH_MAX);
    vocoder.setStretch(stretch);

    Serial.print("Stretch: ");
    Serial.print(stretch, 2);
    Serial.print("x  (");
    Serial.print(1.0f / stretch, 2);
    Serial.println("x speed)");
}

// ---------------------------------------------------------------------------
// Sample buffer — loaded from SD into RAM at startup
// Adjust size to fit your sample. 44100*5 = ~5 s mono at 44100 Hz.
// On Teensy 4.1 with PSRAM: use EXTMEM int16_t sampleBuffer[...];
// ---------------------------------------------------------------------------
// For Teensy 4.1 with PSRAM, replace DMAMEM with EXTMEM for up to 16MB.
DMAMEM int16_t sampleBuffer[44100 * 5];
static uint32_t sampleCount = 0;

// ---------------------------------------------------------------------------
// Load raw 16-bit PCM from SD. Skips a standard 44-byte WAV header.
// ---------------------------------------------------------------------------
bool loadSampleFromSD(const char *filename)
{
    File f = SD.open(filename);
    if (!f) {
        Serial.print("Could not open: ");
        Serial.println(filename);
        return false;
    }

    f.seek(44);  // skip WAV header
    sampleCount = 0;
    const uint32_t maxSamples = sizeof(sampleBuffer) / sizeof(sampleBuffer[0]);

    while (f.available() && sampleCount < maxSamples) {
        int lo = f.read();
        int hi = f.read();
        if (lo < 0 || hi < 0) break;
        sampleBuffer[sampleCount++] = (int16_t)((hi << 8) | lo);
    }
    f.close();

    Serial.print("Loaded ");
    Serial.print(sampleCount);
    Serial.println(" samples");
    return sampleCount > 0;
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup()
{
    AudioMemory(40);
    Serial.begin(57600);

   // codec.enable();
    //codec.volume(0.5f);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD init failed");
        return;
    }

    if (loadSampleFromSD("01.WAV")) {
        vocoder.setSample(sampleBuffer, sampleCount);
        vocoder.setLoop(true);
        applyStretch(stretch);
        vocoder.play();
        Serial.println("p = slower, q = faster, r = restart, s = stop");
    }
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop()
{
    static uint32_t lastRead = 0;

    if (Serial.available() > 0)
    {
        char key = Serial.read();

        if (key == 'p') {
            applyStretch(stretch + CONTROL_STEP);  // slower
        }
        else if (key == 'q') {
            applyStretch(stretch - CONTROL_STEP);  // faster
        }
        else if (key == 'r') {
            vocoder.stop();
            vocoder.play();
        }
        else if (key == 's') {
            vocoder.stop();
            Serial.println("Stopped");
        }
    }

    if (USE_POT && millis() - lastRead >= POT_READ_MS)
    {
        lastRead = millis();
        int raw = analogRead(POT_PIN);
        float mappedStretch = STRETCH_MIN + (raw / 1023.0f) * (STRETCH_MAX - STRETCH_MIN);
        applyStretch(mappedStretch);
    }
}
