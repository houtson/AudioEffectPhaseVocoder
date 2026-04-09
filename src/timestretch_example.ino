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
// Phase 1 (current): Effect is 1-in/1-out; AudioPlaySdWav feeds it from SD.
// Phase 2: Effect becomes 0-in/1-out and owns the sample buffer directly.
//          setSample(), play(), stop(), setLoop() will replace AudioPlaySdWav.
// Phase 3: setStretch() / setSpeed() will be active.

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "effect_phaseVocoder.h"

// ---------------------------------------------------------------------------
// Audio graph
// ---------------------------------------------------------------------------

// --- Phase 1 only: temporary WAV source feeding the vocoder ---
AudioPlaySdWav           voice;
AudioEffectPhaseVocoder  vocoder;
AudioOutputI2S           audioOut;
AudioControlSGTL5000     codec;

// Phase 1 connections — voice feeds vocoder
AudioConnection patchIn   (voice,   0, vocoder, 0);
AudioConnection patchLeft (vocoder, 0, audioOut, 0);
AudioConnection patchRight(vocoder, 0, audioOut, 1);

// --- Phase 2: remove voice, patchIn, and uncomment this block ---
// AudioEffectPhaseVocoder  vocoder;
// AudioOutputI2S           audioOut;
// AudioControlSGTL5000     codec;
// AudioConnection patchLeft (vocoder, 0, audioOut, 0);
// AudioConnection patchRight(vocoder, 0, audioOut, 1);

// ---------------------------------------------------------------------------
// Pin & timing config
// ---------------------------------------------------------------------------
static const int   POT_PIN      = A0;
static const float SPEED_MIN    = 0.5f;   // full CCW → half speed
static const float SPEED_MAX    = 1.5f;   // full CW  → 1.5× speed
static const int   POT_READ_MS  = 50;     // read pot every 50 ms
static const float CONTROL_STEP = 0.02f;
static const bool  USE_POT      = false;
float speed = 1.0f;

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void applySpeed(float newSpeed)
{
    speed = clampf(newSpeed, SPEED_MIN, SPEED_MAX);

    // Phase 3: uncomment when setStretch() is implemented
    // vocoder.setStretch(speed);

    Serial.print("Speed: ");
    Serial.print(speed, 3);
    Serial.println("x  (stretch control not yet active — Phase 3)");
}

// ---------------------------------------------------------------------------
// Phase 2: sample buffer — fill this before calling vocoder.setSample().
// Replace with your own loader (SD card, USB, etc.).
// int16_t sampleBuffer[44100 * 3];  // ~3 s at 44100 Hz
// static uint32_t sampleCount = 0;
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup()
{
    AudioMemory(40);
    Serial.begin(57600);

    codec.enable();
    codec.volume(0.5f);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD init failed");
        return;
    }

    // Phase 1: play directly from SD via AudioPlaySdWav
    applySpeed(speed);
    voice.play("01.WAV");

    // --- Phase 2: load raw PCM into buffer and hand to vocoder ---
    // if (loadSampleFromSD("01.wav")) {
    //     vocoder.setSample(sampleBuffer, sampleCount);
    //     vocoder.setLoop(true);
    //     applySpeed(speed);
    //     vocoder.play();
    // }
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop()
{
    static uint32_t lastRead = 0;

    // Phase 1: restart when playback finishes
    if (!voice.isPlaying()) {
        voice.play("01.WAV");
    }

    if (Serial.available() > 0)
    {
        char key = Serial.read();

        if (key == 'p') {
            applySpeed(speed + CONTROL_STEP);
        }
        else if (key == 'q') {
            applySpeed(speed - CONTROL_STEP);
        }
        // Phase 1: 'r' replays from the beginning
        else if (key == 'r') {
            voice.stop();
            voice.play("01.WAV");
        }
    }

    if (USE_POT && millis() - lastRead >= POT_READ_MS)
    {
        lastRead = millis();
        int raw = analogRead(POT_PIN);
        float mappedSpeed = SPEED_MIN + (raw / 1023.0f) * (SPEED_MAX - SPEED_MIN);
        applySpeed(mappedSpeed);
    }
}

// ---------------------------------------------------------------------------
// Phase 2: SD loader — uncomment when setSample() is implemented
// ---------------------------------------------------------------------------
// bool loadSampleFromSD(const char *filename)
// {
//     File f = SD.open(filename);
//     if (!f)
//         return false;
//
//     f.seek(44);  // skip 44-byte WAV header
//     sampleCount = 0;
//     while (f.available() && sampleCount < sizeof(sampleBuffer) / sizeof(sampleBuffer[0]))
//     {
//         int lo = f.read();
//         int hi = f.read();
//         if (lo < 0 || hi < 0) break;
//         sampleBuffer[sampleCount++] = (int16_t)((hi << 8) | lo);
//     }
//     f.close();
//     return sampleCount > 0;
// }
