// timestretch_example.ino
//
// Phase vocoder time-stretching — Teensy 4.x + Audio Shield (SGTL5000)
//
// Controls (Serial):
//   SPACE  start / restart playback
//   s      stop playback
//   p      faster  (decrease stretch)
//   q      slower  (increase stretch)
//   1-3    load and play 01.WAV / 02.WAV / 03.WAV
//   t      transient threshold: 4  (sensitive — more phase resets)
//   y      transient threshold: 8  (default)
//   u      transient threshold: 16 (subtle — fewer phase resets)
//   d      toggle profiling report
//   h      print help
//
// Optional: pot wiper → A0 (outer legs to 3.3 V and GND)
//   Pot maps to stretch STRETCH_MIN … STRETCH_MAX

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "effect_phaseVocoder.h"

// ---------------------------------------------------------------------------
// Audio graph
// ---------------------------------------------------------------------------
AudioEffectPhaseVocoder vocoder;
AudioOutputUSB          audioOut;
// AudioOutputI2S       audioOut;
// AudioControlSGTL5000 codec;

AudioConnection patchLeft (vocoder, 0, audioOut, 0);
AudioConnection patchRight(vocoder, 0, audioOut, 1);

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
static const int   POT_PIN      = A0;
static const float STRETCH_MIN  = 0.5f;   // 0.5 = half duration (2× faster)
static const float STRETCH_MAX  = 1.5f;   // 1.5 = 1.5× duration (slower)
static const int   POT_READ_MS  = 50;
static const float CONTROL_STEP = 0.01f;
static const bool  USE_POT      = false;

float stretch   = 1.0f;
bool  profiling = false;

// ---------------------------------------------------------------------------
// Sample buffer
// DMAMEM places in OCRAM (512 KB) instead of DTCM.
// On Teensy 4.1 with PSRAM: replace DMAMEM with EXTMEM for much larger buffers.
// ---------------------------------------------------------------------------
DMAMEM int16_t sampleBuffer[44100 * 5];
static uint32_t sampleCount = 0;

static const char *sampleFiles[] = { "01.WAV", "02.WAV", "03.WAV" };
static int currentSample = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

static void applyStretch(float newStretch) {
    stretch = clampf(newStretch, STRETCH_MIN, STRETCH_MAX);
    vocoder.setStretch(stretch);
    float durationS = stretch * (sampleCount / (float)AUDIO_SAMPLE_RATE_EXACT);
    Serial.print("x  Speed: "); Serial.print(1.0f / stretch, 2);
    Serial.print("x  Duration: "); Serial.print(durationS, 1); Serial.println("s");
}

static void printHelp() {
    Serial.println("--- Controls ---");
    Serial.println("  SPACE  start / restart");
    Serial.println("  s      stop");
    Serial.println("  p      faster");
    Serial.println("  q      slower");
    Serial.println("  1-3    load 01.WAV / 02.WAV / 03.WAV");
    Serial.println("  t      transient threshold: 4  (sensitive)");
    Serial.println("  y      transient threshold: 8  (default)");
    Serial.println("  u      transient threshold: 16 (subtle)");
    Serial.println("  d      toggle profiling");
    Serial.println("  e      print per-section costs");
    Serial.println("  h      this help");
}

// ---------------------------------------------------------------------------
// WAV loader — handles mono and stereo (stereo: left channel only).
// Reads channel count from WAV header offset 22.
// ---------------------------------------------------------------------------
static bool loadSampleFromSD(const char *filename) {
    File f = SD.open(filename);
    if (!f) {
        Serial.print("Could not open: "); Serial.println(filename);
        return false;
    }

    f.seek(22);
    int chanLo = f.read();
    int chanHi = f.read();
    if (chanLo < 0 || chanHi < 0) { f.close(); return false; }
    const uint16_t numChannels = (uint16_t)((chanHi << 8) | chanLo);

    f.seek(44);  // skip 44-byte WAV header to raw PCM data
    sampleCount = 0;
    const uint32_t maxSamples = sizeof(sampleBuffer) / sizeof(sampleBuffer[0]);

    while (f.available() && sampleCount < maxSamples) {
        int lo = f.read();
        int hi = f.read();
        if (lo < 0 || hi < 0) break;
        sampleBuffer[sampleCount++] = (int16_t)((hi << 8) | lo);
        if (numChannels == 2) {
            // skip right channel sample
            if (f.read() < 0 || f.read() < 0) break;
        }
    }
    f.close();

    Serial.print("Loaded: "); Serial.print(filename);
    Serial.print("  samples: "); Serial.print(sampleCount);
    Serial.print("  ch: "); Serial.print(numChannels);
    Serial.print("  dur: "); Serial.print(sampleCount / (float)AUDIO_SAMPLE_RATE_EXACT, 1);
    Serial.println("s");
    return sampleCount > 0;
}

static void loadAndPlay(int index) {
    currentSample = index;
    vocoder.stop();
    if (loadSampleFromSD(sampleFiles[index])) {
        vocoder.setSample(sampleBuffer, sampleCount);
        vocoder.setLoop(true);
        applyStretch(stretch);
        vocoder.play();
    }
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    AudioMemory(40);
    Serial.begin(57600);
    while (!Serial && millis() < 2000) {}

    // codec.enable();
    // codec.volume(0.5f);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD init failed");
        return;
    }

    vocoder.setTransientThreshold(16.0f);
    loadAndPlay(0);
    printHelp();
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
    static uint32_t lastPotRead = 0;
    static uint32_t lastProf    = 0;

    if (Serial.available() > 0) {
        char key = Serial.read(); 
        if      (key == ' ')                { vocoder.stop(); vocoder.play(); Serial.println("Playing"); }
        else if (key == 's')                { vocoder.stop(); Serial.println("Stopped"); }
        else if (key == 'p')                applyStretch(stretch - CONTROL_STEP);  // faster
        else if (key == 'q')                applyStretch(stretch + CONTROL_STEP);  // slower
        else if (key >= '1' && key <= '3')  loadAndPlay(key - '1');
        else if (key == 't')                { vocoder.setTransientThreshold(4.0f);  Serial.println("Transient threshold: 4"); }
        else if (key == 'y')                { vocoder.setTransientThreshold(8.0f);  Serial.println("Transient threshold: 8"); }
        else if (key == 'u')                { vocoder.setTransientThreshold(16.0f); Serial.println("Transient threshold: 16"); }
        else if (key == 'd')                { profiling = !profiling; Serial.println(profiling ? "Profiling: on" : "Profiling: off"); }
        else if (key == 'e')                {
            float tW, tF, tA, tS, tI, tO;
            if (vocoder.getProfilingDetailed(tW, tF, tA, tS, tI, tO)) {
                Serial.println("--- Section costs (mean µs) ---");
                Serial.print("  window fill:     "); Serial.println(tW, 1);
                Serial.print("  forward FFT:     "); Serial.println(tF, 1);
                Serial.print("  phase analysis:  "); Serial.println(tA, 1);
                Serial.print("  phase synthesis: "); Serial.println(tS, 1);
                Serial.print("  inverse FFT:     "); Serial.println(tI, 1);
                Serial.print("  OLA + output:    "); Serial.println(tO, 1);
            }
        }
        else if (key == 'h')                printHelp();
    }

    // Profiling report every 2 seconds
    if (profiling && millis() - lastProf >= 2000) {
        lastProf = millis();
        float    meanUs;
        uint32_t peakUs;
        if (vocoder.getProfiling(meanUs, peakUs)) {
            const float budgetUs = 1e6f / (AUDIO_SAMPLE_RATE_EXACT / (float)AUDIO_BLOCK_SAMPLES);
            Serial.print("update()  mean: "); Serial.print(meanUs, 1);
            Serial.print(" us  peak: ");      Serial.print(peakUs);
            Serial.print(" us  load: ");      Serial.print(meanUs / budgetUs * 100.0f, 1);
            Serial.println("%");
        }
    }

    if (USE_POT && millis() - lastPotRead >= POT_READ_MS) {
        lastPotRead = millis();
        int raw = analogRead(POT_PIN);
        applyStretch(STRETCH_MIN + (raw / 1023.0f) * (STRETCH_MAX - STRETCH_MIN));
    }
}
