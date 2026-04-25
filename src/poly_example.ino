// poly_example.ino
//
// Polyphonic phase vocoder — 10 samples, 4 simultaneous voices
//
// Controls (Serial):
//   1-9, 0   trigger sample 1-10 (0 = sample 10)
//   p        faster  (decrease stretch on last selected)
//   q        slower  (increase stretch on last selected)
//   w        pitch up 1 semitone
//   x        pitch down 1 semitone
//   z        reset pitch to 0
//   r        toggle reverse
//   l        toggle loop
//   SPACE    re-trigger last selected sample
//   s        stop all voices
//   t/y/u    transient threshold 4 / 8 / 16 (all voices)
//   d        toggle profiling report (2 s interval)
//   h        print help
//
//  controls  apply to the most-recently triggered sample.
// Settings per sample persist between triggers.

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "effect_phaseVocoder.h"

// 4 voices
AudioEffectPhaseVocoder voice0, voice1, voice2, voice3;
AudioEffectFade fade0, fade1, fade2, fade3;
AudioMixer4 mixer;
AudioOutputUSB audioOut;
// AudioOutputI2S       audioOut;
// AudioControlSGTL5000 codec;

AudioConnection pv0(voice0, 0, fade0, 0);
AudioConnection pv1(voice1, 0, fade1, 0);
AudioConnection pv2(voice2, 0, fade2, 0);
AudioConnection pv3(voice3, 0, fade3, 0);
AudioConnection pf0(fade0, 0, mixer, 0);
AudioConnection pf1(fade1, 0, mixer, 1);
AudioConnection pf2(fade2, 0, mixer, 2);
AudioConnection pf3(fade3, 0, mixer, 3);
AudioConnection pmL(mixer, 0, audioOut, 0);
AudioConnection pmR(mixer, 0, audioOut, 1);

static AudioEffectPhaseVocoder *voice[4] = {&voice0, &voice1, &voice2, &voice3};
static AudioEffectFade *fader[4] = {&fade0, &fade1, &fade2, &fade3};

// Config
static const int NUM_SAMPLES = 10;
static const int NUM_VOICES = 4;
static const float STRETCH_MIN = 0.5f;
static const float STRETCH_MAX = 2.0f;
static const float CONTROL_STEP = 0.01f;
static const float VOICE_GAIN = 0.45f;

// Per-sample settings
struct SampleSettings
{
    float stretch = 1.0f;
    float pitch_st = 0.0f;
    bool reverse = false;
    bool loop = true;
};

SampleSettings settings[NUM_SAMPLES];

// PSRAM sample storage
static const char *sampleFiles[NUM_SAMPLES] = {
    "01.WAV", "02.WAV", "03.WAV", "04.WAV", "05.WAV",
    "06.WAV", "07.WAV", "08.WAV", "09.WAV", "10.WAV"};

int16_t *sampleData[NUM_SAMPLES] = {};
uint32_t sampleFrames[NUM_SAMPLES] = {};


// Voice tracking
struct VoiceState
{
    int sample = -1;        // -1 = free
    uint32_t startTime = 0; // millis() when last triggered (for steal-oldest)
};
VoiceState voices[NUM_VOICES];

int lastSelected = 0; // most recently triggered sample

bool profiling = false;
uint32_t lastProf = 0;

static const float PROF_BUDGET_US = 1e6f / (AUDIO_SAMPLE_RATE_EXACT / (float)AUDIO_BLOCK_SAMPLES);

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi
                                : v;
}

static void applySettings(int vi, int si)
{
    const SampleSettings &s = settings[si];
    voice[vi]->setStretch(s.stretch);
    voice[vi]->setPitchShift(s.pitch_st);
    voice[vi]->setReverse(s.reverse);
    voice[vi]->setLoop(s.loop);
}

// AudioEffectFade runs the ramp in the audio ISR — no per-step delays needed.
// One delay() waits for the fade to finish before the voice is reassigned.
static const int FADE_OUT_MS = 20;
static const int FADE_IN_MS = 10;

static void fadeOutVoice(int vi)
{
    fader[vi]->fadeOut(FADE_OUT_MS);
    delay(FADE_OUT_MS + 3);
    voice[vi]->stop();
}

// Returns a voice index ready to use.  Fades out and reclaims the oldest
// active voice if all slots are occupied.
static int allocVoice()
{
    for (int i = 0; i < NUM_VOICES; i++)
    {
        if (voices[i].sample == -1)
            return i;
    }
    int oldest = 0;
    uint32_t age = voices[0].startTime;
    for (int i = 1; i < NUM_VOICES; i++)
    {
        if (voices[i].startTime < age)
        {
            age = voices[i].startTime;
            oldest = i;
        }
    }
    fadeOutVoice(oldest);
    voices[oldest].sample = -1;
    return oldest;
}

// Push live setting changes to every voice currently playing sample si.
static void pushSettings(int si)
{
    for (int vi = 0; vi < NUM_VOICES; vi++)
    {
        if (voices[vi].sample == si)
            applySettings(vi, si);
    }
}

// Sample trigger
static void triggerSample(int si)
{
    if (!sampleData[si] || sampleFrames[si] == 0)
    {
        Serial.printf("Sample %d not loaded.\n", si + 1);
        return;
    }

    lastSelected = si;
    int vi = allocVoice();

    voice[vi]->setSample(sampleData[si], sampleFrames[si]);
    applySettings(vi, si);
    voice[vi]->play();
    fader[vi]->fadeIn(FADE_IN_MS);

    voices[vi].sample = si;
    voices[vi].startTime = millis();

    Serial.printf("Voice %d -> sample %d  x%.2f  pitch=%.0fst  rev=%s  loop=%s\n",
                  vi, si + 1, 1.0f / settings[si].stretch, settings[si].pitch_st,
                  settings[si].reverse ? "Y" : "N", settings[si].loop ? "Y" : "N");
}

// SD→PSRAM loader, supports 8/16/24-bit mono or stereo WAV @ 44100 Hz.
static bool loadSample(int si)
{
    const char *filename = sampleFiles[si];
    File f = SD.open(filename);
    if (!f)
    {
        Serial.print("Cannot open: ");
        Serial.println(filename);
        return false;
    }
    // Read fmt fields: channels (offset 22), bits-per-sample (offset 34)
    f.seek(22);
    int chanLo = f.read(), chanHi = f.read();
    if (chanLo < 0 || chanHi < 0) { f.close(); return false; }
    const uint16_t numCh = (uint16_t)((chanHi << 8) | chanLo);
    f.seek(34);
    int bpsLo = f.read(), bpsHi = f.read();
    if (bpsLo < 0 || bpsHi < 0) { f.close(); return false; }
    const uint16_t bitsPerSample = (uint16_t)((bpsHi << 8) | bpsLo);
    const uint32_t bytesPerSample = (bitsPerSample + 7u) / 8u;
    // Scan for "data" chunk — header may have extra chunks (LIST, JUNK, etc.)
    f.seek(12);
    uint32_t dataBytes = 0;
    bool foundData = false;
    uint8_t szBuf[4];
    char id[4];
    while (f.available() >= 8)
    {
        f.read(id, 4);
        f.read(szBuf, 4);
        uint32_t chunkSize = (uint32_t)szBuf[0] | ((uint32_t)szBuf[1] << 8) | ((uint32_t)szBuf[2] << 16) | ((uint32_t)szBuf[3] << 24);
        if (id[0]=='d' && id[1]=='a' && id[2]=='t' && id[3]=='a')
        {
            dataBytes = chunkSize;
            foundData = true;
            break;
        }
        f.seek(f.position() + chunkSize + (chunkSize & 1));
    }
    if (!foundData)
    {
        Serial.print("No data chunk: ");
        Serial.println(filename);
        f.close();
        return false;
    }
    const uint32_t frameStride = bytesPerSample * numCh;
    uint32_t frameCount = dataBytes / frameStride;
    if (sampleData[si])
    {
        extmem_free(sampleData[si]);
        sampleData[si] = nullptr;
        sampleFrames[si] = 0;
    }
    sampleData[si] = (int16_t *)extmem_malloc(frameCount * sizeof(int16_t));
    if (!sampleData[si])
    {
        Serial.print("EXTMEM alloc failed: ");
        Serial.println(filename);
        f.close();
        return false;
    }
    uint8_t frameBuf[6]; // max frameStride: stereo 24-bit = 6 bytes
    uint32_t written = 0;
    while (written < frameCount && f.read(frameBuf, frameStride) == frameStride)
    {
        int16_t sample;
        if (bytesPerSample == 1)
            sample = (int16_t)(((uint16_t)frameBuf[0] - 128u) << 8);
        else if (bytesPerSample == 2)
            sample = (int16_t)(((uint16_t)frameBuf[1] << 8) | frameBuf[0]);
        else
            sample = (int16_t)(((uint16_t)frameBuf[2] << 8) | frameBuf[1]);
        sampleData[si][written++] = sample;
    }
    f.close();
    sampleFrames[si] = written;
    Serial.printf("Loaded %s  %lu frames  ch=%u  %ubit  %.1fs\n",
                  filename, written, numCh, bitsPerSample, written / (float)AUDIO_SAMPLE_RATE_EXACT);
    return written > 0;
}

// Help
static void printHelp()
{
    Serial.println("--- Polyphonic Phase Vocoder (4 voices, 10 samples) ---");
    Serial.println("  1-9, 0  trigger sample 1-10");
    Serial.println("  p       faster  (>1.0 speed)");
    Serial.println("  q       slower  (<1.0 speed)");
    Serial.println("  w       pitch +1 semitone");
    Serial.println("  x       pitch -1 semitone");
    Serial.println("  z       reset pitch");
    Serial.println("  r       toggle reverse");
    Serial.println("  l       toggle loop");
    Serial.println("  SPACE   re-trigger last selected");
    Serial.println("  s       stop all");
    Serial.println("  t/y/u   transient threshold 4/8/16");
    Serial.println("  d       toggle profiling (2s report)");
    Serial.println("  h       this help");
    Serial.println("Controls target the last triggered sample.");
}

// setup
void setup()
{
    AudioMemory(80);
    Serial.begin(57600);
    while (!Serial && millis() < 2000)
    {
    }

    // codec.enable();
    // codec.volume(0.5f);

    for (int i = 0; i < NUM_VOICES; i++)
    {
        mixer.gain(i, VOICE_GAIN);
        voice[i]->setTransientThreshold(16.0f);
    }

    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("SD init failed");
        return;
    }

    Serial.println("Loading samples into PSRAM...");
    for (int i = 0; i < NUM_SAMPLES; i++)
        loadSample(i);
    Serial.println("Ready.");
    printHelp();

    triggerSample(0);
}

// loop
void loop()
{
    // sweep up voices
    for (int vi = 0; vi < NUM_VOICES; vi++)
    {
        if (voices[vi].sample != -1 && !voice[vi]->isPlaying())
        {
            voices[vi].sample = -1;
        }
    }

    if (profiling && millis() - lastProf >= 2000)
    {
        lastProf = millis();
        for (int vi = 0; vi < NUM_VOICES; vi++)
        {
            float meanUs;
            uint32_t peakUs;
            if (voice[vi]->getProfiling(meanUs, peakUs))
            {
                Serial.printf("V%d  mean: %.1f us  peak: %lu us  load: %.1f%%\n",
                              vi, meanUs, peakUs, meanUs / PROF_BUDGET_US * 100.0f);
            }
        }
    }

    if (!Serial.available())
        return;
    char key = Serial.read();

    // Sample triggers
    if (key >= '1' && key <= '9')
    {
        lastSelected = key - '1';
        Serial.printf("Sample %d Selected, Hit SPACE to trigger\n", lastSelected +1);
        return;
    }
    if (key == '0')
    {
        lastSelected = 9;
        Serial.print("Sample 0 Selected, Hit SPACE to trigger\n");
        return;
    }

    int si = lastSelected;

    switch (key)
    {
    case ' ':
        triggerSample(si);
        break;
    case 's':
        for (int vi = 0; vi < NUM_VOICES; vi++)
        {
            fader[vi]->fadeOut(FADE_OUT_MS);
        }
        delay(FADE_OUT_MS + 3);
        for (int vi = 0; vi < NUM_VOICES; vi++)
        {
            voice[vi]->stop();
            voices[vi].sample = -1;
        }
        Serial.println("All stopped.");
        break;
    case 'p':
        settings[si].stretch = clampf(settings[si].stretch - CONTROL_STEP, STRETCH_MIN, STRETCH_MAX);
        pushSettings(si);
        Serial.printf("Sample %d speed: %.2f\n", si + 1, 1.0f / settings[si].stretch);
        break;
    case 'q':
        settings[si].stretch = clampf(settings[si].stretch + CONTROL_STEP, STRETCH_MIN, STRETCH_MAX);
        pushSettings(si);
        Serial.printf("Sample %d speed: %.2f\n", si + 1, 1.0f / settings[si].stretch);
        break;
    case 'w':
        settings[si].pitch_st += 1.0f;
        pushSettings(si);
        Serial.printf("Sample %d pitch: %.0fst\n", si + 1, settings[si].pitch_st);
        break;
    case 'x':
        settings[si].pitch_st -= 1.0f;
        pushSettings(si);
        Serial.printf("Sample %d pitch: %.0fst\n", si + 1, settings[si].pitch_st);
        break;
    case 'z':
        settings[si].pitch_st = 0.0f;
        settings[si].stretch = 1.0f;
        pushSettings(si);
        Serial.printf("Sample %d speed: 1.0 pitch: 0st\n", si + 1);
        break;
    case 'r':
        settings[si].reverse = !settings[si].reverse;
        pushSettings(si);
         Serial.printf("Sample %d reverse: %s\n", si + 1, settings[si].reverse ? "on" : "off");
        break;
    case 'l':
        settings[si].loop = !settings[si].loop;
        pushSettings(si);
         Serial.printf("Sample %d loop: %s\n", si + 1, settings[si].loop ? "on" : "off");
        break;
    case 't':
        for (int vi = 0; vi < NUM_VOICES; vi++)
            voice[vi]->setTransientThreshold(4.0f);
        Serial.println("Transient threshold: 4");
        break;
    case 'y':
        for (int vi = 0; vi < NUM_VOICES; vi++)
            voice[vi]->setTransientThreshold(8.0f);
        Serial.println("Transient threshold: 8");
        break;
    case 'u':
        for (int vi = 0; vi < NUM_VOICES; vi++)
            voice[vi]->setTransientThreshold(16.0f);
        Serial.println("Transient threshold: 16");
        break;
    case 'd':
        profiling = !profiling;
        lastProf = millis();
        Serial.println(profiling ? "Profiling: on" : "Profiling: off");
        break;
    case 'h':
        printHelp();
        break;
    }
}
