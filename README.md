# AudioEffectPhaseVocoder

A phase vocoder audio effect for Teensy 4.x, forked from [duff2013/AudioEffectVocoder](https://github.com/duff2013/AudioEffectVocoder) by Colin Duffy.

## About this fork

This fork adds **time-stretching**, **pitch shifting**, and **reverse playback** — playing audio at variable speed and pitch without the constraints of standard sample playback. The effect owns its sample buffer directly (loaded from SD or PSRAM) and integrates with the [Teensy Audio Library](https://www.pjrc.com/teensy/td_libs_Audio.html) as a 0-input / 1-output node.

**Target hardware:** Teensy 4.0 / 4.1 + Audio Shield (SGTL5000). Requires Teensy 4 due to the CMSIS 5 FFT API (`arm_cfft_f32`).

## API

```cpp
AudioEffectPhaseVocoder vocoder;

// Load a sample (mono 16-bit PCM, 44100 Hz)
vocoder.setSample(sampleBuffer, numSamples);

// Playback control
vocoder.play();
vocoder.stop();
vocoder.setLoop(true);
bool playing = vocoder.isPlaying();

// Time-stretching
// > 1.0 = slower (e.g. 2.0 = double duration)
// < 1.0 = faster (e.g. 0.5 = half duration)
vocoder.setStretch(1.5f);
float s = vocoder.getStretch();

// Pitch shifting (semitones). 0 = no shift, +12 = octave up, -12 = octave down.
// Independent of time-stretching — both can be applied simultaneously.
vocoder.setPitchShift(3.0f);
float p = vocoder.getPitchShift();

// Reverse playback — plays the sample backwards through the phase vocoder.
// Time-stretching and pitch shifting apply in reverse as normal.
vocoder.setReverse(true);
bool r = vocoder.isReverse();

// Transient detection sensitivity (default 16.0)
// Lower = more phase resets on attacks → better transient preservation
// Higher = fewer resets → smoother sustain
vocoder.setTransientThreshold(16.0f);

// Performance profiling — returns mean and peak update() time in µs since last call.
float meanUs; uint32_t peakUs;
if (vocoder.getProfiling(meanUs, peakUs)) { ... }
```

## Examples

### `timestretch_example.ino`

Single-voice sketch. Loads up to three WAV files from SD card and plays them with real-time time-stretch and pitch control over Serial.

| Key   | Action                             |
| ----- | ---------------------------------- |
| SPACE | Start / restart                    |
| s     | Stop                               |
| 1–3   | Load 01.WAV / 02.WAV / 03.WAV      |
| p     | Faster                             |
| q     | Slower                             |
| w     | Pitch up 1 semitone                |
| x     | Pitch down 1 semitone              |
| z     | Reset pitch to 0                   |
| r     | Toggle reverse                     |
| t     | Transient threshold: 4 (sensitive) |
| y     | Transient threshold: 8             |
| u     | Transient threshold: 16 (subtle)   |
| d     | Toggle profiling report            |
| h     | Help                               |

### `poly_example.ino`

Four-voice polyphonic sketch. Loads 10 WAV files into PSRAM at startup. Each voice is independently triggerable with per-sample persistent settings. Uses steal-oldest voice allocation with `AudioEffectFade` crossfades.

| Key     | Action                                        |
| ------- | --------------------------------------------- |
| 1–9, 0  | Trigger sample 1–10                           |
| SPACE   | Re-trigger last selected sample               |
| s       | Stop all voices                               |
| p       | Faster (last selected sample)                 |
| q       | Slower (last selected sample)                 |
| w       | Pitch up 1 semitone                           |
| x       | Pitch down 1 semitone                         |
| z       | Reset pitch to 0                              |
| r       | Toggle reverse                                |
| l       | Toggle loop                                   |
| t/y/u   | Transient threshold 4 / 8 / 16 (all voices)   |
| d       | Toggle profiling report (2 s interval)        |
| h       | Help                                          |

Controls apply to the most recently triggered sample. Settings persist between triggers.

3 test WAV files are included in the `test/` directory — copy them to an SD card.

## Memory

The DSP arrays (~67 KB) live in fast DTCM (RAM1). Sample buffers should be placed in OCRAM or PSRAM to avoid overflowing DTCM:

```cpp
DMAMEM int16_t sampleBuffer[44100 * 5];   // Teensy 4.0/4.1 — OCRAM (~430 KB)
EXTMEM int16_t sampleBuffer[44100 * 60];  // Teensy 4.1 + PSRAM — up to ~8 MB
```

## How it works

The phase vocoder uses a Short-Time Fourier Transform (STFT) to analyse overlapping frames of audio, manipulate the phase spectrum, and reconstruct the output via overlap-add.

**Time-stretching** decouples the analysis hop (how fast input is consumed) from the synthesis hop (always 128 samples, locked to Teensy's audio block rate):

```text
stretch_factor = synthesis_hop / analysis_hop
```

- `stretch > 1` → smaller analysis hop → reads input slowly → longer output
- `stretch < 1` → larger analysis hop → reads input fast → shorter output

**Reverse playback** rebuilds the analysis window each frame by reading samples in descending order (`window[j] = sample[read_pos - j]`), giving the FFT a genuinely time-reversed view. Phase advances in the same direction as forward playback, so all existing phase accumulation logic applies unchanged. At unity stretch and pitch, reverse falls through to a lightweight passthrough path.

**Transient detection** via spectral flux compares the instantaneous sum of positive magnitude increases against a smoothed reference. On a detected transient, `Phase_Sum` is set to the current analysis phase rather than accumulated, which preserves harmonic coherence on drum hits and attacks.

**FFT size:** 1024 — **Overlap:** 8× — **Platform:** Teensy 4.0 / 4.1

## Credits

- Original vocoder: [Colin Duffy](https://github.com/duff2013/AudioEffectVocoder)
- Phase vocoder algorithm based on [Stephan M. Bernsee's smbPitchShift](http://blogs.zynaptiq.com/bernsee/repo/smbPitchShift.cpp)
