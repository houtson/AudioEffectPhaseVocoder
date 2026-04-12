# AudioEffectPhaseVocoder

A phase vocoder audio effect for Teensy 4.x, forked from [duff2013/AudioEffectVocoder](https://github.com/duff2013/AudioEffectVocoder) by Colin Duffy.

## About this fork

This fork adds **time-stretching** — playing audio at variable speed without changing pitch. The effect owns its sample buffer directly (loaded from SD or PSRAM) and integrates with the [Teensy Audio Library](https://www.pjrc.com/teensy/td_libs_Audio.html) as a 0-input / 1-output node.

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
vocoder.setPitchShift(3.0f);   // up 3 semitones
float p = vocoder.getPitchShift();

// Transient detection sensitivity (default 16.0)
// Lower = more phase resets on attacks → better transient preservation
// Higher = fewer resets → smoother sustain
vocoder.setTransientThreshold(16.0f);

// Performance profiling
float meanUs; uint32_t peakUs;
if (vocoder.getProfiling(meanUs, peakUs)) { ... }
```

## Example sketch controls (Serial)

| Key   | Action                              |
| ----- | ----------------------------------- |
| SPACE | Start / restart                     |
| s     | Stop                                |
| p     | Faster                              |
| q     | Slower                              |
| 1-3   | Load 01.WAV / 02.WAV / 03.WAV      |
| w     | Pitch up 1 semitone                 |
| x     | Pitch down 1 semitone               |
| z     | Reset pitch to 0                    |
| t     | Transient threshold: 4 (sensitive)  |
| y     | Transient threshold: 8 (default)    |
| u     | Transient threshold: 16 (subtle)    |
| d     | Toggle profiling report             |
| h     | Help                                |

## Memory

The DSP arrays (~75 KB) live in fast DTCM (RAM1). The sample buffer should be placed in OCRAM or PSRAM to avoid overflowing DTCM:

```cpp
DMAMEM int16_t sampleBuffer[44100 * 5];   // Teensy 4.0/4.1 — OCRAM (~430 KB)
EXTMEM int16_t sampleBuffer[44100 * 60];  // Teensy 4.1 + PSRAM — up to ~8 MB
```

## How it works

The phase vocoder uses a Short-Time Fourier Transform (STFT) to analyse overlapping frames of audio, manipulate the phase spectrum, and reconstruct the output. Time-stretching decouples the analysis hop (how fast input is consumed) from the synthesis hop (always 128 samples, locked to Teensy's audio block rate):

```text
stretch_factor = synthesis_hop / analysis_hop
```

- `stretch > 1` → smaller analysis hop → reads input slowly → longer output
- `stretch < 1` → larger analysis hop → reads input fast → shorter output

Transient frames (sharp energy increases) trigger a phase reset — `Phase_Sum` is set to the analysis phase rather than accumulated — which preserves harmonic relationships on drum hits and attacks.

**FFT size:** 1024 — **Overlap:** 8× — **Platform:** Teensy 4.0 / 4.1

## Credits

- Original vocoder: [Colin Duffy](https://github.com/duff2013/AudioEffectVocoder)
- Phase vocoder algorithm based on [Stephan M. Bernsee's smbPitchShift](http://blogs.zynaptiq.com/bernsee/repo/smbPitchShift.cpp)
