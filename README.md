# AudioEffectPhaseVocoder

A phase vocoder audio effect for Teensy 3.5+/4, forked from [duff2013/AudioEffectVocoder](https://github.com/duff2013/AudioEffectVocoder) by Colin Duffy.

## About this fork

This fork extends the original phase vocoder with **time-stretching** — the ability to play back audio samples at variable speeds without changing pitch. The effect is designed for use with the [Teensy Audio Library](https://www.pjrc.com/teensy/td_libs_Audio.html).

## Status

Active development. See the [Development Plan](Development%20Plan%20-%20Phase%20Vocoder%20Time-Str) for the full roadmap.

| Phase | Description | Status |
| ----- | ----------- | ------ |
| 1 | Cleanup & bug fixes | In progress |
| 2 | Internalize sample playback | Planned |
| 3 | Time-stretching | Planned |
| 4 | Transient handling (drums) | Planned |

## Planned API

```cpp
AudioEffectPhaseVocoder vocoder;

vocoder.setSample(data, numSamples); // point to a sample buffer
vocoder.setStretch(2.0f);           // 2.0 = double duration (slower)
vocoder.setLoop(true);
vocoder.play();
```

## Parameters

- `setStretch(float factor)` — `1.0` = normal speed, `2.0` = half speed, `0.5` = double speed
- `setSample(const int16_t *data, uint32_t numSamples)` — load a sample buffer
- `play()` / `stop()` / `isPlaying()`
- `setLoop(bool loop)`

## How it works

The phase vocoder uses a Short-Time Fourier Transform (STFT) to analyze overlapping frames of audio, manipulate the phase spectrum, and reconstruct the output. Time-stretching is achieved by decoupling the analysis hop (how fast input is consumed) from the synthesis hop (always 128 samples, locked to Teensy's audio block rate):

```text
stretch_factor = synthesis_hop / analysis_hop
```

- `stretch > 1` → reads input slowly → output is longer (slower playback)
- `stretch < 1` → reads input fast → output is shorter (faster playback)

**FFT size:** 1024 — **Overlap:** 8x — **Platform:** Teensy 3.5+ / 4.x

## Credits

- Original vocoder: [Colin Duffy](https://github.com/duff2013/AudioEffectVocoder)
- Phase vocoder algorithm based on [Stephan M. Bernsee's smbPitchShift](http://blogs.zynaptiq.com/bernsee/repo/smbPitchShift.cpp)
