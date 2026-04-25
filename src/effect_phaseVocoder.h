/* Phase Vocoder for Teensy 4
 * Copyright (c) 2019, Colin Duffy
 * Copyright (c) 2026, Paul Feely
 *
 * Based off Stephan M. Bernsee smbPitchShift.
 * http://blogs.zynaptiq.com/bernsee/repo/smbPitchShift.cpp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef phase_vocoder_h_
#define phase_vocoder_h_

#include "Arduino.h"
#include <Audio.h>
#include "AudioStream.h"
#include <arm_math.h>
#include "arm_const_structs.h"

extern "C"
{
    extern float atan2_fast(float y, float x);
    extern void split_rfft_f32(float *pSrc, uint32_t fftLen, const float *pATable, const float *pBTable, float *pDst);
    extern void split_rifft_f32(float *pSrc, uint32_t fftLen, const float *pATable, const float *pBTable, float *pDst);
    extern const float win1024_f32[];
    extern const float coefA_512_f32[];
    extern const float coefB_512_f32[];
}

#define FFT_SIZE 1024
#define OVER_SAMPLE 8
#define HALF_FFT_SIZE (FFT_SIZE / 2)
#define SYNTH_HOP (FFT_SIZE / OVER_SAMPLE) // synthesis hop (samples per frame)

// Stretch factor limits
#define PV_STRETCH_MIN 0.5f
#define PV_STRETCH_MAX 2.0f

class AudioEffectPhaseVocoder : public AudioStream
{
public:
    AudioEffectPhaseVocoder() : AudioStream(0, nullptr),
                                sample_data(nullptr),
                                sample_length(0),
                                read_pos(0.0f),
                                playing(false),
                                looping(false),
                                stretch_factor(1.0f),
                                analysis_hop((float)SYNTH_HOP),
                                pitch_ratio(1.0f),
                                pitch_semitones(0.0f),
                                reverse(false),
                                was_passthrough(true),
                                prev_flux(0.0f),
                                transient_threshold(16.0f),
                                prof_sum(0), prof_peak(0), prof_count(0)
    {
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase, 0, HALF_FFT_SIZE * sizeof(float));
        memset(Phase_Sum, 0, HALF_FFT_SIZE * sizeof(float));
        memset(Synth_Accum, 0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
        memset(Prev_Magn, 0, HALF_FFT_SIZE * sizeof(float));
        instance = &arm_cfft_sR_f32_len512;
        window = win1024_f32;
        coefA = coefA_512_f32;
        coefB = coefB_512_f32;
    }

    // Set the sample to be played. Sample data must remain valid while playing.
    void setSample(const int16_t *data, uint32_t numSamples)
    {
        AudioNoInterrupts();
        playing = false;
        sample_data = data;
        sample_length = numSamples;
        read_pos = 0.0f;
        prev_flux = 0.0f;
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase, 0, HALF_FFT_SIZE * sizeof(float));
        memset(Phase_Sum, 0, HALF_FFT_SIZE * sizeof(float));
        memset(Synth_Accum, 0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
        memset(Prev_Magn, 0, HALF_FFT_SIZE * sizeof(float));
        AudioInterrupts();
    }

    // stretch > 1.0 = slower (e.g. 2.0 = double duration)
    // stretch < 1.0 = faster (e.g. 0.5 = half duration)
    void setStretch(float factor)
    {
        if (factor < PV_STRETCH_MIN)
            factor = PV_STRETCH_MIN;
        if (factor > PV_STRETCH_MAX)
            factor = PV_STRETCH_MAX;
        // analysis_hop write is 32-bit on Cortex-M7 (atomic), but volatile
        // ensures the ISR always reads the latest value.
        stretch_factor = factor;
        analysis_hop = (float)SYNTH_HOP / stretch_factor;
    }

    // Transient threshold — lower = more phase resets (sharper attacks).
    // Default 16.0. Tune to taste.
    void setTransientThreshold(float t) { transient_threshold = t; }

    float getStretch() const { return stretch_factor; }
    float getTransientThreshold() const { return transient_threshold; }

    void setReverse(bool r) { reverse = r; }
    bool isReverse() { return reverse; }

    void play()
    {
        if (!sample_data)
            return;
        AudioNoInterrupts();
        read_pos = reverse ? (float)(sample_length - 1) : 0.0f;
        prev_flux = 0.0f;
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase, 0, HALF_FFT_SIZE * sizeof(float));
        memset(Phase_Sum, 0, HALF_FFT_SIZE * sizeof(float));
        memset(Synth_Accum, 0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
        memset(Prev_Magn, 0, HALF_FFT_SIZE * sizeof(float));
        playing = true;
        AudioInterrupts();
    }

    void stop()
    {
        AudioNoInterrupts();
        playing = false;
        read_pos = 0.0f;
        AudioInterrupts();
    }

    void setLoop(bool loop) { looping = loop; }
    bool isPlaying() { return playing; }

    // Returns mean and peak update() execution time in microseconds since the
    // last call, then resets the accumulators. Returns false if no data yet.
    bool getProfiling(float &meanUs, uint32_t &peakUs)
    {
        AudioNoInterrupts();
        const uint32_t sum = prof_sum;
        const uint32_t count = prof_count;
        const uint32_t peak = prof_peak;
        prof_sum = prof_peak = prof_count = 0;
        AudioInterrupts();
        if (count == 0)
            return false;
        meanUs = (float)sum / (float)count;
        peakUs = peak;
        return true;
    }

    // Pitch shift in semitones. 0 = no shift, +12 = one octave up, -12 = one octave down.
    void setPitchShift(float semitones)
    {
        pitch_semitones = semitones;
        pitch_ratio = powf(2.0f, semitones / 12.0f);
    }
    float getPitchShift() const { return pitch_semitones; }

    virtual void update(void);

private:
    // Sample buffer (written from main thread, read from ISR — use AudioNoInterrupts guards)
    const int16_t *sample_data;
    uint32_t sample_length;
    float read_pos;
    volatile bool playing;
    bool looping;

    // Stretch control (analysis_hop read from ISR, written from main thread)
    volatile float stretch_factor;
    volatile float analysis_hop;

    // Pitch shifting
    float pitch_ratio; // 1.0 = no shift, 2.0 = octave up, 0.5 = octave down
    float pitch_semitones;

    bool reverse;

    // Tracks whether last frame was passthrough so PV state can be flushed on re-entry.
    bool was_passthrough;

    // Transient detection (spectral flux)
    float prev_flux;
    float transient_threshold;
    float Prev_Magn[HALF_FFT_SIZE];

    // DSP resources
    const float *window;
    const float *coefA;
    const float *coefB;
    const arm_cfft_instance_f32 *instance;

    // Fixed constant
    const float FREQ_PER_BIN = AUDIO_SAMPLE_RATE_EXACT / (float)FFT_SIZE;

    // Profiling
    volatile uint32_t prof_sum;
    volatile uint32_t prof_peak;
    volatile uint32_t prof_count;

    float input_window[FFT_SIZE];
    float Phase_Sum[HALF_FFT_SIZE];
    float FFT_Frame[FFT_SIZE];
    float Last_Phase[HALF_FFT_SIZE];
    float Synth_Freq[HALF_FFT_SIZE];
    float Synth_Magn[HALF_FFT_SIZE];
    float Synth_Accum[FFT_SIZE + AUDIO_BLOCK_SAMPLES];
    float FFT_Split_Frame[FFT_SIZE * 2];
    float IFFT_Synth_Split_Frame[FFT_SIZE * 2];
};

#endif
