/* Phase Vocoder for Teensy 4
 * Copyright (c) 2019, Colin Duffy
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

extern "C" {
    extern float atan2_fast(float y, float x);
    extern void split_rfft_f32 (float * pSrc, uint32_t fftLen, const float * pATable, const float * pBTable, float * pDst);
    extern void split_rifft_f32 (float * pSrc, uint32_t fftLen, const float * pATable, const float * pBTable, float * pDst);
    extern const float win1024_f32[];
    extern const float coefA_512_f32[];
    extern const float coefB_512_f32[];
}

#define FFT_SIZE      1024
#define OVER_SAMPLE   8  // 8 = 8× oversampled FFT  sor 16 (64-sample hop). Higher = better transient response but more CPU.
#define HALF_FFT_SIZE (FFT_SIZE / 2)
#define SYNTH_HOP     (FFT_SIZE / OVER_SAMPLE)   // 64 — synthesis hop (samples per frame)

// Stretch factor limits
#define PV_STRETCH_MIN   0.5f
#define PV_STRETCH_MAX   1.5f

class AudioEffectPhaseVocoder : public AudioStream {
public:
    AudioEffectPhaseVocoder() : AudioStream(0, nullptr),
                           sample_data(nullptr),
                           sample_length(0),
                           read_pos(0.0f),
                           playing(false),
                           looping(false),
                           stretch_factor(1.0f),
                           analysis_hop((float)SYNTH_HOP),
                           prev_energy(0.0f),
                           transient_threshold(8.0f),
                           prof_sum(0), prof_peak(0), prof_count(0),
                           prof_t_window(0), prof_t_fft(0), prof_t_analysis(0),
                           prof_t_synthesis(0), prof_t_ifft(0), prof_t_ola(0)
    {
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase,   0, FFT_SIZE * sizeof(float));
        memset(Phase_Sum,    0, FFT_SIZE * sizeof(float));
        memset(Synth_Accum,  0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
        instance = &arm_cfft_sR_f32_len512;
        window   = win1024_f32;
        coefA    = coefA_512_f32;
        coefB    = coefB_512_f32;
    }

    void setSample(const int16_t *data, uint32_t numSamples) {
        AudioNoInterrupts();
        playing       = false;
        sample_data   = data;
        sample_length = numSamples;
        read_pos      = 0.0f;
        prev_energy   = 0.0f;
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase,   0, FFT_SIZE * sizeof(float));
        memset(Phase_Sum,    0, FFT_SIZE * sizeof(float));
        memset(Synth_Accum,  0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
        AudioInterrupts();
    }

    // stretch > 1.0 = slower (e.g. 2.0 = double duration)
    // stretch < 1.0 = faster (e.g. 0.5 = half duration)
    void setStretch(float factor) {
        if (factor < PV_STRETCH_MIN) factor = PV_STRETCH_MIN;
        if (factor > PV_STRETCH_MAX) factor = PV_STRETCH_MAX;
        // analysis_hop write is 32-bit on Cortex-M7 (atomic), but volatile
        // ensures the ISR always reads the latest value.
        stretch_factor = factor;
        analysis_hop   = (float)SYNTH_HOP / stretch_factor;
    }

    // Transient threshold — lower = more phase resets (sharper attacks).
    // Default 8.0 works well for drums. Tune to taste.
    void setTransientThreshold(float t) { transient_threshold = t; }

    float getStretch()             const { return stretch_factor; }
    float getTransientThreshold()  const { return transient_threshold; }

    void play() {
        if (!sample_data) return;
        AudioNoInterrupts();
        read_pos    = 0.0f;
        prev_energy = 0.0f;
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase,   0, FFT_SIZE * sizeof(float));
        memset(Phase_Sum,    0, FFT_SIZE * sizeof(float));
        memset(Synth_Accum,  0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
        playing = true;
        AudioInterrupts();
    }

    void stop() {
        AudioNoInterrupts();
        playing  = false;
        read_pos = 0.0f;
        AudioInterrupts();
    }

    void setLoop(bool loop) { looping = loop; }
    bool isPlaying()        { return playing; }

    // Returns mean and peak update() execution time in microseconds since the
    // last call, then resets the accumulators. Returns false if no data yet.
    bool getProfiling(float &meanUs, uint32_t &peakUs) {
        AudioNoInterrupts();
        const uint32_t sum   = prof_sum;
        const uint32_t count = prof_count;
        const uint32_t peak  = prof_peak;
        prof_sum = prof_peak = prof_count = 0;
        AudioInterrupts();
        if (count == 0) return false;
        meanUs = (float)sum / (float)count;
        peakUs = peak;
        return true;
    }

    // Per-section mean costs (µs) accumulated since last call.
    // Sections: window fill | forward FFT | phase analysis | phase synthesis | inverse FFT | OLA
    bool getProfilingDetailed(float &tWindow, float &tFft, float &tAnalysis,
                              float &tSynthesis, float &tIfft, float &tOla) {
        AudioNoInterrupts();
        const uint32_t count = prof_count;
        const uint32_t w = prof_t_window, f = prof_t_fft, a = prof_t_analysis;
        const uint32_t s = prof_t_synthesis, i = prof_t_ifft, o = prof_t_ola;
        prof_t_window = prof_t_fft = prof_t_analysis = 0;
        prof_t_synthesis = prof_t_ifft = prof_t_ola = 0;
        AudioInterrupts();
        if (count == 0) return false;
        tWindow    = (float)w / count;
        tFft       = (float)f / count;
        tAnalysis  = (float)a / count;
        tSynthesis = (float)s / count;
        tIfft      = (float)i / count;
        tOla       = (float)o / count;
        return true;
    }

    // Stub — pitch shifting not yet implemented
    void setPitchShift(float semitones) { (void)semitones; }

    virtual void update(void);

private:
    // Sample buffer (written from main thread, read from ISR — use AudioNoInterrupts guards)
    const int16_t    *sample_data;
    uint32_t          sample_length;
    float             read_pos;
    volatile bool     playing;
    bool              looping;

    // Stretch control (analysis_hop read from ISR, written from main thread)
    volatile float stretch_factor;
    volatile float analysis_hop;

    // Transient detection
    float prev_energy;
    float transient_threshold;

    // DSP resources
    const float *window;
    const float *coefA;
    const float *coefB;
    const arm_cfft_instance_f32 *instance;

    // Fixed constant
    const float FREQ_PER_BIN = AUDIO_SAMPLE_RATE_EXACT / (float)FFT_SIZE;

    // Profiling — total
    volatile uint32_t prof_sum;
    volatile uint32_t prof_peak;
    volatile uint32_t prof_count;
    // Profiling — per section
    volatile uint32_t prof_t_window;
    volatile uint32_t prof_t_fft;
    volatile uint32_t prof_t_analysis;
    volatile uint32_t prof_t_synthesis;
    volatile uint32_t prof_t_ifft;
    volatile uint32_t prof_t_ola;

    float    input_window          [FFT_SIZE];
    float    Phase_Sum             [FFT_SIZE];
    float    FFT_Frame             [FFT_SIZE];
    float    Last_Phase            [FFT_SIZE];
    float    Synth_Freq            [FFT_SIZE];
    float    Synth_Magn            [FFT_SIZE];
    float    Synth_Accum           [FFT_SIZE + AUDIO_BLOCK_SAMPLES];
    float    FFT_Split_Frame       [FFT_SIZE * 2];
    float    IFFT_Synth_Split_Frame[FFT_SIZE * 2];
};

#endif
