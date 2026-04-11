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
#define OVER_SAMPLE   8
#define HALF_FFT_SIZE (FFT_SIZE / 2)

// Stretch factor limits — analysis_hop must stay within [1, FFT_SIZE)
#define PV_STRETCH_MIN   0.1f
#define PV_STRETCH_MAX   10.0f

class AudioEffectPhaseVocoder : public AudioStream {
public:
    AudioEffectPhaseVocoder() : AudioStream(0, nullptr),
                           sample_data(nullptr),
                           sample_length(0),
                           read_pos(0.0f),
                           playing(false),
                           looping(false),
                           stretch_factor(1.0f),
                           analysis_hop((float)(FFT_SIZE / OVER_SAMPLE))
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
        playing       = false;
        sample_data   = data;
        sample_length = numSamples;
        read_pos      = 0.0f;
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase,   0, FFT_SIZE * sizeof(float));
        memset(Phase_Sum,    0, FFT_SIZE * sizeof(float));
        memset(Synth_Accum,  0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
    }

    // stretch > 1.0 = slower (e.g. 2.0 = double duration)
    // stretch < 1.0 = faster (e.g. 0.5 = half duration)
    void setStretch(float factor) {
        if (factor < PV_STRETCH_MIN) factor = PV_STRETCH_MIN;
        if (factor > PV_STRETCH_MAX) factor = PV_STRETCH_MAX;
        stretch_factor = factor;
        analysis_hop   = (float)AUDIO_BLOCK_SAMPLES / stretch_factor;
    }

    float getStretch() { return stretch_factor; }

    void play()             { if (sample_data) playing = true; }
    void stop()             { playing = false; read_pos = 0.0f; }
    void setLoop(bool loop) { looping = loop; }
    bool isPlaying()        { return playing; }

    // Stub — pitch shifting not yet implemented
    void setPitchShift(float semitones) { (void)semitones; }

    virtual void update(void);

private:
    // Sample buffer
    const int16_t *sample_data;
    uint32_t       sample_length;
    float          read_pos;
    bool           playing;
    bool           looping;

    // Stretch control
    float stretch_factor;
    float analysis_hop;   // = AUDIO_BLOCK_SAMPLES / stretch_factor (can be fractional)

    // DSP resources
    const float *window;
    const float *coefA;
    const float *coefB;
    const arm_cfft_instance_f32 *instance;

    // Fixed constants
    const float FREQ_PER_BIN = AUDIO_SAMPLE_RATE_EXACT / (float)FFT_SIZE;

    float input_window          [FFT_SIZE];
    float Phase_Sum             [FFT_SIZE];
    float FFT_Frame             [FFT_SIZE];
    float Last_Phase            [FFT_SIZE];
    float Synth_Freq            [FFT_SIZE];
    float Synth_Magn            [FFT_SIZE];
    float Synth_Accum           [FFT_SIZE + AUDIO_BLOCK_SAMPLES];
    float FFT_Split_Frame       [FFT_SIZE * 2];
    float IFFT_Synth_Split_Frame[FFT_SIZE * 2];
};

#endif
