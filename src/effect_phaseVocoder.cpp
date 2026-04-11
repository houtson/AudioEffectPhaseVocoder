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
#include <Arduino.h>
#include "effect_phaseVocoder.h"

void AudioEffectPhaseVocoder::update(void) {
#if defined(__ARM_ARCH_7EM__)

    if (!playing) return;

    // Snapshot analysis_hop so it stays consistent within one frame even if
    // setStretch() is called from the main thread mid-update.
    const float hop    = analysis_hop;
    const int   ihop   = (int)hop;            // integer part used for memmove/loop counts
    const float expect = 2.0f * M_PI * hop / (float)FFT_SIZE;

    // --- Slide the analysis window by analysis_hop ---
    // ihop samples are consumed from the sample buffer each frame.
    // At stretch 1.0, hop == AUDIO_BLOCK_SAMPLES (128) == the old STEP_SIZE.
    int slide = (ihop < FFT_SIZE) ? ihop : FFT_SIZE - 1;
    memmove(input_window, input_window + slide, (FFT_SIZE - slide) * sizeof(float));

    // Fill the new `slide` samples with linear interpolation on the fractional read position.
    for (int i = 0; i < slide; i++) {
        float pos = read_pos + (float)i;
        if (looping && pos >= (float)sample_length)
            pos = fmodf(pos, (float)sample_length);

        uint32_t idx  = (uint32_t)pos;
        float    frac = pos - (float)idx;

        uint32_t idx1 = (looping && idx + 1 >= sample_length) ? 0 : idx + 1;

        float s0 = (idx  < sample_length) ? sample_data[idx]  * (1.0f / 32768.0f) : 0.0f;
        float s1 = (idx1 < sample_length) ? sample_data[idx1] * (1.0f / 32768.0f) : 0.0f;

        input_window[FFT_SIZE - slide + i] = s0 + frac * (s1 - s0);
    }

    // Advance read position by analysis_hop (fractional advance preserved across frames)
    read_pos += hop;
    if (read_pos >= (float)sample_length) {
        if (looping) {
            read_pos = fmodf(read_pos, (float)sample_length);
        } else {
            playing = false;
        }
    }

    // --- Apply analysis window and forward FFT ---
    for (int k = 0; k < FFT_SIZE; k++) {
        FFT_Frame[k] = input_window[k] * window[k];
    }

    arm_cfft_f32(instance, FFT_Frame, 0, 1);
    split_rfft_f32(FFT_Frame, HALF_FFT_SIZE, coefA, coefB, FFT_Split_Frame);

    // --- Phase analysis: compute true instantaneous frequency per bin ---
    memset(Synth_Magn, 0, HALF_FFT_SIZE * sizeof(float));
    memset(Synth_Freq, 0, HALF_FFT_SIZE * sizeof(float));

    for (int k = 0; k < HALF_FFT_SIZE; k++) {
        float real  = FFT_Split_Frame[2 * k];
        float imag  = FFT_Split_Frame[2 * k + 1];
        float magn  = 2.0f * sqrtf(real * real + imag * imag);
        float phase = atan2_fast(imag, real);

        // Deviation from expected phase advance for this bin given analysis_hop
        float delta = phase - Last_Phase[k] - expect * (float)k;
        Last_Phase[k] = phase;

        // Wrap delta to [-π, π]
        long qpd = (long)(delta / M_PI);
        if (qpd >= 0) qpd += qpd & 1;
        else          qpd -= qpd & 1;
        delta -= M_PI * (float)qpd;

        // True instantaneous frequency in Hz — independent of hop size
        float true_freq = (float)k * FREQ_PER_BIN
                        + FREQ_PER_BIN * delta * (float)FFT_SIZE / (2.0f * M_PI * hop);

        Synth_Magn[k] = magn;
        Synth_Freq[k] = true_freq;
    }

    // --- Phase synthesis: accumulate output phase at fixed synthesis hop ---
    // Synthesis hop is always AUDIO_BLOCK_SAMPLES regardless of stretch,
    // keeping output locked to Teensy's audio block clock.
    const float synth_hop = (float)AUDIO_BLOCK_SAMPLES;

    for (int k = 0; k < HALF_FFT_SIZE; k++) {
        Phase_Sum[k] += 2.0f * M_PI * synth_hop / AUDIO_SAMPLE_RATE_EXACT * Synth_Freq[k];

        FFT_Split_Frame[2 * k]     = Synth_Magn[k] * arm_cos_f32(Phase_Sum[k]);
        FFT_Split_Frame[2 * k + 1] = Synth_Magn[k] * arm_sin_f32(Phase_Sum[k]);
    }

    // --- Inverse FFT ---
    split_rifft_f32(FFT_Split_Frame, HALF_FFT_SIZE, coefA, coefB, IFFT_Synth_Split_Frame);
    arm_cfft_f32(instance, IFFT_Synth_Split_Frame, 1, 1);

    // --- Overlap-add into output accumulator ---
    // norm = synth_hop / HALF_FFT_SIZE compensates for the Hann window OLA gain
    // and the 1/N scaling of the split-rfft/rifft pair.
    const float norm = synth_hop / (float)HALF_FFT_SIZE;
    for (int k = 0; k < FFT_SIZE; k++) {
        Synth_Accum[k] += IFFT_Synth_Split_Frame[k] * window[k] * norm;
    }

    // --- Output one block and shift accumulator by synthesis hop ---
    audio_block_t *synthBlock = allocate();
    if (synthBlock) {
        arm_float_to_q15(Synth_Accum, synthBlock->data, AUDIO_BLOCK_SAMPLES);
        transmit(synthBlock);
        release(synthBlock);
    }

    memmove(Synth_Accum, Synth_Accum + (int)synth_hop, FFT_SIZE * sizeof(float));
    memset(Synth_Accum + FFT_SIZE, 0, (int)synth_hop * sizeof(float));

#endif
}
