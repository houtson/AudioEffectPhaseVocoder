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

    // --- Passthrough: bypass phase vocoder at unity stretch and pitch ---
    if (stretch_factor == 1.0f && pitch_ratio == 1.0f) {
        audio_block_t *passBlock = allocate();
        if (passBlock) {
            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                float pos = read_pos + (float)i;
                if (looping && pos >= (float)sample_length)
                    pos = fmodf(pos, (float)sample_length);
                uint32_t idx  = (uint32_t)pos;
                float    frac = pos - (float)idx;
                uint32_t idx1 = (looping && idx + 1 >= sample_length) ? 0 : idx + 1;
                float s0 = (idx  < sample_length) ? sample_data[idx]  * (1.0f / 32768.0f) : 0.0f;
                float s1 = (idx1 < sample_length) ? sample_data[idx1] * (1.0f / 32768.0f) : 0.0f;
                int32_t out = (int32_t)((s0 + frac * (s1 - s0)) * 32768.0f);
                passBlock->data[i] = (int16_t)__SSAT(out, 16);
            }
            transmit(passBlock);
            release(passBlock);
        }
        read_pos += (float)AUDIO_BLOCK_SAMPLES;
        if (read_pos >= (float)sample_length) {
            if (looping) read_pos = fmodf(read_pos, (float)sample_length);
            else         playing = false;
        }
        was_passthrough = true;
        return;
    }

    // Flush stale PV state if we just left passthrough mode — prevents a click.
    if (was_passthrough) {
        memset(input_window, 0, FFT_SIZE * sizeof(float));
        memset(Last_Phase,   0, FFT_SIZE * sizeof(float));
        memset(Phase_Sum,    0, FFT_SIZE * sizeof(float));
        memset(Synth_Accum,  0, (FFT_SIZE + AUDIO_BLOCK_SAMPLES) * sizeof(float));
        memset(Prev_Magn,    0, HALF_FFT_SIZE * sizeof(float));
        prev_flux       = 0.0f;
        was_passthrough = false;
    }

    elapsedMicros elapsed = 0;
    elapsedMicros sectionTimer = 0;

    // Snapshot analysis_hop so it stays consistent across both frames even if
    // setStretch() is called from the main thread mid-update.
    const float hop       = analysis_hop;
    const int   ihop      = (int)hop;
    const float expect    = 2.0f * M_PI * hop / (float)FFT_SIZE;

    // phase_step and norm are fixed to SYNTH_HOP (64), not AUDIO_BLOCK_SAMPLES (128).
    // We run two SYNTH_HOP frames per update() to fill one audio block.
    const float phase_step = 2.0f * M_PI * (float)SYNTH_HOP / AUDIO_SAMPLE_RATE_EXACT;
    const float freq_scale = FREQ_PER_BIN * (float)FFT_SIZE / (2.0f * M_PI * hop);
    const float norm       = (float)SYNTH_HOP / (float)HALF_FFT_SIZE;

    int slide = (ihop < FFT_SIZE) ? ihop : FFT_SIZE - 1;

    audio_block_t *synthBlock = allocate();

    // Run AUDIO_BLOCK_SAMPLES/SYNTH_HOP frames per update() to fill one audio block.
    // With OVER_SAMPLE=16, SYNTH_HOP=64: 2 frames. With OVER_SAMPLE=8, SYNTH_HOP=128: 1 frame.
    const int frames_per_block = AUDIO_BLOCK_SAMPLES / SYNTH_HOP;
    for (int frame = 0; frame < frames_per_block; frame++) {

        // --- Slide the analysis window and fill from sample buffer ---
        memmove(input_window, input_window + slide, (FFT_SIZE - slide) * sizeof(float));

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

        read_pos += hop;
        if (read_pos >= (float)sample_length) {
            if (looping) read_pos = fmodf(read_pos, (float)sample_length);
            else         playing = false;
        }

        prof_t_window += (uint32_t)sectionTimer; sectionTimer = 0;

        // --- Apply analysis window and forward FFT ---
        for (int k = 0; k < FFT_SIZE; k++) {
            FFT_Frame[k] = input_window[k] * window[k];
        }
        arm_cfft_f32(instance, FFT_Frame, 0, 1);
        split_rfft_f32(FFT_Frame, HALF_FFT_SIZE, coefA, coefB, FFT_Split_Frame);

        prof_t_fft += (uint32_t)sectionTimer; sectionTimer = 0;

        // --- Phase analysis: compute true instantaneous frequency per bin ---
        // Spectral flux (sum of positive magnitude increases) is folded in here.
        // It fires only when new spectral content appears — unlike energy ratio,
        // it does not misfire on loud sustained frames.
        float flux = 0.0f;

        for (int k = 0; k < HALF_FFT_SIZE; k++) {
            float real  = FFT_Split_Frame[2 * k];
            float imag  = FFT_Split_Frame[2 * k + 1];
            float magn  = 2.0f * sqrtf(real * real + imag * imag);
            float phase = atan2_fast(imag, real);

            float delta = phase - Last_Phase[k] - expect * (float)k;
            Last_Phase[k] = phase;

            long qpd = (long)(delta / M_PI);
            if (qpd >= 0) qpd += qpd & 1;
            else          qpd -= qpd & 1;
            delta -= M_PI * (float)qpd;

            Synth_Magn[k] = magn;
            Synth_Freq[k] = (float)k * FREQ_PER_BIN + freq_scale * delta;

            float diff = magn - Prev_Magn[k];
            if (diff > 0.0f) flux += diff;
            Prev_Magn[k] = magn;
        }

        prof_t_analysis += (uint32_t)sectionTimer; sectionTimer = 0;

        // --- Transient detection via spectral flux ---
        // Compare instantaneous flux against a smoothed reference.
        // threshold > 1 — lower = more sensitive, higher = fewer resets.
        bool is_transient = (flux > prev_flux * transient_threshold);
        prev_flux = 0.8f * prev_flux + 0.2f * flux;

        // --- Phase synthesis (with optional pitch shift) ---
        //
        // Pitch shifting must happen in the magnitude/frequency domain — before
        // phase accumulation — so that Phase_Sum tracks the correct output frequency.
        // Remapping after synthesis (in the complex domain) produces incoherent phases.
        //
        // FFT_Frame is free at this point and is split into two HALF_FFT_SIZE scratch arrays:
        //   out_magn = FFT_Frame[0 .. HALF_FFT_SIZE-1]
        //   out_freq = FFT_Frame[HALF_FFT_SIZE .. FFT_SIZE-1]
        const float pr = pitch_ratio;
        if (pr != 1.0f) {
            float *out_magn = FFT_Frame;
            float *out_freq = FFT_Frame + HALF_FFT_SIZE;
            memset(FFT_Frame, 0, FFT_SIZE * sizeof(float));

            for (int k = 0; k < HALF_FFT_SIZE; k++) {
                float fdk  = (float)k * pr;
                int   dk0  = (int)fdk;
                int   dk1  = dk0 + 1;
                float frac = fdk - (float)dk0;   // weight towards dk1

                if (dk0 >= HALF_FFT_SIZE) break;

                // Distribute magnitude linearly across the two neighbouring bins.
                // True frequency at each output bin scales with pitch ratio.
                out_magn[dk0] += Synth_Magn[k] * (1.0f - frac);
                out_freq[dk0]  = Synth_Freq[k] * pr;

                if (dk1 < HALF_FFT_SIZE) {
                    out_magn[dk1] += Synth_Magn[k] * frac;
                    out_freq[dk1]  = Synth_Freq[k] * pr;
                }
            }

            for (int k = 0; k < HALF_FFT_SIZE; k++) {
                if (is_transient) {
                    Phase_Sum[k] = atan2_fast(FFT_Split_Frame[2 * k + 1], FFT_Split_Frame[2 * k]);
                } else {
                    Phase_Sum[k] += phase_step * out_freq[k];
                }
                float sinVal, cosVal;
                arm_sin_cos_f32(Phase_Sum[k] * (180.0f / M_PI), &sinVal, &cosVal);
                FFT_Split_Frame[2 * k]     = out_magn[k] * cosVal;
                FFT_Split_Frame[2 * k + 1] = out_magn[k] * sinVal;
            }
        } else {
            for (int k = 0; k < HALF_FFT_SIZE; k++) {
                if (is_transient) {
                    Phase_Sum[k] = atan2_fast(FFT_Split_Frame[2 * k + 1], FFT_Split_Frame[2 * k]);
                } else {
                    Phase_Sum[k] += phase_step * Synth_Freq[k];
                    // Wrap to [-π, π] to prevent float precision loss over time.
                    if (Phase_Sum[k] >  M_PI) Phase_Sum[k] -= 2.0f * M_PI;
                    if (Phase_Sum[k] < -M_PI) Phase_Sum[k] += 2.0f * M_PI;
                }
                FFT_Split_Frame[2 * k]     = Synth_Magn[k] * arm_cos_f32(Phase_Sum[k]);
                FFT_Split_Frame[2 * k + 1] = Synth_Magn[k] * arm_sin_f32(Phase_Sum[k]);
            }
        }

        prof_t_synthesis += (uint32_t)sectionTimer; sectionTimer = 0;

        // --- Inverse FFT ---
        split_rifft_f32(FFT_Split_Frame, HALF_FFT_SIZE, coefA, coefB, IFFT_Synth_Split_Frame);
        arm_cfft_f32(instance, IFFT_Synth_Split_Frame, 1, 1);

        prof_t_ifft += (uint32_t)sectionTimer; sectionTimer = 0;

        // --- Overlap-add into output accumulator ---
        for (int k = 0; k < FFT_SIZE; k++) {
            Synth_Accum[k] += IFFT_Synth_Split_Frame[k] * window[k] * norm;
        }

        // Write this frame's SYNTH_HOP samples into the output block.
        if (synthBlock) {
            arm_float_to_q15(Synth_Accum, synthBlock->data + frame * SYNTH_HOP, SYNTH_HOP);
        }

        // Advance the accumulator by one synthesis hop.
        // Array size is FFT_SIZE + AUDIO_BLOCK_SAMPLES; shift left by SYNTH_HOP.
        memmove(Synth_Accum, Synth_Accum + SYNTH_HOP,
                (FFT_SIZE + AUDIO_BLOCK_SAMPLES - SYNTH_HOP) * sizeof(float));
        memset(Synth_Accum + FFT_SIZE + AUDIO_BLOCK_SAMPLES - SYNTH_HOP, 0,
               SYNTH_HOP * sizeof(float));

        prof_t_ola += (uint32_t)sectionTimer; sectionTimer = 0;
    }

    if (synthBlock) {
        transmit(synthBlock);
        release(synthBlock);
    }

    // --- Total profiling ---
    const uint32_t us = (uint32_t)elapsed;
    prof_sum += us;
    if (us > prof_peak) prof_peak = us;
    prof_count++;

#endif
}
