
// -- see: https://github.com/Sensirion/embedded-sgp/blob/master/sgp40_voc_index/sensirion_voc_algorithm.c

// BSD 3-Clause License

// Copyright (c) 2018, Sensirion AG
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

const VOCALGORITHM_SAMPLING_INTERVAL: f64 = 1.0;
const VOCALGORITHM_INITIAL_BLACKOUT: f64 = 45.0;
const VOCALGORITHM_VOC_INDEX_GAIN: f64 = 230.0;
const VOCALGORITHM_SRAW_STD_INITIAL: f64 = 50.0;
const VOCALGORITHM_SRAW_STD_BONUS: f64 = 220.0;
const VOCALGORITHM_TAU_MEAN_VARIANCE_HOURS: f64 = 12.0;
const VOCALGORITHM_TAU_INITIAL_MEAN: f64 = 20.0;
const VOCALGORITHM_INIT_DURATION_MEAN: f64 = 3600.0 * 0.75;
const VOCALGORITHM_INIT_TRANSITION_MEAN: f64 = 0.01;
const VOCALGORITHM_TAU_INITIAL_VARIANCE: f64 = 2500.0;
const VOCALGORITHM_INIT_DURATION_VARIANCE: f64 = 3600. * 1.45;
const VOCALGORITHM_INIT_TRANSITION_VARIANCE: f64 = 0.01;
const VOCALGORITHM_GATING_THRESHOLD: f64 = 340.0;
const VOCALGORITHM_GATING_THRESHOLD_INITIAL: f64 = 510.0;
const VOCALGORITHM_GATING_THRESHOLD_TRANSITION: f64 = 0.09;
const VOCALGORITHM_GATING_MAX_DURATION_MINUTES: f64 = 60.0 * 3.0;
const VOCALGORITHM_GATING_MAX_RATIO: f64 = 0.3;
const VOCALGORITHM_SIGMOID_L: f64 = 500.0;
const VOCALGORITHM_SIGMOID_K: f64 = -0.0065;
const VOCALGORITHM_SIGMOID_X0: f64 = 213.0;
const VOCALGORITHM_VOC_INDEX_OFFSET_DEFAULT: f64 = 100.0;
const VOCALGORITHM_LP_TAU_FAST: f64 = 20.0;
const VOCALGORITHM_LP_TAU_SLOW: f64 = 500.0;
const VOCALGORITHM_LP_ALPHA: f64 = -0.2;
#[allow(dead_code)]
const VOCALGORITHM_PERSISTENCE_UPTIME_GAMMA: f64 = 3.0 * 3600.0;
const VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING: f64 = 64.0;
const VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_FIX16_MAX: f64 = 32767.0;


// -- Struct to hold all the states of the VOC algorithm.
pub struct VocAlgorithmParams {
    #[allow(dead_code)]
    m_voc_index_offset: f64,
    #[allow(dead_code)]
    m_tau_mean_variance_hours: f64,
    #[allow(dead_code)]
    m_gating_max_duration_minutes: f64,
    #[allow(dead_code)]
    m_sraw_std_initial: f64,
    m_uptime: f64,
    m_sraw: f64,
    m_voc_index: f64,
    m_mean_variance_estimator_gating_max_duration_minutes: f64,
    m_mean_variance_estimator_initialized: bool,
    m_mean_variance_estimator_mean: f64,
    m_mean_variance_estimator_sraw_offset: f64,
    m_mean_variance_estimator_std: f64,
    m_mean_variance_estimator_gamma: f64,
    m_mean_variance_estimator_gamma_initial_mean: f64,
    m_mean_variance_estimator_gamma_initial_variance: f64,
    m_mean_variance_estimator_gamma_mean: f64,
    m_mean_variance_estimator_gamma_variance: f64,
    m_mean_variance_estimator_uptime_gamma: f64,
    m_mean_variance_estimator_uptime_gating: f64,
    m_mean_variance_estimator_gating_duration_minutes: f64,
    m_mean_variance_estimator_sigmoid_l: f64,
    m_mean_variance_estimator_sigmoid_k: f64,
    m_mean_variance_estimator_sigmoid_x0: f64,
    m_mox_model_sraw_std: f64,
    m_mox_model_sraw_mean: f64,
    m_sigmoid_scaled_offset: f64,
    m_adaptive_lowpass_a1: f64,
    m_adaptive_lowpass_a2: f64,
    m_adaptive_lowpass_initialized: bool,
    m_adaptive_lowpass_x1: f64,
    m_adaptive_lowpass_x2: f64,
    m_adaptive_lowpass_x3: f64,
}

impl VocAlgorithmParams {
    pub fn new() -> VocAlgorithmParams {
        VocAlgorithmParams {
            m_voc_index_offset: VOCALGORITHM_VOC_INDEX_OFFSET_DEFAULT,
            m_tau_mean_variance_hours: VOCALGORITHM_TAU_MEAN_VARIANCE_HOURS,
            m_gating_max_duration_minutes: VOCALGORITHM_GATING_MAX_DURATION_MINUTES,
            m_sraw_std_initial: VOCALGORITHM_SRAW_STD_INITIAL,
            m_uptime: 0.0,
            m_sraw: 0.0,
            m_voc_index: 0.0,
            m_mean_variance_estimator_gating_max_duration_minutes: VOCALGORITHM_GATING_MAX_DURATION_MINUTES,
            m_mean_variance_estimator_initialized: false,
            m_mean_variance_estimator_mean: 0.0,
            m_mean_variance_estimator_sraw_offset: 0.0,
            m_mean_variance_estimator_std: VOCALGORITHM_SRAW_STD_INITIAL,
            m_mean_variance_estimator_gamma: (VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING * VOCALGORITHM_SAMPLING_INTERVAL / 3600.0) / (VOCALGORITHM_TAU_MEAN_VARIANCE_HOURS + (VOCALGORITHM_SAMPLING_INTERVAL / 3600.0)),
            m_mean_variance_estimator_gamma_initial_mean: (VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING * VOCALGORITHM_SAMPLING_INTERVAL) / (VOCALGORITHM_TAU_INITIAL_MEAN + VOCALGORITHM_SAMPLING_INTERVAL),
            m_mean_variance_estimator_gamma_initial_variance: (VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING * VOCALGORITHM_SAMPLING_INTERVAL) / (VOCALGORITHM_TAU_INITIAL_VARIANCE + VOCALGORITHM_SAMPLING_INTERVAL),
            m_mean_variance_estimator_gamma_mean: 0.0,
            m_mean_variance_estimator_gamma_variance: 0.0,
            m_mean_variance_estimator_uptime_gamma: 0.0,
            m_mean_variance_estimator_uptime_gating: 0.0,
            m_mean_variance_estimator_gating_duration_minutes: 0.0,
            m_mean_variance_estimator_sigmoid_l: 0.0,
            m_mean_variance_estimator_sigmoid_k: 0.0,
            m_mean_variance_estimator_sigmoid_x0: 0.0,
            m_mox_model_sraw_std: VOCALGORITHM_SRAW_STD_INITIAL,
            m_mox_model_sraw_mean: 0.0,
            m_sigmoid_scaled_offset: VOCALGORITHM_VOC_INDEX_OFFSET_DEFAULT,
            m_adaptive_lowpass_a1: VOCALGORITHM_SAMPLING_INTERVAL / (VOCALGORITHM_LP_TAU_FAST + VOCALGORITHM_SAMPLING_INTERVAL),
            m_adaptive_lowpass_a2: VOCALGORITHM_SAMPLING_INTERVAL / (VOCALGORITHM_LP_TAU_SLOW + VOCALGORITHM_SAMPLING_INTERVAL),
            m_adaptive_lowpass_initialized: false,
            m_adaptive_lowpass_x1: 0.0,
            m_adaptive_lowpass_x2: 0.0,
            m_adaptive_lowpass_x3: 0.0,
        }
    }

    pub fn process(&mut self, sraw: u16) -> f64 {
        let mut sraw = sraw;
        if self.m_uptime <= VOCALGORITHM_INITIAL_BLACKOUT {
            self.m_uptime += VOCALGORITHM_SAMPLING_INTERVAL;
        } else {
            if sraw > 0 && sraw < 65000 {
                if sraw < 20001 {
                    sraw = 20001;
                } else if sraw > 52767 {
                    sraw = 52767;
                }
                self.m_sraw = (sraw - 20000) as f64;
            }
            self.m_voc_index = self.mox_model_process(self.m_sraw);
            self.m_voc_index = self.sigmoid_scaled_process(self.m_voc_index);
            self.m_voc_index = self.adaptive_lowpass_process(self.m_voc_index);
            if self.m_voc_index < 0.5 {
                self.m_voc_index = 0.5;
            }
            if self.m_sraw > 0.0 {
                self.mean_variance_estimator_process(self.m_sraw, self.m_voc_index);

                self.mox_model_set_parameters(
                    self.mean_variance_estimator_get_std(), 
                    self.mean_variance_estimator_get_mean()
                );
            }
        }
        self.m_voc_index + 0.5
    }    

    fn mox_model_set_parameters(&mut self, sraw_std: f64, sraw_mean: f64) {
        self.m_mox_model_sraw_std = sraw_std;
        self.m_mox_model_sraw_mean = sraw_mean;
    }

    fn mox_model_process(&mut self, sraw: f64) -> f64 {
        ((sraw -self.m_mox_model_sraw_mean) / (-(self.m_mox_model_sraw_std + VOCALGORITHM_SRAW_STD_BONUS))) * VOCALGORITHM_VOC_INDEX_GAIN        
    }

    fn sigmoid_scaled_process(&mut self, sample: f64) -> f64 {
        let x = VOCALGORITHM_SIGMOID_K * (sample - VOCALGORITHM_SIGMOID_X0);
        if x < -50.0 {
            return VOCALGORITHM_SIGMOID_L;
        } else if x > 50.0 {
            return 0.0;
        } else {            
            if sample >= 0.0 {
                let shift = (VOCALGORITHM_SIGMOID_L - (5.0 * self.m_sigmoid_scaled_offset)) / 4.0;
                return (VOCALGORITHM_SIGMOID_L + shift) / ((1.0 + x.exp()) - shift);
            } else {
                return (self.m_sigmoid_scaled_offset / VOCALGORITHM_VOC_INDEX_OFFSET_DEFAULT) *
                    (VOCALGORITHM_SIGMOID_L / (1.0 + x.exp()))
            }
        }
    }

    fn adaptive_lowpass_process(&mut self, sample: f64) -> f64 {
        if !self.m_adaptive_lowpass_initialized {
            self.m_adaptive_lowpass_x1 = sample;
            self.m_adaptive_lowpass_x2 = sample;
            self.m_adaptive_lowpass_x3 = sample;
            self.m_adaptive_lowpass_initialized = true;
        }
        self.m_adaptive_lowpass_x1 = ((1.0 - self.m_adaptive_lowpass_a1) * self.m_adaptive_lowpass_x1) + (self.m_adaptive_lowpass_a1 * sample);
        self.m_adaptive_lowpass_x2 = ((1.0 - self.m_adaptive_lowpass_a2) * self.m_adaptive_lowpass_x2) + (self.m_adaptive_lowpass_a2 * sample);
        let mut abs_delta = self.m_adaptive_lowpass_x1 - self.m_adaptive_lowpass_a2;
        if abs_delta < 0.0 {
            abs_delta = -abs_delta;
        }
        let f1 = (VOCALGORITHM_LP_ALPHA * abs_delta).exp();
        let tau_a = ((VOCALGORITHM_LP_TAU_SLOW - VOCALGORITHM_LP_TAU_FAST) * f1) + VOCALGORITHM_LP_TAU_FAST;
        let a3 = VOCALGORITHM_SAMPLING_INTERVAL / (VOCALGORITHM_SAMPLING_INTERVAL + tau_a);
        self.m_adaptive_lowpass_x3 = ((1.0 -a3) * self.m_adaptive_lowpass_x3) + (a3 * sample);
        self.m_adaptive_lowpass_x3
    }

    fn mean_variance_estimator_get_std(&self) -> f64 {
        self.m_mean_variance_estimator_std
    }

    fn mean_variance_estimator_get_mean(&self) -> f64 {
        self.m_mean_variance_estimator_mean + self.m_mean_variance_estimator_sraw_offset
    }

    fn mean_variance_estimator_sigmoid_set_parameters(&mut self, l: f64, x0: f64, k: f64) {
        self.m_mean_variance_estimator_sigmoid_l = l;
        self.m_mean_variance_estimator_sigmoid_k = k;
        self.m_mean_variance_estimator_sigmoid_x0 = x0;
    }

    fn mean_variance_estimator_sigmoid_process(&mut self, sample: f64) -> f64 {
        let x = self.m_mean_variance_estimator_sigmoid_k * (sample - self.m_mean_variance_estimator_sigmoid_x0);
        if x < -50.0 {
            return self.m_mean_variance_estimator_sigmoid_l;
        } else if x > 50.0 {
            return 0.0;
        } else {            
            return self.m_mean_variance_estimator_sigmoid_l / (1.0 + x.exp());            
        }
    }

    fn mean_variance_estimator_calculate_gamma(&mut self, voc_index_from_prior: f64) {
        let uptime_limit = VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_FIX16_MAX - VOCALGORITHM_SAMPLING_INTERVAL;
        if self.m_mean_variance_estimator_uptime_gamma < uptime_limit {
            self.m_mean_variance_estimator_uptime_gamma = self.m_mean_variance_estimator_uptime_gamma + VOCALGORITHM_SAMPLING_INTERVAL;
        }
        if self.m_mean_variance_estimator_uptime_gating < uptime_limit {
            self.m_mean_variance_estimator_uptime_gating = self.m_mean_variance_estimator_uptime_gating + VOCALGORITHM_SAMPLING_INTERVAL;
        }
        self.mean_variance_estimator_sigmoid_set_parameters(1.0, VOCALGORITHM_INIT_DURATION_MEAN, VOCALGORITHM_INIT_TRANSITION_MEAN);
        let sigmoid_gamma_mean = self.mean_variance_estimator_sigmoid_process(self.m_mean_variance_estimator_uptime_gamma);
        
        let gamma_mean = self.m_mean_variance_estimator_gamma + 
            ((self.m_mean_variance_estimator_gamma_initial_mean - self.m_mean_variance_estimator_gamma) * sigmoid_gamma_mean);

        let sigmoid_uptime_gating = self.mean_variance_estimator_sigmoid_process(self.m_mean_variance_estimator_uptime_gating);
        let gating_threshold_mean = VOCALGORITHM_GATING_THRESHOLD + 
            ((VOCALGORITHM_GATING_THRESHOLD_INITIAL - VOCALGORITHM_GATING_THRESHOLD) * sigmoid_uptime_gating);
        
        self.mean_variance_estimator_sigmoid_set_parameters(1.0, gating_threshold_mean, VOCALGORITHM_GATING_THRESHOLD_TRANSITION);

        let sigmoid_gating_mean = self.mean_variance_estimator_sigmoid_process(voc_index_from_prior);
        self.m_mean_variance_estimator_gamma_mean = sigmoid_gating_mean * gamma_mean;

        self.mean_variance_estimator_sigmoid_set_parameters(1.0, VOCALGORITHM_INIT_DURATION_VARIANCE, VOCALGORITHM_INIT_TRANSITION_VARIANCE);

        let sigmoid_gamma_variance = self.mean_variance_estimator_sigmoid_process(self.m_mean_variance_estimator_uptime_gamma);
        let gamma_variance = self.m_mean_variance_estimator_gamma + 
            ((self.m_mean_variance_estimator_gamma_initial_variance - self.m_mean_variance_estimator_gamma) * (sigmoid_gamma_variance - sigmoid_gamma_mean));

        let sigmoid_threshold_variance = self.mean_variance_estimator_sigmoid_process(self.m_mean_variance_estimator_uptime_gating);
        let gating_threshold_variance = VOCALGORITHM_GATING_THRESHOLD + 
            (VOCALGORITHM_GATING_THRESHOLD_INITIAL - VOCALGORITHM_GATING_THRESHOLD) * (sigmoid_threshold_variance);
        
        self.mean_variance_estimator_sigmoid_set_parameters(1.0, gating_threshold_variance, VOCALGORITHM_GATING_THRESHOLD_TRANSITION);

        let sigmoid_gating_variance = self.mean_variance_estimator_sigmoid_process(voc_index_from_prior);

        self.m_mean_variance_estimator_gamma_variance = sigmoid_gating_variance * gamma_variance;

        self.m_mean_variance_estimator_gating_duration_minutes = self.m_mean_variance_estimator_gating_duration_minutes +
            (VOCALGORITHM_SAMPLING_INTERVAL / 60.0) * 
            (((1.0 - sigmoid_gating_mean) * (1.0 + VOCALGORITHM_GATING_MAX_RATIO)) - VOCALGORITHM_GATING_MAX_RATIO);

        if self.m_mean_variance_estimator_gating_duration_minutes < 0.0 {
            self.m_mean_variance_estimator_gating_duration_minutes = 0.0;
        }
        if self.m_mean_variance_estimator_gating_duration_minutes > self.m_mean_variance_estimator_gating_max_duration_minutes {
            self.m_mean_variance_estimator_uptime_gating = 0.0;
        }
    }

    fn mean_variance_estimator_process(&mut self, sraw: f64, voc_index_from_prior: f64) {
        let mut sraw = sraw;
        if !self.m_mean_variance_estimator_initialized {
            self.m_mean_variance_estimator_initialized = true;
            self.m_mean_variance_estimator_sraw_offset = sraw;
            self.m_mean_variance_estimator_mean = 0.0;
        } else {
            if self.m_mean_variance_estimator_mean >= 100.0 ||
                 self.m_mean_variance_estimator_mean <= -100.0 {
                self.m_mean_variance_estimator_sraw_offset = self.m_mean_variance_estimator_sraw_offset + self.m_mean_variance_estimator_mean;
                self.m_mean_variance_estimator_mean = 0.0;
            }
            sraw = sraw - self.m_mean_variance_estimator_sraw_offset;

            self.mean_variance_estimator_calculate_gamma(voc_index_from_prior);

            let delta_sgp = (sraw - self.m_mean_variance_estimator_mean) / VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING;
            
            let c = if delta_sgp < 0.0 {
                self.m_mean_variance_estimator_std - delta_sgp
            } else {
                self.m_mean_variance_estimator_std + delta_sgp
            };

            let additional_scaling = if c > 1440.0 {
                4.0
            } else {
                1.0
            };

            let _mult_a1 = additional_scaling * (VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING - self.m_mean_variance_estimator_gamma_variance);
            let _sqrt_a = _mult_a1.sqrt();
            
            let _mult_b1 = self.m_mean_variance_estimator_std * (self.m_mean_variance_estimator_std / (VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR_GAMMA_SCALING * additional_scaling));
            let _mult_b2 = ((self.m_mean_variance_estimator_gamma_variance * delta_sgp) / additional_scaling) * delta_sgp;
            let _sqrt_b = (_mult_b1 + _mult_b2).sqrt();
            self.m_mean_variance_estimator_std = _sqrt_a * _sqrt_b;
            self.m_mean_variance_estimator_mean = self.m_mean_variance_estimator_mean + (self.m_mean_variance_estimator_gamma_mean * delta_sgp);                 
        }
    }
}

