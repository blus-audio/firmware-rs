pub mod filter;
pub mod routing;

const Q31_SCALING_FACTOR: f32 = 2147483648.0;

fn clip_q63_to_q31(sample: i64) -> i32 {
    if (sample >> 32) as i32 != (sample as i32) >> 31 {
        0x7FFFFFFF ^ ((sample >> 63) as i32)
    } else {
        sample as i32
    }
}

/// Convert a sample to its 1q31 representation. Clips inputs outside of [-1, 1).
pub fn sample_to_u32(sample: f32) -> u32 {
    clip_q63_to_q31((sample * Q31_SCALING_FACTOR) as i64) as u32
}

/// Convert a 1q31 sample to float in the range [-1, 1)
pub fn sample_to_f32(sample: u32) -> f32 {
    (sample as i32 as f32) / Q31_SCALING_FACTOR
}
