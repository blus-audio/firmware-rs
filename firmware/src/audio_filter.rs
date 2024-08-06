use biquad::*;
use heapless::Vec;

const MAX_DELAY_LENGTH: usize = 32;
const MAX_BIQUAD_COUNT: usize = 10;

const Q31_SCALING_FACTOR: f32 = 2147483648.0;

fn clip_q63_to_q31(sample: i64) -> i32 {
    if (sample >> 32) as i32 != (sample as i32) >> 31 {
        0x7FFFFFFF ^ ((sample >> 63) as i32)
    } else {
        sample as i32
    }
}

/// Convert a float sample to its 1q31 representation. Clips to [-1, 1).
pub fn sample_to_u32(sample: f32) -> u32 {
    clip_q63_to_q31((sample * Q31_SCALING_FACTOR) as i64) as u32
}

/// Convert a 1q31 sample to float in the range [-1, 1)
pub fn sample_to_f32(sample: u32) -> f32 {
    (sample as i32 as f32) / Q31_SCALING_FACTOR
}

struct Delay {
    read_index: usize,
    write_index: usize,
    length: usize,
    delay_line: [f32; MAX_DELAY_LENGTH + 1],
}

impl Delay {
    /// Create a new delay instance with a given delay length in number of samples.
    fn new(length: usize) -> Self {
        if length > MAX_DELAY_LENGTH {
            panic!("Delay exceeds maximum allowed delay.");
        }
        Delay {
            write_index: length,
            read_index: 0,
            length,
            delay_line: [0.0f32; MAX_DELAY_LENGTH + 1],
        }
    }

    /// Increments read and write positions in a circular way.
    fn increment(&mut self) {
        for index in [&mut self.read_index, &mut self.write_index] {
            *index += 1;
            if *index >= self.length {
                *index = 0;
            }
        }
    }

    /// Consume a sample, and give back a delayed sample.
    fn tick(&mut self, sample: f32) -> f32 {
        self.increment();

        self.delay_line[self.write_index] = sample;
        self.delay_line[self.read_index]
    }
}

/// A chain of biquad filters with gain and delay.
pub struct Filter<B: Biquad<f32>> {
    /// The linear gain.
    gain: f32,
    /// A delay in number of samples.
    delay: Delay,
    /// The chain of biquad filters.
    /// FIXME: A vector is slower than an array, why is that?
    biquads: Vec<B, MAX_BIQUAD_COUNT>,
}

impl<B: Biquad<f32>> Filter<B> {
    /// Create a new filter instance.
    ///
    /// # Arguments
    ///
    /// * `gain` - A linear gain for the filter.
    /// * `delay_length` - A delay to apply, in number of samples.
    /// * `biquads` - The biquads to apply when running the filter.
    pub fn new(gain: f32, delay_length: usize, biquads: Vec<B, MAX_BIQUAD_COUNT>) -> Self {
        Filter {
            gain,
            delay: Delay::new(delay_length),
            biquads,
        }
    }

    /// Run the filter on a provided sample.
    pub fn run(&mut self, mut sample: f32) -> f32 {
        for b in &mut self.biquads {
            sample = b.run(sample);
        }
        sample *= self.gain;

        self.delay.tick(sample)
    }
}
