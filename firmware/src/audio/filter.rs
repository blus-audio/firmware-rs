use biquad::*;
use micromath::F32Ext;

type B = DirectForm2Transposed<f32>;
type C = Coefficients<f32>;

const MAX_DELAY_LENGTH: usize = 32;
const MAX_BIQUAD_COUNT: usize = 10;

pub fn get_filters(fs: Hertz<f32>) -> (Filter, Filter, Filter, Filter) {
    let f_co = 1800.hz();
    let biquads_a = [
        B::new(C::from_params(Type::AllPass, fs, f_co, 0.6).unwrap()),
        B::new(C {
            a1: -1.9925941047116,
            a2: 0.992621419175639,
            b0: 1.00200843380849,
            b1: -1.99256829254308,
            b2: 0.990638797535668,
        }),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 660.hz(), 2.5).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 880.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::HighShelf(-8.0), fs, 1200.hz(), 0.35).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 1300.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-3.0), fs, 3450.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];
    let biquads_c = [
        B::new(C::from_params(Type::AllPass, fs, f_co, 0.6).unwrap()),
        B::new(C {
            a1: -1.9925941047116,
            a2: 0.992621419175639,
            b0: 1.00200843380849,
            b1: -1.99256829254308,
            b2: 0.990638797535668,
        }),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 660.hz(), 2.5).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 880.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::HighShelf(-8.0), fs, 1200.hz(), 0.35).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-2.5), fs, 1300.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-3.0), fs, 3450.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];
    let biquads_b = [
        B::new(C::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];
    let biquads_d = [
        B::new(C::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap()),
        B::new(C::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap()),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
        B::new(C {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
        }),
    ];

    // Negative gain inverts a channel.
    let gain_a = -10.0f32.powf(-10.0 / 20.0);
    let gain_b = 10.0f32.powf(-11.5 / 20.0);
    let gain_c = -10.0f32.powf(-10.0 / 20.0);
    let gain_d = 10.0f32.powf(-11.5 / 20.0);

    let delay_a: usize = 0;
    let delay_b: usize = 6;
    let delay_c: usize = 0;
    let delay_d: usize = 6;

    let f_a = Filter::new(gain_a, delay_a, biquads_a);
    let f_b = Filter::new(gain_b, delay_b, biquads_b);
    let f_c = Filter::new(gain_c, delay_c, biquads_c);
    let f_d = Filter::new(gain_d, delay_d, biquads_d);

    (f_a, f_b, f_c, f_d)
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

pub struct Filter {
    gain: f32,
    delay: Delay,
    biquads: [B; MAX_BIQUAD_COUNT],
}

impl Filter {
    /// Create a new filter instance.
    ///
    /// # Arguments
    ///
    /// * `gain` - A linear gain for the filter.
    /// * `delay_length` - A delay to apply, in number of samples.
    /// * `biquads` - The biquads to apply when running the filter.
    pub fn new(gain: f32, delay_length: usize, biquads: [B; MAX_BIQUAD_COUNT]) -> Self {
        Filter {
            gain,
            delay: Delay::new(delay_length),
            biquads,
        }
    }

    /// Run the filter on a provided sample.
    pub fn run(&mut self, mut sample: f32) -> f32 {
        for biquad in self.biquads.as_mut() {
            sample = biquad.run(sample);
        }
        sample = sample * self.gain;

        self.delay.tick(sample)
    }
}
