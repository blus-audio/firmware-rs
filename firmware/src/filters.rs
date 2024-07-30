use biquad::*;
use heapless::Vec;
use micromath::F32Ext;

const MAX_DELAY_LENGTH: usize = 32;
const MAX_BIQUAD_COUNT: usize = 10;

pub fn get_filters(fs: Hertz<f32>) -> (Filter, Filter, Filter, Filter) {
    let f_co = 1800.hz();
    let biquads_a = [
        DirectForm2Transposed::<f32>::new(Coefficients::<f32>::from_params(Type::AllPass, fs, f_co, 0.6).unwrap()),
        DirectForm2Transposed::<f32>::new(Coefficients {
            a1: -1.9925941047116,
            a2: 0.992621419175639,
            b0: 1.00200843380849,
            b1: -1.99256829254308,
            b2: 0.990638797535668,
        }),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-2.5), fs, 660.hz(), 2.5).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(1.0), fs, 880.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::HighShelf(-8.0), fs, 1200.hz(), 0.35).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-2.5), fs, 1300.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-3.0), fs, 3450.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
    ];
    let biquads_c = [
        DirectForm2Transposed::<f32>::new(Coefficients::<f32>::from_params(Type::AllPass, fs, f_co, 0.6).unwrap()),
        DirectForm2Transposed::<f32>::new(Coefficients {
            a1: -1.9925941047116,
            a2: 0.992621419175639,
            b0: 1.00200843380849,
            b1: -1.99256829254308,
            b2: 0.990638797535668,
        }),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-2.5), fs, 660.hz(), 2.5).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(1.0), fs, 880.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::HighShelf(-8.0), fs, 1200.hz(), 0.35).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-2.5), fs, 1300.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-3.0), fs, 3450.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::LowPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
    ];
    let biquads_b = [
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
    ];
    let biquads_d = [
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-9.0), fs, 1700.hz(), 0.3).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(1.0), fs, 7700.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(-1.0), fs, 12000.hz(), 2.0).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::PeakingEQ(6.0), fs, 18000.hz(), 0.6).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
        DirectForm2Transposed::<f32>::new(
            Coefficients::<f32>::from_params(Type::HighPass, fs, f_co, Q_BUTTERWORTH_F32).unwrap(),
        ),
    ];

    let gain_a = -10.0f32.powf(-10.0 / 20.0);
    let gain_b = 10.0f32.powf(-11.5 / 20.0);
    let gain_c = -10.0f32.powf(-10.0 / 20.0);
    let gain_d = 10.0f32.powf(-11.5 / 20.0);

    let delay_a: usize = 0;
    let delay_b: usize = 6;
    let delay_c: usize = 0;
    let delay_d: usize = 6;

    let f_a = Filter::new(gain_a, delay_a, &biquads_a);
    let f_b = Filter::new(gain_b, delay_b, &biquads_b);
    let f_c = Filter::new(gain_c, delay_c, &biquads_c);
    let f_d = Filter::new(gain_d, delay_d, &biquads_d);

    (f_a, f_b, f_c, f_d)
}

pub struct Filter {
    gain: f32,
    delay_line: Vec<f32, MAX_DELAY_LENGTH>,
    biquads: Vec<DirectForm2Transposed<f32>, MAX_BIQUAD_COUNT>,
}

impl Filter {
    /// Create a new filter instance.
    pub fn new(gain: f32, delay: usize, biquads: &[DirectForm2Transposed<f32>]) -> Self {
        let mut filter = Filter {
            gain,
            delay_line: Vec::new(),
            biquads: Vec::from_slice(biquads).unwrap(),
        };

        if delay > filter.delay_line.capacity() {
            panic!("Delay exceeds delay line capacity.");
        }

        filter.delay_line.fill(0.0f32);
        filter.delay_line.truncate(delay);

        filter
    }

    /// Run the filter on a sample.
    pub fn run(&mut self, mut sample: f32) -> f32 {
        for biquad in &mut self.biquads {
            sample = biquad.run(sample)
        }

        self.delay_line.push(sample * self.gain).unwrap();
        self.delay_line.pop().unwrap()
    }
}
