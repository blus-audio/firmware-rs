use biquad::*;
use micromath::F32Ext;

pub fn run_filters(mut sample: f32, filters: &mut [DirectForm2Transposed<f32>], gain: f32) -> f32 {
    for i in 0..filters.len() {
        sample = filters[i].run(sample)
    }

    sample * gain
}

pub struct Filters {
    pub biquads_a: [DirectForm2Transposed<f32>; 9],
    pub biquads_b: [DirectForm2Transposed<f32>; 6],
    pub biquads_c: [DirectForm2Transposed<f32>; 9],
    pub biquads_d: [DirectForm2Transposed<f32>; 6],

    pub gain_a: f32,
    pub gain_b: f32,
    pub gain_c: f32,
    pub gain_d: f32,
}

impl Filters {
    pub fn new(fs: Hertz<f32>) -> Self {
        // Crossover frequency
        let f_co = 1800.hz();

        let filters = Filters {
            biquads_a: [
                DirectForm2Transposed::<f32>::new(
                    Coefficients::<f32>::from_params(Type::AllPass, fs, f_co, 0.6).unwrap(),
                ),
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
            ],
            biquads_c: [
                DirectForm2Transposed::<f32>::new(
                    Coefficients::<f32>::from_params(Type::AllPass, fs, f_co, 0.6).unwrap(),
                ),
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
            ],
            biquads_b: [
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
            ],
            biquads_d: [
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
            ],

            gain_a: -10.0f32.powf(-10.0 / 20.0),
            gain_b: 10.0f32.powf(-11.5 / 20.0),
            gain_c: -10.0f32.powf(-10.0 / 20.0),
            gain_d: 10.0f32.powf(-11.5 / 20.0),
        };

        filters
    }
}
