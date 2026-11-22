//! Matrix factory ANGLE profile: 60 degrees, Actual rates 70/670 deg/s,
//! zero expo/deadband, mincheck 1050 and linear throttle. Angles use body FLU.

const MAX_TILT: f32 = 60.;
const MAX_RATE: f32 = 670.;
const CENTER: f32 = 70.;
const OUTER: f32 = MAX_RATE - CENTER;

fn pulse(rate: f32) -> u16 {
    let rate = rate.clamp(-MAX_RATE, MAX_RATE);
    let x = 2. * rate / (CENTER + (CENTER * CENTER + 4. * OUTER * rate.abs()).sqrt());
    (1500. + 500. * x).round() as u16
}

fn rate(pulse: u16) -> f32 {
    let x = (f32::from(pulse) - 1500.) / 500.;
    x * (CENTER + OUTER * x.abs())
}

#[derive(Clone, Copy, Debug)]
pub struct AngleDemand {
    pub roll: f32,
    pub pitch: f32,
    pub yaw_rate: f32,
    pub thrust: f32,
}

impl AngleDemand {
    #[must_use]
    pub fn channels(self) -> Option<[u16; 4]> {
        [self.roll, self.pitch, self.yaw_rate, self.thrust]
            .iter()
            .all(|v| v.is_finite())
            .then(|| {
                [
                    pulse(self.roll.to_degrees() * MAX_RATE / MAX_TILT),
                    pulse(self.pitch.to_degrees() * MAX_RATE / MAX_TILT),
                    pulse(-self.yaw_rate.to_degrees()),
                    (1050. + 950. * self.thrust.clamp(0., 1.)).round() as u16,
                ]
            })
    }

    #[must_use]
    pub fn from_channels([roll, pitch, yaw, throttle]: [u16; 4]) -> Self {
        Self {
            roll: (rate(roll) * MAX_TILT / MAX_RATE).to_radians(),
            pitch: (rate(pitch) * MAX_TILT / MAX_RATE).to_radians(),
            yaw_rate: -rate(yaw).to_radians(),
            thrust: ((f32::from(throttle) - 1050.) / 950.).clamp(0., 1.),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips_factory_angle_profile() {
        for v in 1000..=2000 {
            let channels = [v, 3000 - v, v, v.max(1050)];
            assert_eq!(
                AngleDemand::from_channels(channels).channels(),
                Some(channels)
            );
        }
        let a = AngleDemand::from_channels([2000, 1500, 2000, 1000]);
        assert!((a.roll.to_degrees() - 60.).abs() < 1e-5 && a.yaw_rate < 0. && a.thrust == 0.);
    }
}
