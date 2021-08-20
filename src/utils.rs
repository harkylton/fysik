use std::f64::consts::PI;

pub const EPSILON: f64 = 0.0001;

pub fn clamp<T: PartialOrd>(low: T, high: T, value: T) -> T {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}

pub fn radians(degrees: f64) -> f64 {
    degrees * PI / 180.0
}

pub fn next_index(index: usize, count: usize) -> usize {
    if index + 1 >= count {
        0
    } else {
        index + 1
    }
}
