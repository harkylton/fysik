use crate::utils::EPSILON;
use std::ops;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RotationMatrix(f64, f64, f64, f64);

impl RotationMatrix {
    pub fn transpose(&self) -> RotationMatrix {
        RotationMatrix(self.0, self.2, self.1, self.3)
    }

    pub fn new(radians: f64) -> RotationMatrix {
        let c = radians.cos();
        let s = radians.sin();

        RotationMatrix(c, -s, s, c)
    }
}

pub trait CrossProduct<Rhs = Self> {
    type Output;

    #[must_use]
    fn cross(self, other: Rhs) -> Self::Output;
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Vec2 {
        Vec2 { x, y }
    }

    pub fn origin() -> Vec2 {
        Vec2 { x: 0.0, y: 0.0 }
    }

    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn distance_squared(self, other: Self) -> f64 {
        let c = self - other;

        c.dot(c)
    }

    pub fn length_squared(self) -> f64 {
        self.x.powf(2.0) + self.y.powf(2.0)
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    pub fn as_slice(self) -> [f64; 2] {
        [self.x, self.y]
    }

    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > EPSILON {
            Vec2::new(self.x / len, self.y / len)
        } else {
            self
        }
    }
}

impl CrossProduct<Vec2> for Vec2 {
    type Output = f64;

    fn cross(self, other: Vec2) -> f64 {
        self.x * other.y - self.y * other.x
    }
}

impl CrossProduct<f64> for Vec2 {
    type Output = Vec2;

    fn cross(self, other: f64) -> Vec2 {
        Vec2::new(other * self.y, -other * self.x)
    }
}

impl CrossProduct<Vec2> for f64 {
    type Output = Vec2;

    fn cross(self, other: Vec2) -> Vec2 {
        Vec2::new(-self * other.y, self * other.x)
    }
}

impl ops::Neg for Vec2 {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl ops::Mul<Vec2> for RotationMatrix {
    type Output = Vec2;

    fn mul(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: self.0 * other.x + self.1 * other.y,
            y: self.2 * other.x + self.3 * other.y,
        }
    }
}

impl ops::Add<Vec2> for Vec2 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl ops::Add<f64> for Vec2 {
    type Output = Self;

    fn add(self, other: f64) -> Self {
        Self {
            x: self.x + other,
            y: self.y + other,
        }
    }
}

impl ops::Add<Vec2> for f64 {
    type Output = Vec2;

    fn add(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: other.x + self,
            y: other.y + self,
        }
    }
}

impl ops::Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, other: f64) -> Self {
        Self {
            x: self.x * other,
            y: self.y * other,
        }
    }
}

impl ops::Div<f64> for Vec2 {
    type Output = Self;

    fn div(self, other: f64) -> Self {
        Self {
            x: self.x / other,
            y: self.y / other,
        }
    }
}

impl ops::Mul<Vec2> for f64 {
    type Output = Vec2;

    fn mul(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: other.x * self,
            y: other.y * self,
        }
    }
}

impl ops::Sub<Vec2> for Vec2 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl ops::Sub<f64> for Vec2 {
    type Output = Self;

    fn sub(self, other: f64) -> Self {
        Self {
            x: self.x - other,
            y: self.y - other,
        }
    }
}

impl ops::SubAssign<Vec2> for Vec2 {
    fn sub_assign(&mut self, other: Self) {
        *self = Self {
            x: self.x - other.x,
            y: self.y - other.y,
        };
    }
}

impl ops::AddAssign<Vec2> for Vec2 {
    fn add_assign(&mut self, other: Self) {
        *self = Self {
            x: self.x + other.x,
            y: self.y + other.y,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::Vec2;
    #[test]
    fn test_dot() {
        let vec = Vec2 { x: 2.0, y: 4.0 };
        assert_eq!(vec.dot(vec), 20.0);
    }

    #[test]
    fn test_dot_negative() {
        let a = Vec2 { x: -2.0, y: 1.0 };
        let b = Vec2 { x: 1.0, y: -4.0 };
        assert_eq!(a.dot(b), -6.0);
    }
}
