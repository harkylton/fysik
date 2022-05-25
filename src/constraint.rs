use crate::body::{BodyHandle, BodySet};
use crate::collisions::Manifold;
use crate::vector::{Vec2, RotationMatrix};

#[derive(Debug)]
pub struct RopeJoint {
    pub a: BodyHandle,
    pub b: BodyHandle,
    pub anchor_a: Vec2,
    pub anchor_b: Vec2,
    pub max_distance: f64,
}

impl RopeJoint {
    pub fn get_contacts(&self, bodies: &BodySet) -> Option<(Vec2, Vec2)> {
        let a = bodies.get(self.a)?;
        let b = bodies.get(self.b)?;

        let rot_a = RotationMatrix::new(a.rotation);
        let rot_b = RotationMatrix::new(b.rotation);

        Some((rot_a * self.anchor_a + a.position, rot_b * self.anchor_b + b.position))
    }

    pub fn solve(&self, bodies: &BodySet) -> Option<Manifold> {
        let (a_contact, b_contact) = self.get_contacts(bodies)?;

        let n = a_contact - b_contact;
        
        let d = n.length();

        if d <= self.max_distance {
            return None
        }

        let penetration = (d - self.max_distance).abs();
        
        let normal = if d < self.max_distance {
            n / d * -1.0
        } else {
            n / d
        };
        
        Some(Manifold::new(
            self.a,
            self.b, 
            normal, 
            penetration, 
            vec![a_contact, b_contact],
        ))
    }
}


