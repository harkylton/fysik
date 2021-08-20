use crate::body::{BodyHandle, BodySet};
use crate::collisions::aabb::{sat, AABB};
use crate::collisions::manifold::{
    circle_vs_circle, circle_vs_poly, poly_vs_circle, poly_vs_poly, Manifold,
};
use crate::shape::Shape;

#[derive(Debug)]
pub struct Pair {
    a: BodyHandle,
    b: BodyHandle,
}

impl Pair {
    pub fn candidates(bodies: &BodySet) -> Vec<Pair> {
        let mut pairs = vec![];

        for (i, (a_handle, a)) in bodies.iter().enumerate() {
            for (b_handle, b) in bodies.iter().skip(i + 1) {
                let abox = AABB::from_body(&a);
                let bbox = AABB::from_body(&b);

                if sat(&abox, &bbox) {
                    pairs.push(Pair {
                        a: a_handle,
                        b: b_handle,
                    });
                }
            }
        }

        pairs
    }

    pub fn solve(&self, bodies: &BodySet) -> Option<Manifold> {
        use Shape::*;
        let a = bodies.get(self.a)?;
        let b = bodies.get(self.b)?;

        match (&a.shape, &b.shape) {
            (Polygon { .. }, Polygon { .. }) => poly_vs_poly(bodies, self.a, self.b),
            (Polygon { .. }, Circle { .. }) => poly_vs_circle(bodies, self.a, self.b),
            (Circle { .. }, Polygon { .. }) => circle_vs_poly(bodies, self.a, self.b),
            (Circle { .. }, Circle { .. }) => circle_vs_circle(bodies, self.b, self.a),
        }
    }
}
