use crate::body::Body;
use crate::shape::Shape;
use crate::vector::{RotationMatrix, Vec2};

#[derive(Debug)]
pub struct AABB {
    min: Vec2,
    max: Vec2,
}

impl AABB {
    pub fn from_body(body: &Body) -> AABB {
        match &body.shape {
            Shape::Polygon { vertices, .. } => {
                let rot_matrix = RotationMatrix::new(body.rotation);
                let mut min = body.position + rot_matrix * vertices[0];
                let mut max = min;

                for &v in vertices.iter().skip(1) {
                    // Apply the rotation to each vertex
                    let r = body.position + rot_matrix * v;

                    if r.x < min.x {
                        min.x = r.x;
                    }
                    if r.y < min.y {
                        min.y = r.y;
                    }
                    if r.x > max.x {
                        max.x = r.x;
                    }
                    if r.y > max.y {
                        max.y = r.y;
                    }
                }

                AABB { min, max }
            }
            Shape::Circle { r } => AABB {
                min: body.position - *r,
                max: body.position + *r,
            },
        }
    }
}

pub fn sat(a: &AABB, b: &AABB) -> bool {
    // Exit with no intersection if found separated along an axis
    if a.max.x < b.min.x || a.min.x > b.max.x {
        return false;
    }

    if a.max.y < b.min.y || a.min.y > b.max.y {
        return false;
    }

    true
}
