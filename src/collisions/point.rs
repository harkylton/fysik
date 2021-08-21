use crate::body::Body;
use crate::shape::Shape;
use crate::utils::EPSILON;
use crate::vector::{RotationMatrix, Vec2};

pub fn point_in_body(point: Vec2, body: &Body) -> bool {
    match body.shape {
        Shape::Circle { .. } => point_in_circle(point, body),
        Shape::Polygon { .. } => point_in_polygon(point, body),
    }
}

fn point_in_circle(point: Vec2, circle: &Body) -> bool {
    let r = match circle.shape {
        Shape::Circle { r } => r,
        _ => panic!("Invalid shape"),
    };

    let normal = circle.position - point;

    // Use squared length to avoid sqrt
    let dist_sqr = normal.length_squared();
    dist_sqr < r * r
}

fn point_in_polygon(point: Vec2, poly: &Body) -> bool {
    let (vertices, normals, count) = match &poly.shape {
        Shape::Polygon {
            vertices,
            normals,
            count,
        } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };

    let rot_matrix = RotationMatrix::new(poly.rotation);
    let center = rot_matrix.transpose() * (point - poly.position);

    let mut separation = -f64::MAX;

    for i in 0..count {
        let s = normals[i].dot(center - vertices[i]);

        if s > 0.0 {
            return false;
        }

        if s > separation {
            separation = s;
        }
    }

    separation < EPSILON
}
