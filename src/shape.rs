use rand::{thread_rng, Rng};
use std::f64::consts::PI;

use crate::utils::{next_index, EPSILON};
use crate::vector::{CrossProduct, Vec2};

#[derive(Debug)]
pub enum Shape {
    Circle {
        r: f64,
    },
    Polygon {
        vertices: Vec<Vec2>,
        normals: Vec<Vec2>,
        count: usize,
    },
}

impl Shape {
    pub fn circle(r: f64) -> Shape {
        Shape::Circle { r }
    }

    pub fn rect(w: f64, h: f64) -> Shape {
        let hw = w / 2.0;
        let hh = h / 2.0;

        Shape::Polygon {
            vertices: vec![
                Vec2::new(-hw, -hh),
                Vec2::new(hw, -hh),
                Vec2::new(hw, hh),
                Vec2::new(-hw, hh),
            ],
            normals: vec![
                Vec2::new(0.0, -1.0),
                Vec2::new(1.0, 0.0),
                Vec2::new(0.0, 1.0),
                Vec2::new(-1.0, 0.0),
            ],
            count: 4,
        }
    }

    pub fn random_polygon() -> Shape {
        let count: usize = thread_rng().gen_range(3..64);
        let mut vertices = Vec::new();
        let e: f64 = thread_rng().gen_range(25.0..40.0);

        for _ in 0..count {
            let x = thread_rng().gen_range(-e..e);
            let y = thread_rng().gen_range(-e..e);
            vertices.push(Vec2::new(x, y));
        }

        if let Ok(polygon) = Shape::polygon(vertices) {
            polygon
        } else {
            Shape::random_polygon()
        }
    }

    pub fn polygon(vertices: Vec<Vec2>) -> Result<Shape, String> {
        let count = vertices.len();

        if count < 3 {
            return Err(String::from("Need at least 3 vertices to create polygon"));
        }

        let mut rightmost_idx = 0;
        let mut x_max = vertices[0].x;

        for i in 0..count {
            let x = vertices[i].x;

            if x > x_max {
                x_max = x;
                rightmost_idx = i;
            } else if x == x_max {
                if vertices[i].y < vertices[rightmost_idx].y {
                    rightmost_idx = i;
                }
            }
        }

        let mut hull = vec![0; count];
        let mut out_count = 0;
        let mut hull_idx = rightmost_idx;

        loop {
            hull[out_count] = hull_idx;

            let mut next_idx = 0;

            for i in 1..count {
                if next_idx == hull_idx {
                    next_idx = i;
                }

                let e1 = vertices[next_idx] - vertices[hull[out_count]];
                let e2 = vertices[i] - vertices[hull[out_count]];
                let c = e1.cross(e2);

                if c < 0.0 || c == 0.0 && e2.length_squared() > e1.length_squared() {
                    next_idx = i;
                }
            }

            out_count += 1;
            hull_idx = next_idx;

            if next_idx == rightmost_idx {
                break;
            }
        }

        let mut p_vertices = Vec::new();

        for i in 0..out_count {
            p_vertices.push(vertices[hull[i]]);
        }

        let mut p_normals = Vec::new();

        for i in 0..out_count {
            let j = next_index(i, out_count);

            let face = p_vertices[j] - p_vertices[i];

            if face.length_squared() < EPSILON * EPSILON {
                return Err(String::from("Zero length face in polygon.."));
            }

            p_normals.push(Vec2::new(face.y, -face.x).normalize());
        }

        Ok(Shape::Polygon {
            vertices: p_vertices,
            normals: p_normals,
            count: out_count,
        })
    }

    pub fn is_polygon(&self) -> bool {
        match self {
            Shape::Polygon { .. } => true,
            _ => false,
        }
    }
    pub fn is_circle(&self) -> bool {
        match self {
            Shape::Circle { .. } => true,
            _ => false,
        }
    }

    pub fn area(&self) -> f64 {
        match self {
            Shape::Circle { r } => r.powf(2.0) * PI,
            Shape::Polygon {
                vertices, count, ..
            } => {
                let mut area = 0.0;

                for i in 0..*count {
                    let p1 = vertices[i];
                    let j = if i + 1 < *count { i + 1 } else { 0 };
                    let p2 = vertices[j];

                    let d = p1.cross(p2);
                    let triangle_area = 0.5 * d;

                    area += triangle_area;
                }

                area
            }
        }
    }

    pub fn inertia(&self, mass: f64) -> f64 {
        match self {
            Shape::Circle { r } => mass * r.powf(2.0) / 2.0,
            Shape::Polygon {
                vertices, count, ..
            } => {
                if mass == 0.0 {
                    return 0.0;
                }

                let mut inertia = 0.0;
                let inv3 = 1.0 / 3.0;

                for i in 0..*count {
                    let p1 = vertices[i];
                    let j = if i + 1 < *count { i + 1 } else { 0 };
                    let p2 = vertices[j];

                    let d = p1.cross(p2);

                    let x2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
                    let y2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
                    inertia += (0.25 * inv3 * d) * (x2 + y2);
                }

                inertia
            }
        }
    }
}
