
use std::f64::consts::PI;
use crate::vector::{Vec2, CrossProduct};
use crate::material::Material;
use crate::math::{next_index, EPSILON};
use generational_arena::{Index,Arena};
use std::ops;
use rand::{thread_rng, Rng};

const MAX_VERTEX_COUNT: usize = 64;

#[derive(Debug)]
pub enum Shape {
    Circle { r: f64 },
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

        for i in 0..count {
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

        while true {
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
            Shape::Polygon { vertices, count, .. } => {
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
            Shape::Polygon { vertices, count, .. } => {
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

#[derive(Debug)]
pub struct Body {
    pub shape: Shape,
    pub material: Material,

    pub mass: f64,
    pub inv_mass: f64,
    pub inertia: f64,
    pub inv_inertia: f64,

    pub position: Vec2,
    pub velocity: Vec2,
    pub force: Vec2,

    pub rotation: f64,
    pub angular_velocity: f64,
    pub torque: f64,

    pub gravity_scale: f64,

    pub dynamic_friction: f64,
    pub static_friction: f64,
}

impl Body {
    pub fn new(
        shape: Shape, 
        position: Vec2, 
        material: Material,
    ) -> Body {
        let mass = material.density * shape.area();
        let inertia = shape.inertia(mass);

        Body {
            shape,
            material,
            mass,
            inertia,
            inv_mass: if mass == 0.0 { 0.0 } else { 1.0 / mass },
            inv_inertia: if inertia == 0.0 { 0.0 } else { 1.0 / inertia },
            position,
            velocity: Vec2::origin(),
            force: Vec2::origin(),
            rotation: 0.0,
            angular_velocity: 0.0,
            torque: 0.0,
            gravity_scale: 1.0,
            dynamic_friction: 0.2,
            static_friction: 0.4,
        }
    }
    
    pub fn apply_impulse(&mut self, impulse: Vec2, contact_vec: Vec2) {
        self.velocity += self.inv_mass * impulse;
        self.angular_velocity += self.inv_inertia * contact_vec.cross(impulse);
    }
    
    pub fn apply_forces(&mut self, dt: f64, gravity: Vec2) {
        if self.inv_mass == 0.0 {
            return;
        }

        self.velocity += (self.force * self.inv_mass + gravity) * (dt / 2.0);
        self.angular_velocity += self.torque * self.inv_inertia * (dt / 2.0);
    }

    pub fn apply_velocities(&mut self, dt: f64, gravity: Vec2) {
        if self.inv_mass == 0.0 {
            return;
        }

        self.position += self.velocity * dt;
        self.rotation += self.angular_velocity * dt;

        self.apply_forces(dt, gravity);
    }

    pub fn clear_forces(&mut self) {
        self.force = Vec2::origin();
        self.torque = 0.0;
    }

}

#[derive(Debug, Clone, Copy)]
pub struct BodyHandle(Index);

pub struct BodySet {
    bodies: Arena<Body>,
}

impl BodySet {
    pub fn new() -> BodySet {
        BodySet {
            bodies: Arena::new(),
        }
    }

    pub fn push(&mut self, body: Body) -> BodyHandle {
        BodyHandle(self.bodies.insert(body))
    }

    pub fn get(&self, handle: BodyHandle) -> Option<&Body> {
        self.bodies.get(handle.0)
    }

    pub fn get_mut(&mut self, handle: BodyHandle) -> Option<&mut Body> {
        let result = self.bodies.get_mut(handle.0)?;
        Some(result)
    }

    pub fn get2_mut(
        &mut self,
        a: BodyHandle,
        b: BodyHandle
    ) -> (Option<&mut Body>, Option<&mut Body>) {
        self.bodies.get2_mut(a.0, b.0)
    }

    pub fn remove(&mut self, handle: BodyHandle) -> Option<Body> {
        self.bodies.remove(handle.0)
    }

    pub fn iter(&self) -> impl Iterator<Item = (BodyHandle, &Body)> {
        self.bodies.iter().map(|(h, b)| (BodyHandle(h), b))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (BodyHandle, &mut Body)> {
        self.bodies.iter_mut().map(|(h, b)| {
            (BodyHandle(h), b)
        })
    }
}

impl ops::Index<BodyHandle> for BodySet {
    type Output = Body;

    fn index(&self, index: BodyHandle) -> &Body {
        &self.bodies[index.0]
    }
}

impl ops::Index<Index> for BodySet {
    type Output = Body;

    fn index(&self, index: Index) -> &Body {
        &self.bodies[index]
    }
}

impl ops::IndexMut<BodyHandle> for BodySet {
    fn index_mut(&mut self, handle: BodyHandle) -> &mut Body {
        let b = &mut self.bodies[handle.0];
        b
    }
}
