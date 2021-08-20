use generational_arena::{Arena, Index};
use std::ops;

use crate::material::Material;
use crate::shape::Shape;
use crate::vector::{CrossProduct, Vec2};

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
    pub fn new(shape: Shape, position: Vec2, material: Material) -> Body {
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
        b: BodyHandle,
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
        self.bodies.iter_mut().map(|(h, b)| (BodyHandle(h), b))
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
