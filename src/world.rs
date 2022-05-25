use crate::body::{Body, BodyHandle, BodySet};
use crate::collisions::{Pair, point_in_body};
use crate::vector::Vec2;
use crate::constraint::RopeJoint;

pub struct World {
    pub bodies: BodySet,
    pub constraints: Vec<RopeJoint>,
    gravity: Vec2,
    impulse_iterations: usize,
    bounds: (Vec2, Vec2),
    time_factor: f64,
}

impl World {
    pub fn new() -> World {
        World {
            bodies: BodySet::new(),
            gravity: Vec2::new(0.0, 50.0),
            impulse_iterations: 10,
            bounds: (Vec2::new(-500.0, -500.0), Vec2::new(1000.0, 1000.0)),
            time_factor: 2.0,
            constraints: Vec::new(),
        }
    }

    pub fn add_constraint(&mut self, constraint: RopeJoint) {
        self.constraints.push(constraint);
    }

    fn prune_bodies(&mut self, min: Vec2, max: Vec2) {
        let mut to_remove = Vec::new();

        for (handle, body) in self.bodies.iter() {
            if body.position.x > max.x
                || body.position.x < min.x
                || body.position.y > max.y
                || body.position.y < min.y
            {
                to_remove.push(handle);
            }
        }

        for handle in to_remove {
            self.bodies.remove(handle);
        }
    }

    pub fn body_at_point(&self, point: Vec2) -> Option<BodyHandle> {
        for (handle, body) in self.bodies.iter() {
            if point_in_body(point, body) {
                return Some(handle);
            }
        }

        None
    }

    pub fn drag_body(&mut self, handle: BodyHandle, start: Vec2, end: Vec2) {
        let body = self.bodies.get_mut(handle).unwrap();

        let contact_point = start - body.position;

        let impulse = (end - start) / body.inv_mass;

        body.apply_impulse(impulse, contact_point);
    }

    pub fn remove_body(&mut self, handle: BodyHandle) -> Option<Body> {
        self.bodies.remove(handle)
    }

    pub fn add_body(&mut self, body: Body) -> BodyHandle {
        self.bodies.push(body)
    }

    pub fn update(&mut self, dt: f64) {
        self.step(dt * self.time_factor);
        self.prune_bodies(self.bounds.0, self.bounds.1);
    }

    pub fn step(&mut self, dt: f64) {
        let mut manifolds = vec![];
        let mut c_manifolds = vec![];

        // Generate collision pairs
        let pairs = Pair::candidates(&self.bodies);

        // Build manifolds
        for pair in pairs {
            if let Some(manifold) = pair.solve(&self.bodies) {
                manifolds.push(manifold);
            }
        }

        for constraint in &self.constraints {
            if let Some(manifold) = constraint.solve(&self.bodies) {
                c_manifolds.push(manifold);
            }
        }

        // Apply forces
        for (_, body) in self.bodies.iter_mut() {
            body.apply_forces(dt, self.gravity)
        }

        // Resolve collisions
        for _ in 0..self.impulse_iterations {
            for manifold in &manifolds {
                manifold.apply_impulse(&mut self.bodies, dt, self.gravity, true);
            }

            for manifold in &c_manifolds {
                manifold.apply_impulse(&mut self.bodies, dt, self.gravity, false);
            }
        }
        // Apply velocities
        for (_, body) in self.bodies.iter_mut() {
            body.apply_velocities(dt, self.gravity)
        }
        // Correct positions
        for manifold in &manifolds {
            manifold.correct_positions(&mut self.bodies);
        }
        for manifold in &c_manifolds {
            manifold.correct_positions(&mut self.bodies);
        }
        // Clear forces
        for (_, body) in self.bodies.iter_mut() {
            body.clear_forces()
        }
    }
}
