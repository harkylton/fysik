use crate::body::{Body, BodyHandle, BodySet};
use crate::collisions::Pair;
use crate::vector::Vec2;

pub struct World {
    pub bodies: BodySet,
    gravity: Vec2,
    impulse_iterations: usize,
    bounds: (Vec2, Vec2),
}

impl World {
    pub fn new() -> World {
        World {
            bodies: BodySet::new(),
            gravity: Vec2::new(0.0, 50.0),
            impulse_iterations: 10,
            bounds: (Vec2::new(-500.0, -500.0), Vec2::new(1000.0, 1000.0)),
        }
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

    pub fn remove_body(&mut self, handle: BodyHandle) -> Option<Body> {
        self.bodies.remove(handle)
    }

    pub fn add_body(&mut self, body: Body) -> BodyHandle {
        self.bodies.push(body)
    }
    pub fn update(&mut self, dt: f64) {
        self.step(dt);
        self.prune_bodies(self.bounds.0, self.bounds.1);
    }

    pub fn step(&mut self, dt: f64) {
        let mut manifolds = vec![];

        // Generate collision pairs
        let pairs = Pair::candidates(&self.bodies);

        // Build manifolds
        for pair in pairs {
            if let Some(manifold) = pair.solve(&self.bodies) {
                manifolds.push(manifold);
            }
        }
        // Apply forces
        for (_, body) in self.bodies.iter_mut() {
            body.apply_forces(dt, self.gravity)
        }
        // Resolve collisions
        for _ in 0..self.impulse_iterations {
            for manifold in &manifolds {
                manifold.apply_impulse(&mut self.bodies, dt, self.gravity);
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
        // Clear forces
        for (_, body) in self.bodies.iter_mut() {
            body.clear_forces()
        }
    }
}
