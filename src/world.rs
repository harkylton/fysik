
extern crate generational_arena;
use generational_arena::Arena;

use crate::body::{Body, BodyHandle, BodySet};
use crate::vector::Vec2;
use crate::collisions::{generate_pairs, solve_pair};
use crate::math::clamp;
use std::time::{Instant};

pub struct World {
    pub bodies: BodySet,
    gravity: Vec2,
    dt: f64,
    last_update: Instant,
    impulse_iterations: usize,
}

impl World {
    pub fn new() -> World {
        World {
            bodies: BodySet::new(),
            gravity: Vec2::new(0.0, 50.0),
            dt: 1.0 / 60.0,
            last_update: Instant::now(),
            impulse_iterations: 10,
        }
    }

    fn prune_bodies(&mut self, min: Vec2, max: Vec2) {
        let mut to_remove = Vec::new();

        for (handle, body) in self.bodies.iter() {
            if body.position.x > max.x || 
               body.position.x < min.x ||
               body.position.y > max.y ||
               body.position.y < min.y {
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
    
    pub fn update(&mut self) {
        //let elapsed = self.last_update.elapsed().as_secs_f64();
        //println!("{:?}", elapsed);
        //let mut accumulator = clamp(0.0, 0.1, elapsed);

        //self.last_update = Instant::now();
        //
        //while accumulator >= self.dt {
        //    self.step();
        //    accumulator -= self.dt;
        //}
        self.step();
        self.prune_bodies(Vec2::new(0.0, 0.0), Vec2::new(1000.0, 1000.0));
    }

    pub fn step(&mut self) {
        let mut manifolds = vec![];

        // Generate collision pairs
        let pairs = generate_pairs(&self.bodies);

        // Build manifolds
        for pair in pairs {
            if let Some(manifold) = solve_pair(&self.bodies, &pair) {
                manifolds.push(manifold);
            }
        }
        
        // Apply forces
        for (handle, body) in self.bodies.iter_mut() {
            body.apply_forces(self.dt, self.gravity)
        }
        
        // Resolve collisions
        for _ in (0..self.impulse_iterations) {
            for manifold in &manifolds {
                manifold.apply_impulse(&mut self.bodies, self.dt, self.gravity);
            }
        }
        
        // Apply velocities
        for (handle, body) in self.bodies.iter_mut() {
            body.apply_velocities(self.dt, self.gravity)
        }
        
        // Correct positions
        for manifold in &manifolds {
            manifold.correct_positions(&mut self.bodies);
        }
        
        // Clear forces
        for (handle, body) in self.bodies.iter_mut() {
            body.clear_forces()
        }
    }
}
