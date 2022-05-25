use crate::body::{BodyHandle, BodySet};
use crate::collisions::utils::{axis_of_least_penetration, bias_gt, clip, find_incident_face};
use crate::shape::Shape;
use crate::utils::{next_index, EPSILON};
use crate::vector::{CrossProduct, RotationMatrix, Vec2};

#[derive(Debug)]
pub struct Manifold {
    a: BodyHandle,
    b: BodyHandle,
    normal: Vec2,
    penetration: f64,
    contacts: Vec<Vec2>,
}

impl Manifold {
    pub fn new(a: BodyHandle, b: BodyHandle, normal: Vec2, penetration: f64, contacts: Vec<Vec2>) -> Manifold {
        Manifold { a, b, normal, penetration, contacts }
    }

    pub fn apply_impulse(&self, bodies: &mut BodySet, dt: f64, gravity: Vec2, apply_friction: bool) {
        if let (Some(a), Some(b)) = bodies.get2_mut(self.a, self.b) {
            if a.inv_mass + b.inv_mass == 0.0 {
                return;
            }

            let contact_count_f = self.contacts.len() as f64;

            for contact in &self.contacts {
                let ra = *contact - a.position;
                let rb = *contact - b.position;

                // Relative velocity
                let rv = b.velocity + b.angular_velocity.cross(rb)
                    - a.velocity
                    - a.angular_velocity.cross(ra);

                let rva = rv.length_squared();
                let ga = (dt * gravity).length_squared();
                let e = if rva < ga + EPSILON {
                    0.0
                } else {
                    a.material.restitution.min(b.material.restitution)
                };

                // Contact velocity
                let cv = rv.dot(self.normal);

                // Do not resolve if velocities are separating
                if cv > 0.0 {
                    return;
                }

                let ra_n = ra.cross(self.normal);
                let rb_n = rb.cross(self.normal);

                let inv_mass_sum = a.inv_mass
                    + b.inv_mass
                    + ra_n.powf(2.0) * a.inv_inertia
                    + rb_n.powf(2.0) * b.inv_inertia;
                // Calculate impulse scalar
                let j = -(1.0 + e) * cv / inv_mass_sum / contact_count_f;

                // Apply impulse
                let impulse = self.normal * j;

                a.apply_impulse(-impulse, ra);
                b.apply_impulse(impulse, rb);
                
                if apply_friction {
                    let sf = (a.static_friction * b.static_friction).sqrt();
                    let df = (a.dynamic_friction * b.dynamic_friction).sqrt();

                    let new_rv = b.velocity + b.angular_velocity.cross(rb)
                        - a.velocity
                        - a.angular_velocity.cross(ra);

                    // Friction impulse
                    let t = (new_rv - self.normal * new_rv.dot(self.normal)).normalize();

                    let jt = -new_rv.dot(t) / inv_mass_sum / contact_count_f;

                    if jt == 0.0 {
                        return;
                    }

                    // Coulumb's law
                    let tangent_impulse = if jt.abs() < j * sf {
                        t * jt
                    } else {
                        t * -j * df
                    };

                    a.apply_impulse(-tangent_impulse, ra);
                    b.apply_impulse(tangent_impulse, rb);
                }
            }
        }
    }

    pub fn correct_positions(&self, bodies: &mut BodySet) {
        if let (Some(a), Some(b)) = bodies.get2_mut(self.a, self.b) {
            let percent: f64 = 0.4; // usually 20% to 80%
            let slop: f64 = 0.05; // usually 0.01 to 0.1

            if self.penetration > slop {
                let correction =
                    (self.penetration - slop) / (a.inv_mass + b.inv_mass) * percent * self.normal;
                a.position -= a.inv_mass * correction;
                b.position += b.inv_mass * correction;
            }
        }
    }
}

pub fn poly_vs_poly(
    bodies: &BodySet,
    a_handle: BodyHandle,
    b_handle: BodyHandle,
) -> Option<Manifold> {
    let a = bodies.get(a_handle)?;
    let b = bodies.get(b_handle)?;

    let (face_a, penetration_a) = axis_of_least_penetration(a, b)?;

    if penetration_a > 0.0 {
        return None;
    }

    let (face_b, penetration_b) = axis_of_least_penetration(b, a)?;

    if penetration_b > 0.0 {
        return None;
    }

    let ref_b;
    let inc_b;
    let ref_index;
    let flip;

    if bias_gt(penetration_a, penetration_b) {
        ref_b = a;
        inc_b = b;
        ref_index = face_a;
        flip = false;
    } else {
        ref_b = b;
        inc_b = a;
        ref_index = face_b;
        flip = true;
    }

    let (ref_vertices, ref_count) = match &ref_b.shape {
        Shape::Polygon {
            vertices, count, ..
        } => (vertices, *count),
        _ => panic!("Invalid shape"),
    };

    let mut incident_face = find_incident_face(ref_b, inc_b, ref_index)?;

    let ref_rot = RotationMatrix::new(ref_b.rotation);

    // Face vertices in world space
    let v1 = ref_rot * ref_vertices[ref_index] + ref_b.position;
    let v2 = ref_rot * ref_vertices[next_index(ref_index, ref_count)] + ref_b.position;

    // Reference face side normal
    let side_plane_normal = (v2 - v1).normalize();

    // Ortagonalize
    let ref_face_normal = Vec2::new(side_plane_normal.y, -side_plane_normal.x);

    let ref_c = ref_face_normal.dot(v1);
    let neg_side = -side_plane_normal.dot(v1);
    let pos_side = side_plane_normal.dot(v2);

    incident_face = clip(incident_face, -side_plane_normal, neg_side)?;
    incident_face = clip(incident_face, side_plane_normal, pos_side)?;

    let normal = if flip {
        -ref_face_normal
    } else {
        ref_face_normal
    };
    let separation_0 = ref_face_normal.dot(incident_face.0) - ref_c;

    let mut contacts = Vec::new();
    let mut penetration = 0.0;

    if separation_0 <= 0.0 {
        contacts.push(incident_face.0);
        penetration += -separation_0;
    }

    let separation_1 = ref_face_normal.dot(incident_face.1) - ref_c;

    if separation_1 < 0.0 {
        contacts.push(incident_face.1);
        penetration += -separation_1;
    }

    penetration = penetration / contacts.len() as f64;

    Some(Manifold {
        a: a_handle,
        b: b_handle,
        penetration,
        normal,
        contacts,
    })
}

pub fn circle_vs_circle(
    bodies: &BodySet,
    a_handle: BodyHandle,
    b_handle: BodyHandle,
) -> Option<Manifold> {
    let a = bodies.get(a_handle)?;
    let b = bodies.get(b_handle)?;

    let a_r = match a.shape {
        Shape::Circle { r } => r,
        _ => panic!("Invalid shape"),
    };

    let b_r = match b.shape {
        Shape::Circle { r } => r,
        _ => panic!("Invalid shape"),
    };

    let normal = b.position - a.position;

    let dist_sqr = normal.length_squared();
    let radius = a_r + b_r;

    if dist_sqr >= radius * radius {
        return None;
    }

    let distance = dist_sqr.sqrt();

    if distance == 0.0 {
        Some(Manifold {
            a: a_handle,
            b: b_handle,
            penetration: a_r,
            normal: Vec2::new(1.0, 0.0),
            contacts: vec![a.position],
        })
    } else {
        let n = normal / distance;

        Some(Manifold {
            a: a_handle,
            b: b_handle,
            penetration: radius - distance,
            normal: n,
            contacts: vec![n * a_r + a.position],
        })
    }
}

pub fn circle_vs_poly(
    bodies: &BodySet,
    circle_handle: BodyHandle,
    poly_handle: BodyHandle,
) -> Option<Manifold> {
    let poly = bodies.get(poly_handle)?;
    let circle = bodies.get(circle_handle)?;

    let (vertices, normals, count) = match &poly.shape {
        Shape::Polygon {
            vertices,
            normals,
            count,
        } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };

    let radius = match circle.shape {
        Shape::Circle { r } => r,
        _ => panic!("Invalid shape"),
    };

    let rot_matrix = RotationMatrix::new(poly.rotation);
    let center = rot_matrix.transpose() * (circle.position - poly.position);

    let mut separation = -f64::MAX;
    let mut face_normal = 0;

    for i in 0..count {
        let s = normals[i].dot(center - vertices[i]);
        if s > radius {
            return None;
        }

        if s > separation {
            separation = s;
            face_normal = i;
        }
    }

    // Check to see if center is within polygon
    if separation < EPSILON {
        let normal = rot_matrix * normals[face_normal];
        let m = Manifold {
            a: circle_handle,
            b: poly_handle,
            normal,
            contacts: vec![normal * radius + circle.position],
            penetration: (-separation + radius) * 2.0,
        };
        return Some(m);
    }

    // Determine which voronoi region of the edge center of circle lies within
    let v1 = vertices[face_normal];
    let i2 = next_index(face_normal, count);
    let v2 = vertices[i2];
    let dot1 = (center - v1).dot(v2 - v1);
    let dot2 = (center - v2).dot(v1 - v2);
    let penetration = radius - separation;

    if dot1 <= 0.0 {
        // Closest to v1
        if center.distance_squared(v1) > radius * radius {
            return None;
        }

        Some(vertex_manifold(
            circle_handle,
            poly_handle,
            penetration,
            v1,
            poly.position,
            center,
            rot_matrix,
        ))
    } else if dot2 < 0.0 {
        // Closest to v2
        if center.distance_squared(v2) > radius * radius {
            return None;
        }

        Some(vertex_manifold(
            circle_handle,
            poly_handle,
            penetration,
            v2,
            poly.position,
            center,
            rot_matrix,
        ))
    } else {
        // Closest to face
        let n = normals[face_normal];
        if (center - v1).dot(n) > radius {
            return None;
        }

        let normal = rot_matrix * -n;

        Some(Manifold {
            a: circle_handle,
            b: poly_handle,
            normal: normal,
            penetration,
            contacts: vec![normal * radius + circle.position],
        })
    }
}

pub fn poly_vs_circle(
    bodies: &BodySet,
    poly_handle: BodyHandle,
    circle_handle: BodyHandle,
) -> Option<Manifold> {
    circle_vs_poly(bodies, circle_handle, poly_handle)
}

fn vertex_manifold(
    circle_handle: BodyHandle,
    poly_handle: BodyHandle,
    penetration: f64,
    vertex: Vec2,
    position: Vec2,
    center: Vec2,
    rot_matrix: RotationMatrix,
) -> Manifold {
    Manifold {
        a: circle_handle,
        b: poly_handle,
        penetration,
        normal: (rot_matrix * (vertex - center)).normalize(),
        contacts: vec![rot_matrix * vertex + position],
    }
}
