
use crate::body::{BodySet, BodyHandle, Body, Shape};
use crate::vector::{Vec2, RotationMatrix, CrossProduct};
use crate::math::{clamp, EPSILON, next_index};

#[derive(Debug)]
pub struct Manifold {
    a: BodyHandle,
    b: BodyHandle,
    normal: Vec2,
    penetration: f64,
    contacts: Vec<Vec2>,
}

#[derive(Debug)]
pub struct Pair {
    a: BodyHandle,
    b: BodyHandle,
}

#[derive(Debug)]
pub struct AABB {
    min: Vec2,
    max: Vec2,
}

impl Manifold {
    pub fn apply_impulse(&self, bodies: &mut BodySet, dt: f64, gravity: Vec2) {
        if let (Some(a), Some(b)) = bodies.get2_mut(self.a, self.b) {
            if a.inv_mass + b.inv_mass == 0.0 {
                return;
            }
            
            let sf = (a.static_friction * b.static_friction).sqrt();
            let df = (a.dynamic_friction * b.dynamic_friction).sqrt();

            let contact_count_f = self.contacts.len() as f64;

            for contact in &self.contacts {
                let ra = *contact - a.position;
                let rb = *contact - b.position;

                // Relative velocity
                let rv = b.velocity + b.angular_velocity.cross(rb) -
                         a.velocity - a.angular_velocity.cross(ra);

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
                
                let inv_mass_sum = a.inv_mass + b.inv_mass + 
                                   ra_n.powf(2.0) * a.inv_inertia + 
                                   rb_n.powf(2.0) * b.inv_inertia;

                // Calculate impulse scalar
                let j = -(1.0 + e) * cv / inv_mass_sum / contact_count_f;

                // Apply impulse
                let impulse = self.normal * j;

                a.apply_impulse(-impulse, ra);
                b.apply_impulse(impulse, rb);
                
                let new_rv = b.velocity + b.angular_velocity.cross(rb) -
                             a.velocity - a.angular_velocity.cross(ra);

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

    pub fn correct_positions(&self, bodies: &mut BodySet) {
        if let (Some(a), Some(b)) = bodies.get2_mut(self.a, self.b) {
            let percent: f64 = 0.4; // usually 20% to 80%
            let slop: f64 = 0.05; // usually 0.01 to 0.1
          
            if self.penetration > slop {
                let correction = (self.penetration - slop)  / (a.inv_mass + b.inv_mass) * percent * self.normal;
                a.position -= a.inv_mass * correction;
                b.position += b.inv_mass * correction;
            }
        }
    }
}

fn to_aabb(body: &Body) -> AABB {
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
        },
        Shape::Circle { r } => AABB { 
            min: body.position - *r, 
            max: body.position + *r 
        },
    }
}

fn sat(a: &AABB, b: &AABB) -> bool {
    // Exit with no intersection if found separated along an axis
    if a.max.x < b.min.x || a.min.x > b.max.x {
        return false;
    }

    if a.max.y < b.min.y || a.min.y > b.max.y {
        return false;
    }

    true
}

pub fn generate_pairs(bodies: &BodySet) -> Vec<Pair> {
    let mut pairs = vec![];

    for (i, (a_handle, a)) in bodies.iter().enumerate() {
        for (b_handle, b) in bodies.iter().skip(i+1) {
            let abox = to_aabb(&a);
            let bbox = to_aabb(&b);

            if sat(&abox, &bbox) {
                pairs.push(Pair { a: a_handle, b: b_handle});
            }
        }
    }

    pairs
}

pub fn solve_pair(bodies: &BodySet, pair: &Pair) -> Option<Manifold> {
    use Shape::*;
    let a = bodies.get(pair.a)?;
    let b = bodies.get(pair.b)?;

    match (&a.shape, &b.shape) {
        (Polygon { .. }, Polygon { .. }) => poly_vs_poly(bodies, pair.a, pair.b),
        (Polygon { .. }, Circle { .. }) => poly_vs_circle(bodies, pair.a, pair.b),
        (Circle { .. }, Polygon { .. }) => circle_vs_poly(bodies, pair.a, pair.b),
        (Circle { .. }, Circle { .. }) => circle_vs_circle(bodies, pair.b, pair.a),
    }
}

fn axis_of_least_penetration(a: &Body, b: &Body) -> Option<(usize, f64)> {
    let (a_vertices, a_normals, a_count) = match &a.shape {
        Shape::Polygon { vertices, normals, count } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };
    
    let (b_vertices, b_normals, b_count) = match &b.shape {
        Shape::Polygon { vertices, normals, count } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };

    if a_count == 0 {
        return None;
    }

    let mut best_distance = -f64::MAX;
    let mut best_index = 0;
     
    let a_rot = RotationMatrix::new(a.rotation);
    let b_rot = RotationMatrix::new(b.rotation);
    let b_rot_t = b_rot.transpose();

    for i in 0..a_count {
        // Face normal for a
        let a_n = a_normals[i];
        let nw = a_rot * a_n;

        // Transform into b's model space
        let n = b_rot_t * nw;

        // Get B extreme along normal
        let s = get_extreme_point(b_vertices, -n)?;
        
        // a vertex, transformed into b model space
        let v = b_rot_t * (a_rot * a_vertices[i] + a.position - b.position);

        // penetration
        let d = n.dot(s - v);
        
        if d > best_distance {
            best_distance = d;
            best_index = i;
        }
    }

    Some((best_index, best_distance))
}

fn get_extreme_point(vertices: &Vec<Vec2>, direction: Vec2) -> Option<Vec2> {
    if vertices.len() == 0 {
        return None;
    }

    let mut best_proj = -f64::MAX;
    let mut best_vertex = vertices[0];

    for vertex in vertices {
        let proj = vertex.dot(direction);

        if(proj > best_proj) {
            best_vertex = *vertex;
            best_proj = proj;
        }
    }

    Some(best_vertex)
}

fn find_incident_face(ref_b: &Body, inc_b: &Body, ref_index: usize) -> Option<(Vec2, Vec2)> {
    let (ref_vertices, ref_normals, ref_count) = match &ref_b.shape {
        Shape::Polygon { vertices, normals, count } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };
    
    let (inc_vertices, inc_normals, inc_count) = match &inc_b.shape {
        Shape::Polygon { vertices, normals, count } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };

    if inc_count == 0 {
        return None;
    }

    let ref_rot = RotationMatrix::new(ref_b.rotation);
    let inc_rot = RotationMatrix::new(inc_b.rotation);
    let inc_rot_t = inc_rot.transpose();

    let ref_normal = inc_rot_t * (ref_rot * ref_normals[ref_index]);
    
    let mut incident_face = 0;
    let mut min_dot = f64::MAX;

    for i in 0..inc_count {
        let dot = ref_normal.dot(inc_normals[i]);
        if dot < min_dot {
            min_dot = dot;
            incident_face = i;
        }
    }
    
    let incident_face_end = next_index(incident_face, inc_count);
    
    Some((
        inc_rot * inc_vertices[incident_face] + inc_b.position,
        inc_rot * inc_vertices[incident_face_end] + inc_b.position,
    ))
}

fn bias_gt(a: f64, b: f64) -> bool {
    let bias_rel = 0.95;
    let bias_abs = 0.01;

    return a >= b * bias_rel + a * bias_abs;
}

fn poly_vs_poly(
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
    
    let mut ref_b;
    let mut inc_b;
    let mut ref_index;
    let mut flip;
    
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

    let (ref_vertices, ref_normals, ref_count) = match &ref_b.shape {
        Shape::Polygon { vertices, normals, count } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };
    
    let (inc_vertices, inc_normals, inc_count) = match &inc_b.shape {
        Shape::Polygon { vertices, normals, count } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };

    let mut incident_face = find_incident_face(ref_b, inc_b, ref_index)?;

    let ref_rot = RotationMatrix::new(ref_b.rotation);
    
    // Face vertices in world space
    let v1 = ref_rot * ref_vertices[ref_index] + ref_b.position;
    let v2 = ref_rot * ref_vertices[next_index(ref_index, ref_count)] + 
             ref_b.position;

    // Reference face side normal
    let side_plane_normal = (v2 - v1).normalize();

    // Ortagonalize
    let ref_face_normal = Vec2::new(side_plane_normal.y, -side_plane_normal.x);

    let ref_c = ref_face_normal.dot(v1);
    let neg_side = -side_plane_normal.dot(v1);
    let pos_side = side_plane_normal.dot(v2);

    incident_face = clip(incident_face, -side_plane_normal, neg_side)?;
    incident_face = clip(incident_face, side_plane_normal, pos_side)?;

    let normal = if flip { -ref_face_normal } else { ref_face_normal };
    
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

fn clip(face: (Vec2, Vec2), n: Vec2, c: f64) -> Option<(Vec2, Vec2)> {
    let d1 = n.dot(face.0) - c;
    let d2 = n.dot(face.1) - c;
    
    if d1 * d2 < 0.0 {
        let alpha = d1 / (d1 - d2);
        let v_int = face.0 + alpha * (face.1 - face.0);
        if d1 < 0.0 {
            Some((face.0, v_int))
        } else {
            Some((face.1, v_int))
        }
    } else if d1 <= 0.0 && d2 <= 0.0 {
        Some(face)
    } else {
        None
    }
}

fn circle_vs_circle(
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

fn circle_vs_poly(
    bodies: &BodySet, 
    circle_handle: BodyHandle,
    poly_handle: BodyHandle, 
) -> Option<Manifold> {
    let poly = bodies.get(poly_handle)?;
    let circle = bodies.get(circle_handle)?;

    let (vertices, normals, count) = match &poly.shape {
        Shape::Polygon { vertices, normals, count } => (vertices, normals, *count),
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
    let i2 = if face_normal + 1 < count { face_normal + 1 } else { 0 };
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

fn poly_vs_circle(
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

//
//real FindAxisLeastPenetration( uint32 *faceIndex, PolygonShape *A, PolygonShape *B )
//{
//  real bestDistance = -FLT_MAX;
//  uint32 bestIndex;
//
//  for(uint32 i = 0; i < A->m_vertexCount; ++i)
//  {
//    // Retrieve a face normal from A
//    Vec2 n = A->m_normals[i];
//    Vec2 nw = A->u * n;
//
//    // Transform face normal into B's model space
//    Mat2 buT = B->u.Transpose( );
//    n = buT * nw;
//
//    // Retrieve support point from B along -n
//    Vec2 s = B->GetSupport( -n );
//
//    // Retrieve vertex on face from A, transform into
//    // B's model space
//    Vec2 v = A->m_vertices[i];
//    v = A->u * v + A->body->position;
//    v -= B->body->position;
//    v = buT * v;
//
//    // Compute penetration distance (in B's model space)
//    real d = Dot( n, s - v );
//
//    // Store greatest distance
//    if(d > bestDistance)
//    {
//      bestDistance = d;
//      bestIndex = i;
//    }
//  }
//
//  *faceIndex = bestIndex;
//  return bestDistance;
//}
//
//void FindIncidentFace( Vec2 *v, PolygonShape *RefPoly, PolygonShape *IncPoly, uint32 referenceIndex )
//{
//  Vec2 referenceNormal = RefPoly->m_normals[referenceIndex];
//
//  // Calculate normal in incident's frame of reference
//  referenceNormal = RefPoly->u * referenceNormal; // To world space
//  referenceNormal = IncPoly->u.Transpose( ) * referenceNormal; // To incident's model space
//
//  // Find most anti-normal face on incident polygon
//  int32 incidentFace = 0;
//  real minDot = FLT_MAX;
//  for(uint32 i = 0; i < IncPoly->m_vertexCount; ++i)
//  {
//    real dot = Dot( referenceNormal, IncPoly->m_normals[i] );
//    if(dot < minDot)
//    {
//      minDot = dot;
//      incidentFace = i;
//    }
//  }
//
//  // Assign face vertices for incidentFace
//  v[0] = IncPoly->u * IncPoly->m_vertices[incidentFace] + IncPoly->body->position;
//  incidentFace = incidentFace + 1 >= (int32)IncPoly->m_vertexCount ? 0 : incidentFace + 1;
//  v[1] = IncPoly->u * IncPoly->m_vertices[incidentFace] + IncPoly->body->position;
//}
//
//int32 Clip( Vec2 n, real c, Vec2 *face )
//{
//  uint32 sp = 0;
//  Vec2 out[2] = {
//    face[0],
//    face[1]
//  };
//
//  // Retrieve distances from each endpoint to the line
//  // d = ax + by - c
//  real d1 = Dot( n, face[0] ) - c;
//  real d2 = Dot( n, face[1] ) - c;
//
//  // If negative (behind plane) clip
//  if(d1 <= 0.0f) out[sp++] = face[0];
//  if(d2 <= 0.0f) out[sp++] = face[1];
//
//  // If the points are on different sides of the plane
//  if(d1 * d2 < 0.0f) // less than to ignore -0.0f
//  {
//    // Push interesection point
//    real alpha = d1 / (d1 - d2);
//    out[sp] = face[0] + alpha * (face[1] - face[0]);
//    ++sp;
//  }
//
//  // Assign our new converted values
//  face[0] = out[0];
//  face[1] = out[1];
//
//  assert( sp != 3 );
//
//  return sp;
//}
//
//void PolygontoPolygon( Manifold *m, Body *a, Body *b )
//{
//  PolygonShape *A = reinterpret_cast<PolygonShape *>(a->shape);
//  PolygonShape *B = reinterpret_cast<PolygonShape *>(b->shape);
//  m->contact_count = 0;
//
//  // Check for a separating axis with A's face planes
//  uint32 faceA;
//  real penetrationA = FindAxisLeastPenetration( &faceA, A, B );
//  if(penetrationA >= 0.0f)
//    return;
//
//  // Check for a separating axis with B's face planes
//  uint32 faceB;
//  real penetrationB = FindAxisLeastPenetration( &faceB, B, A );
//  if(penetrationB >= 0.0f)
//    return;
//
//  uint32 referenceIndex;
//  bool flip; // Always point from a to b
//
//  PolygonShape *RefPoly; // Reference
//  PolygonShape *IncPoly; // Incident
//
//  // Determine which shape contains reference face
//  if(BiasGreaterThan( penetrationA, penetrationB ))
//  {
//    RefPoly = A;
//    IncPoly = B;
//    referenceIndex = faceA;
//    flip = false;
//  }
//
//  else
//  {
//    RefPoly = B;
//    IncPoly = A;
//    referenceIndex = faceB;
//    flip = true;
//  }
//
//  // World space incident face
//  Vec2 incidentFace[2];
//  FindIncidentFace( incidentFace, RefPoly, IncPoly, referenceIndex );
//
//  //        y
//  //        ^  ->n       ^
//  //      +---c ------posPlane--
//  //  x < | i |\
//  //      +---+ c-----negPlane--
//  //             \       v
//  //              r
//  //
//  //  r : reference face
//  //  i : incident poly
//  //  c : clipped point
//  //  n : incident normal
//
//  // Setup reference face vertices
//  Vec2 v1 = RefPoly->m_vertices[referenceIndex];
//  referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
//  Vec2 v2 = RefPoly->m_vertices[referenceIndex];
//
//  // Transform vertices to world space
//  v1 = RefPoly->u * v1 + RefPoly->body->position;
//  v2 = RefPoly->u * v2 + RefPoly->body->position;
//
//  // Calculate reference face side normal in world space
//  Vec2 sidePlaneNormal = (v2 - v1);
//  sidePlaneNormal.Normalize( );
//
//  // Orthogonalize
//  Vec2 refFaceNormal( sidePlaneNormal.y, -sidePlaneNormal.x );
//
//  // ax + by = c
//  // c is distance from origin
//  real refC = Dot( refFaceNormal, v1 );
//  real negSide = -Dot( sidePlaneNormal, v1 );
//  real posSide =  Dot( sidePlaneNormal, v2 );
//
//  // Clip incident face to reference face side planes
//  if(Clip( -sidePlaneNormal, negSide, incidentFace ) < 2)
//    return; // Due to floating point error, possible to not have required points
//
//  if(Clip(  sidePlaneNormal, posSide, incidentFace ) < 2)
//    return; // Due to floating point error, possible to not have required points
//
//  // Flip
//  m->normal = flip ? -refFaceNormal : refFaceNormal;
//
//  // Keep points behind reference face
//  uint32 cp = 0; // clipped points behind reference face
//  real separation = Dot( refFaceNormal, incidentFace[0] ) - refC;
//  if(separation <= 0.0f)
//  {
//    m->contacts[cp] = incidentFace[0];
//    m->penetration = -separation;
//    ++cp;
//  }
//  else
//    m->penetration = 0;
//
//  separation = Dot( refFaceNormal, incidentFace[1] ) - refC;
//  if(separation <= 0.0f)
//  {
//    m->contacts[cp] = incidentFace[1];
//
//    m->penetration += -separation;
//    ++cp;
//
//    // Average penetration
//    m->penetration /= (real)cp;
//  }
//
//  m->contact_count = cp;
//}
