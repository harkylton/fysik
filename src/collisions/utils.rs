use crate::body::Body;
use crate::shape::Shape;
use crate::utils::next_index;
use crate::vector::{RotationMatrix, Vec2};

pub fn axis_of_least_penetration(a: &Body, b: &Body) -> Option<(usize, f64)> {
    let (a_vertices, a_normals, a_count) = match &a.shape {
        Shape::Polygon {
            vertices,
            normals,
            count,
        } => (vertices, normals, *count),
        _ => panic!("Invalid shape"),
    };

    let b_vertices = match &b.shape {
        Shape::Polygon { vertices, .. } => vertices,
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

pub fn find_incident_face(ref_b: &Body, inc_b: &Body, ref_index: usize) -> Option<(Vec2, Vec2)> {
    let ref_normals = match &ref_b.shape {
        Shape::Polygon { normals, .. } => normals,
        _ => panic!("Invalid shape"),
    };

    let (inc_vertices, inc_normals, inc_count) = match &inc_b.shape {
        Shape::Polygon {
            vertices,
            normals,
            count,
        } => (vertices, normals, *count),
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

pub fn bias_gt(a: f64, b: f64) -> bool {
    let bias_rel = 0.95;
    let bias_abs = 0.01;

    return a >= b * bias_rel + a * bias_abs;
}

pub fn clip(face: (Vec2, Vec2), n: Vec2, c: f64) -> Option<(Vec2, Vec2)> {
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

fn get_extreme_point(vertices: &Vec<Vec2>, direction: Vec2) -> Option<Vec2> {
    if vertices.len() == 0 {
        return None;
    }

    let mut best_proj = -f64::MAX;
    let mut best_vertex = vertices[0];

    for vertex in vertices {
        let proj = vertex.dot(direction);

        if proj > best_proj {
            best_vertex = *vertex;
            best_proj = proj;
        }
    }

    Some(best_vertex)
}
