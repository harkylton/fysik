#[derive(Debug)]
pub struct Material {
    pub density: f64,
    pub restitution: f64,
}

pub const ROCK: Material = Material {
    density: 0.6,
    restitution: 0.1,
};

pub const WOOD: Material = Material {
    density: 0.3,
    restitution: 0.2,
};

pub const METAL: Material = Material {
    density: 1.2,
    restitution: 0.05,
};

pub const BOUNCY_BALL: Material = Material {
    density: 0.3,
    restitution: 0.8,
};

pub const SUPER_BALL: Material = Material {
    density: 0.3,
    restitution: 0.95,
};

pub const PILLOW: Material = Material {
    density: 0.1,
    restitution: 0.2,
};

pub const STATIC: Material = Material {
    density: 0.0,
    restitution: 0.4,
};
