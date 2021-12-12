#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct Ray {
    pub position: [f64; 4],
    pub position_derivative: [f64; 4], // order of coordinates are t,r,theta,phi
}

impl Ray {
    pub fn new() -> Self {
        Default::default()
    }
}
