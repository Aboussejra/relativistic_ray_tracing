#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct Camera {
    pub position: [f64; 3],    // r, theta, phi
    pub orientation: [f64; 3], // theta, phi, psi
}
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct FoV {
    pub x: f64,
    pub y: f64,
}
impl Camera {
    pub fn new() -> Self {
        Default::default()
    }
}
