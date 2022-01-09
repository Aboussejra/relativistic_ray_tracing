use ndarray::Array1;
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Camera {
    pub position: Array1<f64>,    // r, theta, phi
    pub orientation: Array1<f64>, // theta, phi, psi
}
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct FoV {
    pub x: f64,
    pub y: f64,
}
impl Camera {
    pub fn new() -> Self {
        Camera {
            position: Array1::<f64>::zeros(3),
            orientation: Array1::<f64>::zeros(3),
        }
    }

    pub fn render(&self) {}
}
