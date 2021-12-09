#[derive(Debug)]
pub struct Camera {
    pub position: Position,
    pub orientation: Orientation,
}
#[derive(Debug)]
pub struct Position {
    pub r: f64,
    pub theta: f64,
    pub phi: f64,
}
#[derive(Debug)]
pub struct Orientation {
    pub theta: f64,
    pub phi: f64,
    pub psi: f64,
}
#[derive(Debug)]
pub struct FoV {
    pub x: f64,
    pub y: f64,
}
