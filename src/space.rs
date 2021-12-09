use crate::camera::Position;
use ndarray::{prelude, Array, Array3, Dim};
#[derive(Debug, Clone)]
pub struct Espace {
    pub rs: f64,
    pub christoffel: Array3<f64>,
}

impl Espace {
    fn update_christoffel(mut self, position: Position) {
        let r = position.r;

        let A = 1. / (1. - self.rs / r);
        //self.christoffel[0][0][0] = A;
    }
}
