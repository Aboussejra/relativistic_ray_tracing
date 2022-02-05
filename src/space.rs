use crate::obstacle::Obstacle;
use ndarray::{Array1, Array3};

#[derive(Debug, Clone, PartialEq)]
pub struct Space {
    pub rs: f64,
    pub c: f64,
    pub christoffel: Array3<f64>,
    pub obstacles: Vec<Obstacle>,
}

impl Space {
    pub fn update_christoffel(&mut self, position: &Array1<f64>) {
        let r = position[1];
        let theta = position[2];

        let a = 1. / (1. - (self.rs / r));
        let ap = -self.rs / ((r - self.rs).powi(2));
        let b = self.c * self.c * (self.rs / r - 1.);
        let bp = -self.c * self.c * self.rs / (r * r);

        // i = 0 : TIME
        self.christoffel[[0, 0, 1]] = bp / (2. * b);
        self.christoffel[[0, 1, 0]] = bp / (2. * b);

        // i = 1 : R
        self.christoffel[[1, 0, 0]] = -bp / (2. * a);
        self.christoffel[[1, 1, 1]] = ap / (2. * a);
        self.christoffel[[1, 2, 2]] = -r / a;
        self.christoffel[[1, 3, 3]] = -r * ((theta.sin()).powi(2)) / a;

        // i = 2 : Theta
        self.christoffel[[2, 0, 0]] = 0.;
        self.christoffel[[2, 1, 2]] = 1. / r;
        self.christoffel[[2, 2, 1]] = 1. / r;
        self.christoffel[[2, 3, 3]] = -(theta.sin()) * (theta.cos());
        // i = 3 : Phi
        self.christoffel[[3, 1, 3]] = 1. / r;
        self.christoffel[[3, 3, 1]] = 1. / r;
        self.christoffel[[3, 2, 3]] = 1. / (theta.tan());
        self.christoffel[[3, 3, 2]] = 1. / (theta.tan());
    }
}
