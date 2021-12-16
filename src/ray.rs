use ndarray::Array1;

use crate::space::Space;
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Ray {
    pub position: Array1<f64>,
    pub position_derivative: Array1<f64>, // order of coordinates are t,r,theta,phi
}

impl Ray {
    pub fn new() -> Self {
        Ray {
            position: Array1::<f64>::zeros(4),
            position_derivative: Array1::<f64>::zeros(4),
        }
    }

    fn next_step(&mut self, d_lambda: f64) {
        todo!();
    }

    fn trace(&mut self, number_steps: i32) {
        todo!();
    }
}

fn second_derivative(
    position: Array1<f64>,
    position_derivative: Array1<f64>,
    mut space: Space,
) -> Array1<f64> {
    space.update_christoffel(&position);
    let mut second_derivative = Array1::<f64>::zeros(4);
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                second_derivative[i] -=
                    space.christoffel[[i, j, k]] * position_derivative[j] * position_derivative[k];
            }
        }
    }
    second_derivative
}
