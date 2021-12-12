use crate::space::Space;
#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct Ray {
    pub position: [f64; 4],
    pub position_derivative: [f64; 4], // order of coordinates are t,r,theta,phi
}

impl Ray {
    pub fn new() -> Self {
        Default::default()
    }

    fn next_step(&mut self, d_lambda: f64) {
        todo!();
    }

    fn trace(&mut self, number_steps: i32) {
        todo!();
    }
}

fn second_derivative(
    position: &[f64; 4],
    position_derivative: &[f64; 4],
    mut space: Space,
) -> [f64; 4] {
    space.update_christoffel(position);
    let mut second_derivative = [0.; 4];
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
