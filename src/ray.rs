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

    fn second_derivative(&self, mut space: Space) -> [f64; 4] {
        space.update_christoffel(&self.position);
        let mut second_derivative = [0.; 4];
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    second_derivative[i] -= space.christoffel[[i, j, k]]
                        * self.position_derivative[j]
                        * self.position_derivative[k];
                }
            }
        }
        second_derivative
    }

    fn next_step(&mut self, d_lambda: f64) {
        todo!();
    }

    fn trace(&mut self, number_steps: i32) {
        todo!();
    }
}
