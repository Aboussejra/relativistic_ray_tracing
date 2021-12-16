use core::num;

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

    fn next_step(&mut self, d_lambda: f64, space: &mut Space) {
        // Runge kutta 4
        let initial_position = &self.position;
        let initial_position_derivative = &self.position_derivative;

        let k1 = &second_derivative(&initial_position, &initial_position_derivative, space);
        let k2 = &second_derivative(
            &(initial_position + initial_position_derivative * d_lambda / 2.),
            &(initial_position_derivative + k1 * d_lambda / 2.),
            space,
        );
        let k3 = &second_derivative(
            &(initial_position
                + initial_position_derivative * d_lambda / 2.
                + k1 * d_lambda.powf(2.) / 4.),
            &(initial_position_derivative + k2 * d_lambda / 2.),
            space,
        );
        let k4 = second_derivative(
            &(initial_position
                + initial_position_derivative * d_lambda
                + k2 * d_lambda.powf(2.) / 2.),
            &(initial_position_derivative + k3 * d_lambda),
            space,
        );
        self.position = initial_position
            + d_lambda * initial_position_derivative
            + d_lambda.powf(2.) / 6. * (k1 + k2 + k3);
        self.position_derivative =
            initial_position_derivative + d_lambda / 6. * (k1 + 2. * k2 + 2. * k3 + k4);
    }

    pub fn trace(mut self, space: &mut Space, number_steps: i32, d_lambda: f64) {
        // Performs the number of calls to next_step() specified in argument
        println!("-----Trace : {}-----", number_steps);
        for n in 0..number_steps {
            self.next_step(d_lambda, space);
            print!("Step {} out of {}", n, number_steps);
            println!(" : t = {t}   r = {r}   theta = {th}   phi = {p}",
                t = self.position[0],
                r = self.position[1],
                th = self.position[2],
                p = self.position[3]
            );
        }
    }
}

fn second_derivative(
    position: &Array1<f64>,
    position_derivative: &Array1<f64>,
    space: &mut Space,
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
