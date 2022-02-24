use ndarray::Array1;

use crate::{obstacle::CollisionPoint, space::Space};

static _C: f64 = 1.;

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Ray {
    pub position: Array1<f64>,
    pub position_derivative: Array1<f64>, // order of coordinates are t,r,theta,phi
}

impl Ray {
    pub fn new() -> Self {
        // Initializes ray with zero arrays
        Ray {
            position: Array1::<f64>::zeros(4),
            position_derivative: Array1::<f64>::zeros(4),
        }
    }
    ///Initializes a ray in given space and given integration step size with :
    /// - initial position,
    /// - initial movement direction (spherical coordinates in proper frame)
    ///        (0,_) points outwards
    ///        (pi,_) points inwards (towards black hole)
    ///        (pi/2,0) is tangent, points towards the "north pole"
    ///        (pi/2,pi/2) is tangent, follows the "latitudes"
    /// - initial velocity magnitude
    ///
    pub fn new_i(
        _step_size: f64,
        initial_position: &Array1<f64>,    // Size 4 (t, r, theta, phi)
        initial_orientation: &Array1<f64>, // Size 2 (theta, phi)
        _initial_velocity: f64,
        space: &Space,
    ) -> Self {
        /*
        if initial_velocity != 1.{
            println!("\nTime-like geodesics initialization not implemented yet. Initializing arrays to 0 instead...\n");
            return Ray::new();
        }*/

        let position = initial_position.clone();
        let mut position_derivative = Array1::<f64>::zeros(4);
        let metric = space.metric(&position);
        position_derivative[0] = 1. / ((-metric[0]).sqrt());
        position_derivative[1] = (initial_orientation[0].cos()) / (metric[1].sqrt());
        position_derivative[2] =
            (initial_orientation[0].sin()) * (initial_orientation[1].cos()) / (metric[2].sqrt());
        position_derivative[3] =
            (initial_orientation[0].sin()) * (initial_orientation[1].sin()) / (metric[3].sqrt());

        Ray {
            position,
            position_derivative,
        }
    }

    fn next_step(&mut self, d_lambda: f64, space: &Space) {
        // Runge kutta 4 integration method, computes one step
        let initial_position = &self.position;
        let initial_position_derivative = &self.position_derivative;

        let k1 = &second_derivative(initial_position, initial_position_derivative, space);
        let k2 = &second_derivative(
            &(initial_position + (initial_position_derivative * d_lambda / 2.)),
            &(initial_position_derivative + (k1 * d_lambda / 2.)),
            space,
        );
        let k3 = &second_derivative(
            &(initial_position
                + (initial_position_derivative * d_lambda / 2.)
                + (k1 * (d_lambda.powf(2.)) / 4.)),
            &(initial_position_derivative + (k2 * d_lambda / 2.)),
            space,
        );
        let k4 = second_derivative(
            &(initial_position
                + (initial_position_derivative * d_lambda)
                + (k2 * (d_lambda.powf(2.)) / 2.)),
            &(initial_position_derivative + (k3 * d_lambda)),
            space,
        );
        self.position = initial_position
            + (d_lambda * initial_position_derivative)
            + ((d_lambda.powf(2.)) / 6. * (k1 + k2 + k3));
        self.position_derivative =
            initial_position_derivative + (d_lambda / 6. * (k1 + (2. * k2) + (2. * k3) + k4));
        //println!("Step size {}", d_lambda);
    }

    pub fn trace(
        &mut self,
        space: &Space,
        number_steps: i32,
        step_size: f64,
        adaptive_step: bool,
        verbose: bool,
    ) -> Option<CollisionPoint> {
        // Performs the number of calls to next_step() specified in argument
        if verbose {
            println!("-----Trace : {}-----", number_steps);
            print!("Initial state : ");
            print!(
                " : t = {t}   r = {r}   theta = {th}   phi = {p}",
                t = self.position[0],
                r = self.position[1],
                th = self.position[2],
                p = self.position[3]
            );
            println!(
                "   dt = {dt}   dr = {dr}   dtheta = {dth}   dphi = {dp}",
                dt = self.position_derivative[0],
                dr = self.position_derivative[1],
                dth = self.position_derivative[2],
                dp = self.position_derivative[3]
            );
        }
        for n in 0..number_steps {
            let old_position = &self.position.clone();
            let mut d_lambda = step_size;
            if adaptive_step {
                d_lambda = (step_size * (1. - space.rs / self.position[1]).abs())
                    .max(space.rs * step_size / 200.);
                let pole_orth_velocity = ((self.position[1] * self.position_derivative[2]).powi(2)
                    + (self.position[1] * self.position_derivative[3] * (self.position[2]).sin())
                        .powi(2))
                .sqrt();

                let pole_distance = self.position[1] * self.position[2].sin();
                if pole_distance.abs() < step_size * pole_orth_velocity {
                    d_lambda = step_size.sqrt() * pole_distance / pole_orth_velocity / 10.;
                }
            }
            self.next_step(d_lambda, space);
            if verbose {
                print!("\n\n* Step {} out of {}", n + 1, number_steps);
                print!(
                    " : t = {t}   r = {r}   theta = {th}   phi = {p}",
                    t = self.position[0],
                    r = self.position[1],
                    th = self.position[2],
                    p = self.position[3]
                );
                println!(
                    "   dt = {dt}   dr = {dr}   dtheta = {dth}   dphi = {dp}",
                    dt = self.position_derivative[0],
                    dr = self.position_derivative[1],
                    dth = self.position_derivative[2],
                    dp = self.position_derivative[3]
                );
                println!("  -  Local step size : {}", d_lambda);
                let distance = ((self.position[1] - old_position[1]).powi(2)
                    / (1. - space.rs / self.position[1])
                    + (self.position[2] - old_position[2]).powi(2) * (self.position[1].powi(2))
                    + (self.position[3] - old_position[3]).powi(2)
                        * ((self.position[1] * (self.position[2].sin())).powi(2)))
                .sqrt();
                println!("  -  Reference step size : {}", distance);
                let momentum_conservation =
                    - self.position_derivative[0].powi(2) * (1. - space.rs / self.position[1]) / space.c.powi(2)
                    + self.position_derivative[1].powi(2) / (1. - space.rs / self.position[1])
                    + (self.position_derivative[2] * self.position[1]).powi(2)
                    + (self.position_derivative[3] * self.position[1] * self.position[2].sin())
                        .powi(2);
                println!("  -  Conservation of momentum = {}", momentum_conservation);
            }
            let new_position = &self.position.clone();
            if f64::is_nan(self.position[1]) {
                return None;
            }
            for obs in &space.obstacles {
                let interpolation = obs.collision(old_position, new_position, d_lambda);
                if interpolation >= 0. {
                    let collision_position =
                        new_position * interpolation + old_position * (1. - interpolation);
                    return Some(CollisionPoint {
                        collision_point: collision_position.clone(),
                        color: obs.color(&collision_position),
                    });
                }
            }
        }
        None
    }
}

fn second_derivative(
    // Computes second derivative of movement at given position and velocity, in given space
    position: &Array1<f64>,
    position_derivative: &Array1<f64>,
    space: &Space,
) -> Array1<f64> {
    let updated_space = space.update_christoffel(position);
    let mut second_derivative = Array1::<f64>::zeros(4);
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                second_derivative[i] -= updated_space.christoffel[[i, j, k]]
                    * position_derivative[j]
                    * position_derivative[k];
            }
        }
    }
    second_derivative
}
