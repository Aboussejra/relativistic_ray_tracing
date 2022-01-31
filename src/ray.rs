use ndarray::Array1;

use crate::{
    obstacle::{CollisionPoint, Obstacle},
    space::Space,
};

static C: f64 = 1.;

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
    pub fn new_i(
        step_size: f64,
        initial_position: &Array1<f64>,    // Size 4 (t, r, theta, phi)
        initial_orientation: &Array1<f64>, // Size 2 (theta, phi)
        initial_velocity: f64,
        space: &Space,
    ) -> Self {
        /*  Initializes a ray in given space and given integration step size with :
        *   - initial position,
        *   - initial movement direction (spherical coordinates in proper frame)
                (0,_) points outwards
                (pi,_) points inwards (towards black hole)
                (pi/2,0) is tangent, points towards the "north pole"
                (pi/2,pi/2) is tangent, follows the "latitudes"
        *   - initial velocity magnitude
        */

        let position = initial_position.clone();
        let mut position_derivative = Array1::<f64>::zeros(4);
        position_derivative[1] = initial_velocity // dr coordinate
            * step_size
            * C
            * (initial_orientation[0].cos())
            * ((1. - (space.rs / initial_position[1])).sqrt());
        position_derivative[2] = initial_velocity // dtheta coordinate
            * step_size
            * C
            * (initial_orientation[0].sin())
            * (initial_orientation[1].cos())
            / initial_position[1];
        position_derivative[3] = initial_velocity // dphi coordinate
            * step_size
            * C
            * (initial_orientation[0].sin())
            * (initial_orientation[1].sin())
            / (initial_position[1] * (initial_position[2].sin()));
        position_derivative[0] = (((step_size.powf(2.))
            - (((position_derivative[1].powf(2.) / (1. - (space.rs / initial_position[1])))
                + ((position_derivative[2] * initial_position[1]).powf(2.))
                + ((position_derivative[3]
                    * initial_position[1]
                    * (initial_position[2].sin()))
                .powf(2.)))
                / (C.powf(2.))))
            / (1. - (space.rs / initial_position[1])))
            .max(0.)
            .sqrt();
        Ray {
            position,
            position_derivative,
        }
    }

    fn next_step(&mut self, d_lambda: f64, space: &mut Space) {
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
    }

    pub fn trace(
        &mut self,
        space: &mut Space,
        number_steps: i32,
        d_lambda: f64,
        verbose: bool,
    ) -> Option<CollisionPoint> {
        // Performs the number of calls to next_step() specified in argument
        if verbose {
            println!("-----Trace : {}-----", number_steps);
        }
        let blackhole = Obstacle::BlackHole { r: space.rs };
        let _obstacle = Obstacle::Ring {
            r_min: 3. * space.rs,
            r_max: 5. * space.rs,
        };

        if verbose {
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
            self.next_step(d_lambda, space);
            if verbose {
                print!("Step {} out of {}", n + 1, number_steps);
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
                let true_velocity = (self.position_derivative[1].powf(2.)
                    + (self.position_derivative[2] * self.position[1]).powf(2.)
                    + (self.position_derivative[3] * self.position[1] * self.position[2].sin())
                        .powf(2.))
                .sqrt();
                println!("                     velocity = {v}", v = true_velocity);
            }
            let new_position = &self.position.clone();
            let has_collided = blackhole.collision(old_position, new_position);
            if has_collided {
                return Some(CollisionPoint {
                    collision_point: new_position.clone(),
                    color: blackhole.color(new_position),
                });
            }
        }
        None
    }
}

fn second_derivative(
    // Computes second derivative of movement at given position and velocity, in given space
    position: &Array1<f64>,
    position_derivative: &Array1<f64>,
    space: &mut Space,
) -> Array1<f64> {
    space.update_christoffel(position);
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
