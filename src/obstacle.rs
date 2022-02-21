use image::Rgb;
//use std::rand::{task_rng, Rng};
use ndarray::Array1;
use noise::{HybridMulti, MultiFractal, NoiseFn, Seedable};
use rand::prelude::*;
use std::f64::consts::PI;

#[derive(Debug, Clone, PartialEq)]
pub enum Obstacle {
    BlackHole {
        r: f64,
    }, // Ray cannot enter black hole
    BlackHolePredict {
        r: f64,
    }, // Ray cannot point towards black hole
    MaxDistance {
        r: f64,
    }, // Ray cannot escape max distance from origin
    Ring {
        r_min: f64,
        r_max: f64,
    }, // 2D ring placed on the equator plane in spherical coordinates, with inner and outer radii
    AccretionDisk {
        r_min: f64,
        r_max: f64,
        thickness: f64,
    }, // Same as Ring, but semi-transparent and with thickness.
}
#[derive(Debug, Clone, PartialEq)]
pub struct CollisionPoint {
    pub collision_point: Array1<f64>,
    pub color: Rgb<f64>,
}
impl Obstacle {
    pub fn collision(
        &self,
        ray_pos_t: &Array1<f64>,
        ray_pos_t_plus_dt: &Array1<f64>,
        step_size: f64,
    ) -> f64 {
        match self {
            Obstacle::BlackHole { r } => {
                if ray_pos_t_plus_dt[1] <= *r {
                    0.
                } else {
                    -1.
                }
            }
            Obstacle::MaxDistance { r } => {
                if ray_pos_t_plus_dt[1] >= *r {
                    0.
                } else {
                    -1.
                }
            }
            Obstacle::BlackHolePredict { r } => {
                if ray_pos_t_plus_dt[1] >= r * 3. / 2. {
                    // r*2 distance choosen arbitrarly (photon sphere radius = r*3/2)
                    //println!("ignore");
                    return -1.;
                }
                // Convert positions to cartesian frame
                let xyz_1 = [
                    (ray_pos_t[3].cos()) * (ray_pos_t[2].sin()) * ray_pos_t[1],
                    (ray_pos_t[2].sin()) * (ray_pos_t[3].sin()) * ray_pos_t[1],
                    (ray_pos_t[2].cos()) * ray_pos_t[1],
                ];
                let xyz_2 = [
                    (ray_pos_t_plus_dt[3].cos())
                        * (ray_pos_t_plus_dt[2].sin())
                        * ray_pos_t_plus_dt[1],
                    (ray_pos_t_plus_dt[2].sin())
                        * (ray_pos_t_plus_dt[3].sin())
                        * ray_pos_t_plus_dt[1],
                    (ray_pos_t_plus_dt[2].cos()) * ray_pos_t_plus_dt[1],
                ];
                let scalar_prod: f64 = xyz_1.iter().zip(xyz_2.iter()).map(|(x, y)| x * y).sum();
                let norm1: f64 = (xyz_1.iter().map(|x| x.powi(2)).sum::<f64>()).sqrt();
                let norm2: f64 = (xyz_2.iter().map(|x| x.powi(2)).sum::<f64>()).sqrt();
                // Angle between position vectors
                let angle3d = (scalar_prod / (norm1 * norm2)).acos();

                let t11 = -(r / ray_pos_t[1]).acos();
                let t12 = (r / ray_pos_t[1]).acos();
                let t21 = angle3d - ((r / ray_pos_t_plus_dt[1]).acos());
                let t22 = angle3d + ((r / ray_pos_t_plus_dt[1]).acos());

                // If the conditions are met the ray is pointing towards the black hole: we delete it prematurely
                if t11 < t21 && t12 > t22 {
                    0.
                } else {
                    -1.
                }
            }
            Obstacle::Ring { r_min, r_max } => {
                // Find intersection between path and ring: p_intersect = a * ray_pos2 + (1-a) * ray_pos1
                let a = (PI / 2. - (ray_pos_t[2] % PI).abs())
                    / ((ray_pos_t_plus_dt[2] % PI).abs() - (ray_pos_t[2] % PI).abs()); // We search for an intersection in the equator plane defined by (theta=PI/2)
                let r_intersect = ray_pos_t[1] * (1. - a) + ray_pos_t_plus_dt[1] * a; // Radial position of intersection point
                if (0. ..=1.).contains(&a) && r_intersect >= *r_min && r_intersect <= *r_max {
                    a
                } else {
                    -1.
                }
            }
            Obstacle::AccretionDisk {
                r_min,
                r_max,
                thickness,
            } => {
                let a = (PI / 2. - (ray_pos_t[2] % PI).abs())
                    / ((ray_pos_t_plus_dt[2] % PI).abs() - (ray_pos_t[2] % PI).abs()); // We search for an intersection in the equator plane defined by (theta=PI/2)
                let r_intersect = ray_pos_t[1] * (1. - a) + ray_pos_t_plus_dt[1] * a; // Radial position of intersection point
                                                                                      // First detection criterion : path segment intersects equator plan
                let altitude_1 = (ray_pos_t[2] - PI / 2.).sin() * ray_pos_t[1];
                let altitude_2 = (ray_pos_t_plus_dt[2] - PI / 2.).sin() * ray_pos_t_plus_dt[1];
                let da = altitude_2 - altitude_1;
                let mut collision_length = 0.;
                let sign = if altitude_2 > altitude_1 { 1. } else { -1. };

                // If the ray's too far, no collision --> ignore rest
                if a.abs() > 2. || r_intersect < *r_min || r_intersect > *r_max {
                    return -1.;
                }

                if altitude_1.abs() <= thickness / 2. && altitude_2.abs() <= thickness / 2. {
                    collision_length = step_size;
                    //println!("Both inside length : {}", collision_length);
                } else if altitude_2.abs() <= thickness / 2. {
                    collision_length = step_size * (thickness * sign / 2. + altitude_2) / da;
                    //println!("2 inside length : {}", collision_length);
                } else if altitude_1.abs() <= thickness / 2. {
                    collision_length = step_size * (thickness * sign / 2. - altitude_1) / da;
                    //println!("1 inside length : {}", collision_length);
                } else if (0. ..=1.).contains(&a) {
                    collision_length = step_size * thickness * sign / da;
                    //println!("Through length : {}", collision_length);
                }

                let mut rng = rand::thread_rng();
                let random_collision: f64 = rng.gen();
                //println!("Collision_length = {}; prob = {}", collision_length, (-collision_length).exp());
                if 1. - (-collision_length / 2.).exp() >= random_collision {
                    collision_length
                } else {
                    -1.
                }
            }
        }
    }
    pub fn color(&self, ray_pos: &Array1<f64>) -> Rgb<f64> {
        match self {
            Obstacle::BlackHole { r: _ } => Rgb::<f64>([0., 0., 0.]),
            Obstacle::BlackHolePredict { r: _ } => Rgb::<f64>([0., 0., 0.]),
            Obstacle::MaxDistance { r: _ } => Rgb::<f64>([0., 20., 50.]),
            Obstacle::Ring { r_min: _, r_max: _ } => {
                let random_gen = HybridMulti::default()
                    .set_frequency(0.05)
                    .set_octaves(6)
                    .set_lacunarity(2.)
                    .set_persistence(0.8)
                    .set_seed(0);
                let val = random_gen.get([ray_pos[1], ray_pos[2] / PI / 10.]).powi(2) * 255.;
                Rgb::<f64>([val, val * 0.7, 0.])
            }
            Obstacle::AccretionDisk {
                r_min: _,
                r_max: _,
                thickness: _,
            } => Rgb::<f64>([255., 240., 10.]),
        }
    }
}
