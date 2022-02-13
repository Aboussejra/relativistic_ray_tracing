use image::Rgb;
use ndarray::Array1;
use std::f64::consts::PI;
#[derive(Debug, Clone, PartialEq)]
pub enum Obstacle {
    BlackHole { r: f64 },            // Ray cannot enter black hole
    BlackHolePredict { r: f64 },     // Ray cannot point towards black hole
    MaxDistance { r: f64 },          // Ray cannot escape max distance from origin
    Ring { r_min: f64, r_max: f64 }, // 2D ring placed on the equator plane in spherical coordinates, with inner and outer radii
}
#[derive(Debug, Clone, PartialEq)]
pub struct CollisionPoint {
    pub collision_point: Array1<f64>,
    pub color: Rgb<u8>,
}
impl Obstacle {
    pub fn collision(&self, ray_pos_t: &Array1<f64>, ray_pos_t_plus_dt: &Array1<f64>) -> f64 {
        match self {
            Obstacle::BlackHole { r } => {
                if ray_pos_t_plus_dt[1] <= *r {
                    return 0.;
                } else {
                    return -1.;
                }
            }
            Obstacle::MaxDistance { r } => {
                if ray_pos_t_plus_dt[1] >= *r {
                    return 0.;
                } else {
                    return -1.;
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
                    return 0.;
                } else {
                    return -1.;
                }
            }
            Obstacle::Ring { r_min, r_max } => {
                // Find intersection between path and ring: p_intersect = a * ray_pos2 + (1-a) * ray_pos1
                let a = (PI / 2. - (ray_pos_t[2] % PI).abs())
                    / ((ray_pos_t_plus_dt[2] % PI).abs() - (ray_pos_t[2] % PI).abs()); // We search for an intersection in the equator plane defined by (theta=PI/2)
                let r_intersect = ray_pos_t[1] * (1. - a) + ray_pos_t_plus_dt[1] * a; // Radial position of intersection point
                if (0. ..=1.).contains(&a) && r_intersect >= *r_min && r_intersect <= *r_max {
                    return a;
                } else {
                    return -1.;
                }
            }
        }
    }
    pub fn color(&self, _ray_pos: &Array1<f64>) -> Rgb<u8> {
        match self {
            Obstacle::BlackHole { r: _ } => Rgb::<u8>([0, 0, 0]),
            Obstacle::BlackHolePredict { r: _ } => Rgb::<u8>([255, 0, 255]),
            Obstacle::MaxDistance { r: _ } => Rgb::<u8>([0, 20, 50]),
            Obstacle::Ring { r_min: _, r_max: _ } => Rgb::<u8>([255, 255, 10]),
        }
    }
}
