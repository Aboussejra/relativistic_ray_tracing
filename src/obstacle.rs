use image::Rgb;
use ndarray::Array1;
use std::f64::consts::PI;
#[derive(Debug, Clone, PartialEq)]
pub enum Obstacle {
    BlackHole { r: f64 },            // Ray cannot enter black hole
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
                if ray_pos_t_plus_dt[1] <= *r {return 0.;}
                else {return -1.;}
            },
            Obstacle::MaxDistance { r } => {
                if ray_pos_t_plus_dt[1] >= *r {return 0.;}
                else {return -1.;}
            },
            Obstacle::Ring { r_min, r_max } => {
                // Find intersection between path and ring: p_intersect = a * ray_pos2 + (1-a) * ray_pos1
                let a = (PI / 2. - (ray_pos_t[2] % PI).abs())
                    / ((ray_pos_t_plus_dt[2] % PI).abs() - (ray_pos_t[2] % PI).abs()); // We search for an intersection in the equator plane defined by (theta=PI/2)
                let r_intersect = ray_pos_t[1] * (1. - a) + ray_pos_t_plus_dt[1] * a; // Radial position of intersection point
                if (0. ..=1.).contains(&a) && r_intersect >= *r_min && r_intersect <= *r_max {
                    return a;
                }
                else {return -1.;}
            }
        }
    }
    pub fn color(&self, _ray_pos: &Array1<f64>) -> Rgb<u8> {
        match self {
            Obstacle::BlackHole { r: _ } => Rgb::<u8>([0, 0, 0]),
            Obstacle::MaxDistance { r: _ } => Rgb::<u8>([0, 50, 0]),
            Obstacle::Ring { r_min: _, r_max: _ } => Rgb::<u8>([255, 255, 10]),
        }
    }
}
