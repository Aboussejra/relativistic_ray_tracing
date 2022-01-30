use ndarray::Array1;
use std::f64::consts::PI;

#[derive(Debug, Clone, PartialEq)]
pub enum Obstacle {
    BlackHole { r: f64 },            // Ray cannot enter black hole
    MaxDistance { r: f64 },          // Ray cannot escape max distance from origin
    Ring { r_min: f64, r_max: f64 }, // 2D ring placed on the equator plane in spherical coordinates, with inner and outer radii
}
#[derive(Debug, Default, Clone, PartialEq)]
pub struct CollisionPoint {
    pub has_collided: bool,
    pub collision_point: Array1<f64>,
}
impl Obstacle {
    pub fn collision(&self, ray_pos_t: &Array1<f64>, ray_pos_t_plus_dt: &Array1<f64>) -> bool {
        match self {
            Obstacle::BlackHole { r } => ray_pos_t_plus_dt[1] <= *r,
            Obstacle::MaxDistance { r } => ray_pos_t_plus_dt[1] >= *r,
            Obstacle::Ring { r_min, r_max } => {
                // Find intersection between path and ring: p_intersect = alpha * ray_pos2 + (1-alpha) * ray_pos1
                let alpha = (PI / 2. - (ray_pos_t[2] % PI).abs())
                    / ((ray_pos_t_plus_dt[2] % PI).abs() - (ray_pos_t[2] % PI).abs()); // We search for an intersection in the equator plane defined by (theta=PI/2)
                let r_intersect = ray_pos_t[1] * (1. - alpha) + ray_pos_t_plus_dt[1] * alpha; // Radial position of intersection point
                alpha >= 0. && alpha <= 1. && r_intersect >= *r_min && r_intersect <= *r_max
            }
        }
    }
}
