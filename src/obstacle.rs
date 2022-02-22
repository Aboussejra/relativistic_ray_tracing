use image::Rgb;
use ndarray::Array1;
use noise::{HybridMulti, MultiFractal, NoiseFn, Seedable};
use rand::prelude::*;
use std::f64::consts::PI;

/// Different types of obstacles that can be generated inside the scene. Each
/// obsctacle has its own collision rules and surface color, evaluated by the
/// 'collision()' and 'color()' functions. The collision detection uses two
/// consecutive positions to trace the moving object's path. Currently, the
/// implemented types are:
///
///     - BlackHole: A collision is triggered whenever the current position is
///             inside the event horizon (radial coordinate <= black hole
///             radius). The color is pure black (0,0,0).
///     - BlackHolePredict: Triggered whenever the direction deduced by the
///             two positions points towards the black hole's disk. This helps
///             to save useless computation by stopping a path early.
///     - MaxDistance: Triggered if the distance to the origin exceeds an upper
///             limit given by the 'r' parameter. Color is (0,0,0).
///     - Ring: Flat ring placed on the equator plane (theta = PI/2 in spherical
///             coordinates), bounded by its inner radius 'r_min' and outer
///             radius 'r_max'. Collision is triggered when the path crosses
///             the plane. The 'color()' function calls a separate
///             'accretion_texture()' used to compute a procedural texture.
///     - AccretionDisk: A 3D volumetric, semi-transparent version of the 'Ring'
///             obstacle. Has an additional 'thickness' parameter. Uses the same
///             texture but the collision has a probability to occur based on the
///             path's length inside the volume.
///
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
/// (Temporary) Conversion table for black body radiation temperature to
/// RGB values scaled between 0 and 1. Corresponding temperature is
/// index * 1000 Kelvin.
///
static BLACKBODY_INTERP: [[f64; 3]; 21] = [
    [0., 0., 0.],
    [1.0000, 0.0401, 0.0000],
    [1.0000, 0.2484, 0.0061],
    [1.0000, 0.4589, 0.1483],
    [1.0000, 0.6354, 0.3684],
    [1.0000, 0.7792, 0.6180],
    [1.0000, 0.8952, 0.8666],
    [0.9102, 0.9000, 1.0000],
    [0.7644, 0.8139, 1.0000],
    [0.6693, 0.7541, 1.0000],
    [0.6033, 0.7106, 1.0000],
    [0.5551, 0.6776, 1.0000],
    [0.5187, 0.6519, 1.0000],
    [0.4904, 0.6314, 1.0000],
    [0.4677, 0.6146, 1.0000],
    [0.4493, 0.6007, 1.0000],
    [0.4341, 0.5890, 1.0000],
    [0.4212, 0.5791, 1.0000],
    [0.4103, 0.5705, 1.0000],
    [0.4009, 0.5630, 1.0000],
    [0.3928, 0.5565, 1.0000],
];
pub fn accretion_texture(
    r_min: &f64,
    _r_max: &f64,
    ray_pos: &Array1<f64>,
    max_temperature: f64,
) -> Rgb<f64> {
    let random_gen = HybridMulti::default()
        .set_frequency(0.03)
        .set_octaves(6)
        .set_lacunarity(2.)
        .set_persistence(0.8)
        .set_seed(0);
    let blackbodylum = ((1. - (r_min / ray_pos[1]).sqrt()) * (r_min / ray_pos[1]).powi(3))
        / (0.488_f64).powi(4)
        * (random_gen.get([ray_pos[1], ray_pos[2] / PI / 10.]).abs() + 1.);
    let temp = blackbodylum.powf(0.25) * max_temperature / 1000.;
    let interp = temp - temp.floor();
    Rgb::<f64>([
        ((BLACKBODY_INTERP[temp.floor() as usize][0] * (1. - interp)
            + BLACKBODY_INTERP[(temp.floor() + 1.) as usize][0] * interp)
            * blackbodylum
            * 255.),
        ((BLACKBODY_INTERP[temp.floor() as usize][1] * (1. - interp)
            + BLACKBODY_INTERP[(temp.floor() + 1.) as usize][1] * interp)
            * blackbodylum
            * 255.),
        ((BLACKBODY_INTERP[temp.floor() as usize][2] * (1. - interp)
            + BLACKBODY_INTERP[(temp.floor() + 1.) as usize][2] * interp)
            * blackbodylum
            * 255.),
    ])
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
                let altitude_1 = (ray_pos_t[2] - PI / 2.).sin() * ray_pos_t[1];
                let altitude_2 = (ray_pos_t_plus_dt[2] - PI / 2.).sin() * ray_pos_t_plus_dt[1];
                if altitude_1 * altitude_2 > 0.
                    && altitude_1.abs() > thickness / 2.
                    && altitude_2.abs() > thickness / 2.
                {
                    return -1.;
                }
                // Find intersection between path and ring: p_intersect = a * ray_pos2 + (1-a) * ray_pos1
                let a = (PI / 2. - (ray_pos_t[2] % PI).abs())
                    / ((ray_pos_t_plus_dt[2] % PI).abs() - (ray_pos_t[2] % PI).abs()); // We search for an intersection in the equator plane defined by (theta=PI/2)
                let r_intersect = ray_pos_t[1] * (1. - a) + ray_pos_t_plus_dt[1] * a; // Radial position of intersection point
                                                                                      // First detection criterion : path segment intersects equator plan
                let da = altitude_2 - altitude_1;
                let mut collision_length = 0.;
                let mut collision_point = 0.;
                let sign = if altitude_2 > altitude_1 { 1. } else { -1. };

                // If the ray's too far, no collision --> ignore rest
                if a.abs() > 2. || r_intersect < *r_min || r_intersect > *r_max {
                    return -1.;
                }

                if altitude_1.abs() <= thickness / 2. && altitude_2.abs() <= thickness / 2. {
                    collision_length = step_size;
                    collision_point = 0.5;
                    //println!("Both inside length : {}", collision_length);
                } else if altitude_2.abs() <= thickness / 2. {
                    collision_length = step_size * (thickness * sign / 2. + altitude_2) / da;
                    collision_point = 1.;
                    //println!("2 inside length : {}", collision_length);
                } else if altitude_1.abs() <= thickness / 2. {
                    collision_length = step_size * (thickness * sign / 2. - altitude_1) / da;
                    collision_point = 0.;
                    //println!("1 inside length : {}", collision_length);
                } else if (0. ..=1.).contains(&a) {
                    collision_length = step_size * thickness * sign / da;
                    collision_point = a;
                    //println!("Through length : {}", collision_length);
                }
                let transmissibility = 1.
                    - ((1. - (r_min / r_intersect).sqrt()) * (r_min / r_intersect).powi(3))
                        / (0.488_f64).powi(4);
                let mut rng = rand::thread_rng();
                let random_collision: f64 = rng.gen();
                //println!("Collision_length = {}; prob = {}", collision_length, (-collision_length).exp());
                if transmissibility.powf(collision_length) <= random_collision {
                    collision_point
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
            Obstacle::MaxDistance { r: _ } => Rgb::<f64>([0., 0., 0.]),
            Obstacle::Ring { r_min, r_max } => accretion_texture(r_min, r_max, ray_pos, 8000.),
            Obstacle::AccretionDisk {
                r_min,
                r_max,
                thickness: _,
            } => accretion_texture(r_min, r_max, ray_pos, 8000.),
        }
    }
}
