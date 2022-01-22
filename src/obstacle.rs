use ndarray::Array1;

pub enum Obstacle {
    BlackHole { r: f64 },
}
impl Obstacle {
    pub fn collision(&self, ray_pos_t: &Array1<f64>, ray_pos_t_plus_dt: &Array1<f64>) -> bool {
        match self {
            Obstacle::BlackHole { r } => ray_pos_t_plus_dt[1] <= *r,
        }
    }
}
