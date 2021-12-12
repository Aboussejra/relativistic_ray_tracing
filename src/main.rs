use ndarray::Array3;
use relativistic_ray_tracing::{camera::Camera, ray::Ray, space::Space};

fn main() {
    let mut espace = Space {
        rs: 1.0,
        c: 1.0,
        christoffel: Array3::zeros((4, 4, 4)),
    };
    let position = [2.; 4];
    let _camera = Camera::new();
    let ray = Ray::new().position;
    println!("ray intialised {:?}", ray);
    println!("BEFORE : Space {:?}", espace);
    espace.update_christoffel(&position);
    println!("AFTER : and space {:?}", espace);
    println!("{:?}", espace.christoffel[[0, 0, 1]]);
}
