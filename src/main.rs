use ndarray::{Array, Array3};
use relativistic_ray_tracing::{camera::Position, space::Espace};

fn main() {
    println!("Hello, world!");
    let position = Position {
        r: 0.0,
        theta: 0.0,
        phi: 0.0,
    };
    let espace = Espace {
        rs: 1.0,
        c: 1.0,
        christoffel: Array3::zeros((4, 4, 4)),
    };
    let test = espace.christoffel.clone();
    println!("position {:?} and space {:?}", position, espace);
    println!("{:?}", test[[0, 3, 0]]);
}
