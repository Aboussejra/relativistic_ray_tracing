use std::{error, f64::consts::PI};

use clap::{self, Arg, Command};
use ndarray::{Array1, Array3};
use relativistic_ray_tracing::{camera::Camera, obstacle::Obstacle, space::Space};
fn main() -> Result<(), Box<dyn error::Error>> {
    let matches = app().get_matches();
    let black_hole_radius = match matches.value_of("black_hole_radius") {
        None => 100.,
        Some(b) => match b.parse::<f64>() {
            Ok(b) => b,
            Err(_) => panic!("Could not parse black hole radius {} to f64!", b),
        },
    };
    let image_length = match matches.value_of("image_length") {
        None => 200,
        Some(l) => match l.parse::<u32>() {
            Ok(l) => l,
            Err(_) => panic!("Could not parse image length {} to u32!", l),
        },
    };
    let image_width = match matches.value_of("image_width") {
        None => 200,
        Some(w) => match w.parse::<u32>() {
            Ok(w) => w,
            Err(_) => panic!("Could not parse image width {} to u32!", w),
        },
    };
    println!("Black hole radius {}", black_hole_radius);
    println!("Image length {}", image_length);
    println!("Image width {}", image_width);
    let camera_distance = 30. * black_hole_radius;
    let blackhole = Obstacle::BlackHole {
        r: black_hole_radius,
    };
    let _blackholepred = Obstacle::BlackHolePredict {
        r: black_hole_radius,
    };
    let _ring = Obstacle::Ring {
        r_min: 3. * black_hole_radius,
        r_max: 20. * black_hole_radius,
        temperature: 3000.,
    };
    let accretion_disk = Obstacle::AccretionDisk {
        r_min: 3. * black_hole_radius,
        r_max: 20. * black_hole_radius,
        thickness: 1.,
        temperature: 2500.,
    };
    let max_radius = Obstacle::MaxDistance {
        r: camera_distance * 1.1,
    };
    let mut space = Space {
        rs: 100.0,
        c: 1.0,
        christoffel: Array3::zeros((4, 4, 4)),
        obstacles: Vec::from([blackhole, max_radius, accretion_disk]),
    };

    let mut cam_position = Array1::<f64>::zeros(3);
    cam_position[0] = camera_distance;
    cam_position[1] = PI * 0.455;
    let mut cam_orientation = Array1::<f64>::zeros(3);
    cam_orientation[0] = 0.;
    cam_orientation[1] = 0.;

    let camera = Camera {
        fov: [PI / 2.5, PI / 5.],
        im_size: [image_length, image_width],
        orientation: cam_orientation,
        position: cam_position,
    };
    camera.render(4, 1000, 40., &mut space, 2.5, 0.75);
    Ok(())
}

fn app() -> clap::Command<'static> {
    Command::new("relativistic ray tracing CLI")
        .version(clap::crate_version!())
        .author(clap::crate_authors!())
        .about(
            "
Run the relativistic ray tracer from a command line interface.
        ",
        )
        .arg(
            Arg::new("black_hole_radius")
                .short('b')
                .help("Set the black hole radius for the image that will be traced")
                .takes_value(true),
        )
        .arg(
            Arg::new("image_length")
                .short('l')
                .help("output image length")
                .takes_value(true),
        )
        .arg(
            Arg::new("image_width")
                .short('w')
                .help("output image width")
                .takes_value(true),
        )
}
#[test]
fn verify_app() {
    app().debug_assert();
}
