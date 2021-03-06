use relativistic_ray_tracing::camera::Camera;

fn main() {
    let camera = Camera::new();
    println!("test camera {:?}", camera);
    // Construct a new RGB ImageBuffer with the specified width and height.
}
#[allow(non_snake_case)]
#[cfg(test)]
mod unit_tests {
    use std::f64::consts::PI;

    use image::{ImageBuffer, RgbImage};
    use ndarray::{Array1, Array3};
    use relativistic_ray_tracing::camera::Camera;
    use relativistic_ray_tracing::obstacle::Obstacle;
    use relativistic_ray_tracing::{ray::Ray, space::Space};

    #[test]
    fn ray_tracing() {
        let position = Array1::<f64>::ones(4).mapv(|elem| elem * 2.);
        println!("test {:?}", position);

        let espace = Space {
            rs: 1.0,
            c: 1.0,
            christoffel: Array3::zeros((4, 4, 4)),
            obstacles: Vec::new(),
        };

        let mut ray = Ray::new();

        println!("ray intialised {:?}", ray);
        println!("BEFORE : Space {:?}", espace);
        espace.update_christoffel(&position);
        println!("AFTER : and space {:?}", espace);
        println!("{:?}", espace.christoffel[[0, 0, 1]]);

        println!("\nTest trace() --> ray init. at (t, r, theta, phi) = (0, 6, PI/2, 0),\nwith velocity = 0 (and thus proper time speed = 1).");
        println!("=> r should decrease as the object falls into the black hole.");
        ray.position[1] = 6.;
        ray.position[2] = PI / 2.;
        ray.position_derivative[0] = 1.;
        ray.trace(&espace, 10, 1., true, false);
    }

    #[test]
    fn circular_orbit() {
        let space = Space {
            rs: 1.0,
            c: 1.0,
            christoffel: Array3::zeros((4, 4, 4)),
            obstacles: Vec::new(),
        };
        let mut position = Array1::<f64>::zeros(4);
        position[1] = 1.5;
        position[2] = PI / 2.;
        let mut orientation = Array1::<f64>::zeros(3);
        orientation[0] = PI / 2.;
        orientation[1] = PI / 2.;
        let initial_velocity = space.c; //(C.powf(2.) * space.rs / 2. / (position[1] - space.rs)).sqrt();

        let step_size = 0.4;
        let number_steps = 50;
        let mut ray = Ray::new_i(step_size, &position, &orientation, initial_velocity, &space);

        ray.trace(&space, number_steps, step_size, true, true);

        let error_margin = 1e-3;
        println!("Test initial position : {:?}", position);
        println!("Test final position : {:?}", ray.position);
        assert!(
            (ray.position[1] <= position[1] + error_margin)
                && (ray.position[1] >= position[1] - error_margin)
        );
    }

    #[test]
    fn outward_escape() {
        let space = Space {
            rs: 100.,
            c: 1.0,
            christoffel: Array3::zeros((4, 4, 4)),
            obstacles: Vec::new(),
        };
        let mut position = Array1::<f64>::zeros(4);
        position[1] = 200.;
        position[2] = PI / 2.;
        let mut orientation = Array1::<f64>::zeros(3);
        orientation[0] = 0.;
        orientation[1] = 0.;
        let initial_velocity = space.c; // Escapes at light speed : it's a photon

        let step_size = 2.;
        let number_steps = 100;
        let mut ray = Ray::new_i(step_size, &position, &orientation, initial_velocity, &space);

        ray.trace(&space, number_steps, step_size, true, true);

        let momentum_conservation = -ray.position_derivative[0].powi(2)
            * (1. - space.rs / ray.position[1])
            / space.c.powi(2)
            + ray.position_derivative[1].powi(2) / (1. - space.rs / ray.position[1])
            + (ray.position_derivative[2] * ray.position[1]).powi(2)
            + (ray.position_derivative[3] * ray.position[1] * ray.position[2].sin()).powi(2);

        let error_margin = 1e-3;
        println!("Test initial position : {:?}", position);
        println!("Test final position : {:?}", ray.position);
        assert!(
            (ray.position[1] > position[1])
                && ((ray.position[2] - position[2]).abs() <= error_margin)
                && ((ray.position[3] - position[3]).abs() <= error_margin)
                && (momentum_conservation.abs() <= error_margin)
        );
    }

    #[test]
    fn test_image_plot() {
        let _result = match std::fs::remove_file("test.png") {
            Ok(()) => (),
            Err(e) => println!("Could not remove file because of : {:?}", e),
        };
        let _img: RgbImage = ImageBuffer::new(512, 512);

        // Construct a new by repeated calls to the supplied closure.
        let mut img = ImageBuffer::from_fn(512, 512, |x, _y| {
            if x % 2 == 0 {
                image::Luma([0u8])
            } else {
                image::Luma([255u8])
            }
        });

        // Obtain the image's width and height.
        let (_width, _height) = img.dimensions();

        // Access the pixel at coordinate (100, 100).
        let _pixel = img[(100, 100)];

        // Or use the `get_pixel` method from the `GenericImage` trait.
        let pixel = *img.get_pixel(100, 100);

        // Put a pixel at coordinate (100, 100).
        img.put_pixel(100, 100, pixel);

        // Create a new ImgBuf with width: imgx and height: imgy
        let mut imgbuf = image::ImageBuffer::new(1000, 1000);
        // Iterate over the coordinates and pixels of the image
        for (x, y, pixel) in imgbuf.enumerate_pixels_mut() {
            let r = (0.3 * x as f32) as u8;
            let b = (0.3 * y as f32) as u8;
            *pixel = image::Rgb([r, 0, b]);
        }
        // Save the image as ???fractal.png???, the format is deduced from the path
        imgbuf.save("test.png").expect("Problem on saving image");
    }

    #[test]
    fn test_render() {
        let black_hole_radius = 100.;
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
        let accretionDisk = Obstacle::AccretionDisk {
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
            obstacles: Vec::from([blackhole, max_radius, accretionDisk]),
        };

        let mut cam_position = Array1::<f64>::zeros(3);
        cam_position[0] = camera_distance;
        cam_position[1] = PI * 0.455;
        let mut cam_orientation = Array1::<f64>::zeros(3);
        cam_orientation[0] = 0.;
        cam_orientation[1] = 0.;

        let camera = Camera {
            fov: [PI / 2.5, PI / 5.],
            im_size: [200, 100],
            orientation: cam_orientation,
            position: cam_position,
        };

        camera.render(4, 1000, 40., &mut space, 2.5, 0.75);
    }
}
