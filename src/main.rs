use std::f64::consts::PI;

use ndarray::{Array1, Array3};
use relativistic_ray_tracing::{camera::Camera, ray::Ray, space::Space};
extern crate image;

use image::{GenericImage, GenericImageView, ImageBuffer, RgbImage};
fn main() {
    let mut espace = Space {
        rs: 1.0,
        c: 1.0,
        christoffel: Array3::zeros((4, 4, 4)),
    };

    let position = Array1::<f64>::ones(4).mapv(|elem| elem * 2.);
    println!("test {:?}", position);

    let camera = Camera::new();
    println!("test camera {:?}", camera);

    let mut ray = Ray::new();

    println!("ray intialised {:?}", ray);
    println!("BEFORE : Space {:?}", espace);
    espace.update_christoffel(&position);
    println!("AFTER : and space {:?}", espace);
    println!("{:?}", espace.christoffel[[0, 0, 1]]);

    println!("\nTest trace() --> ray init. at (t, r, theta, phi) = (0, 6, PI/2, 0),\nwith velocity = 0 (and thus proper time speed = 1).");
    println!("=> r should decrease as the object falls into the black hole.");
    ray.position[1] = 6.;
    ray.position[2] = PI/2.;
    ray.position_derivative[0] = 1.;
    ray.trace(&mut espace, 10, 1.);

    // Construct a new RGB ImageBuffer with the specified width and height.
    let img: RgbImage = ImageBuffer::new(512, 512);

    // Construct a new by repeated calls to the supplied closure.
    let mut img = ImageBuffer::from_fn(512, 512, |x, y| {
        if x % 2 == 0 {
            image::Luma([0u8])
        } else {
            image::Luma([255u8])
        }
    });

    // Obtain the image's width and height.
    let (width, height) = img.dimensions();

    // Access the pixel at coordinate (100, 100).
    let pixel = img[(100, 100)];

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
    // Save the image as “fractal.png”, the format is deduced from the path
    imgbuf.save("test.png").expect("Problem on saving image");
}
