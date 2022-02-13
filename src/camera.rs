use crate::{ray::Ray, space::Space};
use ang::atan2;
use image::{ImageBuffer, Rgb, RgbImage};
use ndarray::Array1;
use num_integer::Roots;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use std::f64::consts::PI;
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Camera {
    pub position: Array1<f64>,    // r, theta, phi
    pub orientation: Array1<f64>, // theta, phi, psi
    pub im_size: [u32; 2],
    pub fov: [f64; 2],
}

impl Camera {
    pub fn new() -> Self {
        Camera {
            position: Array1::<f64>::zeros(3),
            orientation: Array1::<f64>::zeros(3),
            im_size: [100, 100],
            fov: [PI / 4.; 2],
        }
    }

    pub fn render(&self, n_rays: usize, number_steps: i32, step_size: f64, space: &mut Space) {
        let n_rays = n_rays.sqrt();
        let n_rays_float = n_rays as f64;
        let size_x = self.im_size[0];
        let size_x_float = size_x as f64;
        let size_y = self.im_size[1];
        let size_y_float = size_y as f64;
        let mut img: RgbImage = ImageBuffer::new(self.im_size[0], self.im_size[1]);

        let coordinates: Vec<(u32, u32)> = img
            .enumerate_pixels()
            .into_iter()
            .map(|(x, y, _)| (x, y))
            .collect();
        let progression = 0;
        let vec_pixels: Vec<Rgb<u8>> = coordinates
            .into_par_iter()
            .map(|(x, y)| {
                let mut r: f64 = 0.;
                let mut g: f64 = 0.;
                let mut b: f64 = 0.;
                for ray_x in 0..n_rays {
                    for ray_y in 0..n_rays {
                        let cx = ((x as f64) - (size_x_float - 1.) / 2.) * self.fov[0]
                            / size_x_float
                            + ((ray_x as f64) - (n_rays_float - 1.) / 2.) * self.fov[0]
                                / (size_x_float * n_rays_float);
                        let cy = ((y as f64) - (size_y_float - 1.) / 2.) * self.fov[1]
                            / size_y_float
                            + ((ray_y as f64) - (n_rays_float - 1.) / 2.) * self.fov[1]
                                / (size_y_float * n_rays_float);
                        let theta = PI + (cx.powi(2) + cy.powi(2)).sqrt();
                        let phi = PI / 2. + self.orientation[2] + atan2(cy, cx).in_radians();
                        if (theta - PI) == 0. {
                            println!(
                                "Zero Angle Theta: for i = {}, j = {} are {},{}",
                                x, y, ray_x, ray_y
                            );
                        }
                        let mut ray_position = Array1::<f64>::zeros(4);
                        ray_position[1] = self.position[0];
                        ray_position[2] = self.position[1];
                        ray_position[3] = self.position[2];
                        let mut ray_orientation = Array1::<f64>::zeros(2);
                        ray_orientation[0] = theta;
                        ray_orientation[1] = phi;
                        let initial_velocity = space.c;

                        let mut ray = Ray::new_i(
                            step_size,
                            &ray_position,
                            &ray_orientation,
                            initial_velocity,
                            space,
                        );
                        let d_lambda = step_size;
                        let result_trace = ray.trace(space, number_steps, d_lambda, true, false);
                        if let Some(collision) = result_trace {
                            let rgb = collision.color;
                            r += rgb[0] as f64;
                            g += rgb[1] as f64;
                            b += rgb[2] as f64;
                        } else {
                            if f64::is_nan(ray.position[1]) {
                                r += 255.;
                                g += 0.;
                                b += 0.;
                            } else {
                                r += 0.;
                                g += 0.;
                                b += 255.;
                            }
                        }
                    }
                }
                let max = r.max(g.max(b));
                let pixel = image::Rgb([
                    (r * 255. / max) as u8,
                    (g * 255. / max) as u8,
                    (b * 255. / max) as u8,
                ]);
                if progression % 100 == 0 {
                    println!(
                        "Progression : {} %",
                        progression as f64 / (size_x_float * size_y_float) * 100.
                    );
                };
                pixel
            })
            .collect();
        for ((_, _, pixel_img), pixel_calculated) in img.enumerate_pixels_mut().zip(vec_pixels) {
            *pixel_img = pixel_calculated;
        }
        let title = "render.png";
        img.save(title).expect("Problem on saving image");
    }
}
