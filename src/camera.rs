use crate::{ray::Ray, space::Space};
use ang::atan2;
use image::{ImageBuffer, RgbImage};
use ndarray::Array1;
use num_integer::Roots;
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
        let img: RgbImage = ImageBuffer::new(self.im_size[0], self.im_size[1]);
        for i in 0..size_x {
            for j in 0..size_y {
                for ray_x in 0..n_rays {
                    for ray_y in 0..n_rays {
                        let cx = (i as f64 - (size_x_float - 1.) / 2.) * self.fov[0] / size_x_float
                            + (ray_x as f64 - n_rays_float / 2.) * self.fov[0]
                                / (size_x_float * (n_rays_float - 1.));
                        let cy = (j as f64 - (size_y_float - 1.) / 2.) * self.fov[1] / size_y_float
                            + (ray_y as f64 - n_rays_float / 2.) * self.fov[1]
                                / (size_y_float * (n_rays_float - 1.));
                        let theta = PI + (cx.powf(2.) + cy.powf(2.)).sqrt();
                        let phi = PI / 2. + self.orientation[1] + atan2(cy, cx).in_radians();
                        if (theta - PI) == 0. {
                            println!(
                                "Zero Angle Theta: for i = {}, j = {} are {},{}",
                                i, j, ray_x, ray_y
                            );
                        }
                        let mut ray_orientation = Array1::<f64>::zeros(2);
                        ray_orientation[0] = theta;
                        ray_orientation[1] = phi;
                        let initial_velocity = space.c;

                        let mut ray = Ray::new_i(
                            step_size,
                            &self.position,
                            &ray_orientation,
                            initial_velocity,
                            space,
                        );
                        let d_lambda = step_size;
                        ray.trace(space, number_steps, d_lambda);
                    }
                }
            }
        }
        let title = format!("render_dims_{:?}.png", img.dimensions());
        img.save(title).expect("Problem on saving image");
    }
}
