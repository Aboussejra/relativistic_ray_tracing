# Relativistic Ray Tracing

This is a program with a CLI  capable of rendering (a 2D image) of a simple scene in a non-Euclidean 3D space.  The technique adopted is Ray Tracing, which is based on a simulation of the paths of each ray of light simulation of the paths of each ray of light that connects a luminous object to a pixel of the camera. The particularity is that in our situation, these rays are not rectilinear but are curved by the local curvature of space-time.  The interest of the project is to generate images images of black holes.

To see what the result may look like with a black hole and an accretion disk, try :

    -b -> black_hole_radius
    -l -> image_length
    -w -> image_wigth
    
    Example run :
    
    cargo run --release --bin cli -- -b 200 -l 50 -w 50


Look at what is in `./doc` to see files explaining the physics behind the project.

# Install Rust >= 1.58.0

Install the rust toolchain with [rustup](https://rustup.rs/).
To install the appropriate rust version, just run the command below in the
project's root directory after having installed `rustup`.

    rustc --version


Project was developed using Gitlab : https://gitlab.com/Aboussejra/relativistic-ray-tracing/-/tree/master
