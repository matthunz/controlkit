use controlkit::{Diagram, Integrator, MultibodyPlant, SpatialInertia};
use nalgebra::Vector3;

fn main() {
    let mut plant = MultibodyPlant::new(2.);
    let body1 = plant.add_rigid_body(
        "body1",
        &SpatialInertia::point_mass(20., Vector3::default()),
    );
    let body2 = plant.add_rigid_body(
        "body2",
        &SpatialInertia::point_mass(20., Vector3::default()),
    );
    plant.add_revolute_joint("joint1", body1, body2, Vector3::z_axis());

    plant.finalize();

    let mut builder = Diagram::builder();
    builder.add_system(Integrator::default());
    let plant_handle = builder.add_system(plant);

    let diagram = builder.build();
    print!("{}", diagram.graphviz());

    dbg!(plant_handle.total_mass(&diagram));
}
