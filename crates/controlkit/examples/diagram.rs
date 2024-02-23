use controlkit::{Diagram, Integrator, InverseKinematics, MultibodyPlant, SpatialInertia};
use nalgebra::{UnitVector3, Vector3};

fn main() {
    let mut plant = MultibodyPlant::new(2.);
    let body1 = plant.add_rigid_body(
        "body1",
        &SpatialInertia::solid_cylinder_with_mass(
            20.,
            5.,
            20.,
            UnitVector3::new_normalize(Vector3::identity()),
        ),
    );
    let body2 = plant.add_rigid_body(
        "body2",
        &SpatialInertia::solid_cylinder_with_mass(
            20.,
            5.,
            20.,
            UnitVector3::new_normalize(Vector3::identity()),
        ),
    );
    let joint1 = plant.add_revolute_joint("joint1", body1, body2, Vector3::z_axis());

    plant.finalize();

    let mut builder = Diagram::builder();
    let plant_handle = builder.add_system(plant);
    let _diagram = builder.build();

    let ik = InverseKinematics::new(&plant_handle);
    dbg!(ik.solve(&plant_handle));
}
