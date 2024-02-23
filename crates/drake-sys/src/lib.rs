#[cxx::bridge(namespace = "drake_bridge")]
mod ffi {
    unsafe extern "C++" {
        include!("drake-sys/ffi/include/drake_ffi.h");

        type MultibodyPlant64;

        fn new_multibody_plant_64(ts: f64) -> UniquePtr<MultibodyPlant64>;

        fn multibody_plant_calc_total_mass_diagram(
            plant: &MultibodyPlant64,
            diagram: &Diagram64,
        ) -> f64;

        type SpatialInertia;

        type RigidBody64;

        fn new_spatial_inertia() -> UniquePtr<SpatialInertia>;

        fn new_spatial_inertia_point_mass(
            mass: f64,
            x: f64,
            y: f64,
            z: f64,
        ) -> UniquePtr<SpatialInertia>;

        fn multibody_plant_add_rigid_body(
            plant: Pin<&mut MultibodyPlant64>,
            name: String,
            inertia: &SpatialInertia,
        ) -> &'static RigidBody64;

        fn multibody_plant_add_revolute_joint(
            plant: Pin<&mut MultibodyPlant64>,
            name: String,
            body: &RigidBody64,
        );

        fn multibody_plant_finalize(plant: Pin<&mut MultibodyPlant64>);

        type Diagram64;

        type DiagramBuilder64;

        fn new_diagram_builder_64() -> UniquePtr<DiagramBuilder64>;

        fn diagram_builder_build(builder: Pin<&mut DiagramBuilder64>) -> UniquePtr<Diagram64>;

        fn diagram_get_graphviz_string(diagram: &Diagram64) -> String;

        type Integrator;

        fn new_integrator() -> UniquePtr<Integrator>;

        fn diagram_builder_add_system_integrator(
            builder: Pin<&mut DiagramBuilder64>,
            integrator: UniquePtr<Integrator>,
        );

        fn diagram_builder_add_system_multibody_plant(
            builder: Pin<&mut DiagramBuilder64>,
            plant: UniquePtr<MultibodyPlant64>,
        ) -> *mut MultibodyPlant64;

    }
}

pub use ffi::*;
