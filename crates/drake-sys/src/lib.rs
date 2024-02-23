#[cxx::bridge(namespace = "drake_bridge")]
mod ffi {
    unsafe extern "C++" {
        include!("drake-sys/ffi/include/drake_ffi.h");

        type MultibodyPlant64;

        fn new_multibody_plant_64(ts: f64) -> UniquePtr<MultibodyPlant64>;

        fn multibody_plant_finalize(plant: Pin<&mut MultibodyPlant64>);

        type Diagram64;

        type DiagramBuilder64;

        fn new_diagram_builder_64() -> UniquePtr<DiagramBuilder64>;

        fn diagram_builder_build(builder: Pin<&mut DiagramBuilder64>) -> UniquePtr<Diagram64>;

        fn diagram_get_graphviz_string(builder: &Diagram64) -> String;
    }
}

pub use ffi::*;
