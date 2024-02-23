use std::pin::Pin;

#[cxx::bridge(namespace = "drake_bridge")]
mod ffi {
    unsafe extern "C++" {
        include!("drake-sys/ffi/include/drake_ffi.h");

        type MultibodyPlant64;

        fn new_multibody_plant_64(ts: f64) -> UniquePtr<MultibodyPlant64>;

        fn multibody_plant_finalize(plant: Pin<&mut MultibodyPlant64>);
    }
}

fn main() {
    let mut plant = ffi::new_multibody_plant_64(2.);
    ffi::multibody_plant_finalize(plant.as_mut().unwrap());
}
