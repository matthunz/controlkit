use crate::{diagram::DiagramBuilder, System};
use cxx::UniquePtr;

pub struct MultibodyPlant {
    raw: UniquePtr<drake_sys::MultibodyPlant64>,
}

impl MultibodyPlant {
    pub fn new(ts: f64) -> Self {
        Self {
            raw: drake_sys::new_multibody_plant_64(ts),
        }
    }

    pub fn finalize(&mut self) {
        let ptr = self.raw.as_mut().unwrap();
        drake_sys::multibody_plant_finalize(ptr)
    }
}

impl System for MultibodyPlant {
    fn add(self, builder: &mut DiagramBuilder) {
        drake_sys::diagram_builder_add_system_multibody_plant(
            builder.raw.as_mut().unwrap(),
            self.raw,
        );
    }
}