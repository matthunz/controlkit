use crate::{diagram::DiagramBuilder, Diagram, System};
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

pub struct MultibodyPlantHandle {
    ptr: *mut drake_sys::MultibodyPlant64,
}

impl MultibodyPlantHandle {
    pub fn total_mass(&self, diagram: &Diagram) -> f64 {
        drake_sys::multibody_plant_calc_total_mass_diagram(unsafe { &*self.ptr }, &diagram.raw)
    }
}

impl System for MultibodyPlant {
    type Handle = MultibodyPlantHandle;

    fn add(self, builder: &mut DiagramBuilder) -> MultibodyPlantHandle {
        let ptr = drake_sys::diagram_builder_add_system_multibody_plant(
            builder.raw.as_mut().unwrap(),
            self.raw,
        );
        MultibodyPlantHandle { ptr }
    }
}
