use crate::{diagram::DiagramBuilder, Diagram, SpatialInertia, System};
use cxx::UniquePtr;

#[derive(Clone, Copy)]
pub struct RigidBody {
    raw: &'static drake_sys::RigidBody64,
}

pub struct MultibodyPlant {
    raw: UniquePtr<drake_sys::MultibodyPlant64>,
}

impl MultibodyPlant {
    pub fn new(ts: f64) -> Self {
        Self {
            raw: drake_sys::new_multibody_plant_64(ts),
        }
    }

    pub fn add_rigid_body(
        &mut self,
        name: impl Into<String>,
        inertia: &SpatialInertia,
    ) -> RigidBody {
        RigidBody {
            raw: drake_sys::multibody_plant_add_rigid_body(
                self.raw.as_mut().unwrap(),
                name.into(),
                &inertia.raw,
            ),
        }
    }

    pub fn add_revolute_joint(&mut self, name: impl Into<String>, body: RigidBody) -> &mut Self {
        drake_sys::multibody_plant_add_revolute_joint(
            self.raw.as_mut().unwrap(),
            name.into(),
            body.raw,
        );
        self
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
