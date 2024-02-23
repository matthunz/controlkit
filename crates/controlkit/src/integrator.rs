use crate::{diagram::DiagramBuilder, System};
use cxx::UniquePtr;

pub struct Integrator {
    raw: UniquePtr<drake_sys::Integrator>,
}

impl Default for Integrator {
    fn default() -> Self {
        Self {
            raw: drake_sys::new_integrator(),
        }
    }
}

impl System for Integrator {
    fn add(self, builder: &mut DiagramBuilder) {
        drake_sys::diagram_builder_add_system_integrator(builder.raw.as_mut().unwrap(), self.raw);
    }
}
