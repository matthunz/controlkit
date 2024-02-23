use crate::{Diagram, System};
use cxx::UniquePtr;

pub struct DiagramBuilder {
    pub(crate) raw: UniquePtr<drake_sys::DiagramBuilder64>,
}

impl DiagramBuilder {
    pub fn new() -> Self {
        Self {
            raw: drake_sys::new_diagram_builder_64(),
        }
    }

    pub fn add_system<S: System>(&mut self, system: S) -> S::Handle {
        system.add(self)
    }

    pub fn build(&mut self) -> Diagram {
        Diagram {
            raw: drake_sys::diagram_builder_build(self.raw.as_mut().unwrap()),
        }
    }
}
