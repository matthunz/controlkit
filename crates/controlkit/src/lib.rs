use cxx::UniquePtr;

pub struct DiagramBuilder {
    raw: UniquePtr<drake_sys::DiagramBuilder64>,
}

impl DiagramBuilder {
    pub fn new() -> Self {
        Self {
            raw: drake_sys::new_diagram_builder_64(),
        }
    }

    pub fn build(&mut self) -> Diagram {
        Diagram {
            raw: drake_sys::diagram_builder_build(self.raw.as_mut().unwrap()),
        }
    }
}

pub struct Diagram {
    raw: UniquePtr<drake_sys::Diagram64>,
}

impl Diagram {
    pub fn graphviz(&self) -> String {
        drake_sys::diagram_get_graphviz_string(&self.raw)
    }
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

    pub fn finalize(&mut self) {
        let ptr = self.raw.as_mut().unwrap();
        drake_sys::multibody_plant_finalize(ptr)
    }
}
