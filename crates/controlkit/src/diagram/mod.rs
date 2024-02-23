use cxx::UniquePtr;

mod builder;
pub use self::builder::DiagramBuilder;

pub struct Diagram {
    pub(crate) raw: UniquePtr<drake_sys::Diagram64>,
}

impl Diagram {
    pub fn builder() -> DiagramBuilder {
        DiagramBuilder::new()
    }

    pub fn graphviz(&self) -> String {
        drake_sys::diagram_get_graphviz_string(&self.raw)
    }
}
