use controlkit::{DiagramBuilder, MultibodyPlant};

fn main() {
    let mut builder = DiagramBuilder::new();
    let diagram = builder.build();
    dbg!(diagram.graphviz());
}
