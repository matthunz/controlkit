use controlkit::{DiagramBuilder, Integrator};

fn main() {
    let mut builder = DiagramBuilder::new();
    builder.add_system(Integrator::default());
    let diagram = builder.build();
    print!("{}", diagram.graphviz());
}
