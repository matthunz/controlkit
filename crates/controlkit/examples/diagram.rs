use controlkit::{Diagram, Integrator, MultibodyPlant};

fn main() {
    let mut plant = MultibodyPlant::new(2.);
    plant.finalize();

    let mut builder = Diagram::builder();
    builder.add_system(Integrator::default());
    let plant_handle = builder.add_system(plant);

    let diagram = builder.build();
    print!("{}", diagram.graphviz());

    dbg!(plant_handle.total_mass(&diagram));
}
