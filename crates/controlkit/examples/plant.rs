use controlkit::MultibodyPlant;

fn main() {
    let mut plant = MultibodyPlant::new(2.);
    plant.finalize();
}
