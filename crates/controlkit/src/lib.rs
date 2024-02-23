pub mod diagram;
pub use self::diagram::Diagram;
use diagram::DiagramBuilder;

mod integrator;
pub use self::integrator::Integrator;

mod multibody_plant;
pub use multibody_plant::MultibodyPlant;

pub trait System {
    fn add(self, builder: &mut DiagramBuilder);
}
