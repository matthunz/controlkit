pub mod diagram;
pub use self::diagram::Diagram;
use cxx::UniquePtr;
use diagram::DiagramBuilder;
use nalgebra::Vector3;

mod integrator;
pub use self::integrator::Integrator;

mod multibody_plant;
pub use multibody_plant::MultibodyPlant;

pub trait System {
    type Handle;

    fn add(self, builder: &mut DiagramBuilder) -> Self::Handle;
}

pub struct SpatialInertia {
    raw: UniquePtr<drake_sys::SpatialInertia>,
}

impl SpatialInertia {
    pub fn point_mass(mass: f64, pos: Vector3<f64>) -> Self {
        Self {
            raw: drake_sys::new_spatial_inertia_point_mass(mass, pos.x, pos.y, pos.z),
        }
    }
}

impl Default for SpatialInertia {
    fn default() -> Self {
        Self {
            raw: drake_sys::new_spatial_inertia(),
        }
    }
}
