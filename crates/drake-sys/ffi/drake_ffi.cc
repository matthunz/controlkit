#include "drake-sys/ffi/include/drake_ffi.h"
#include "rust/cxx.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake_bridge
{
  std::unique_ptr<MultibodyPlant64> new_multibody_plant_64(double ts)
  {
    return std::unique_ptr<MultibodyPlant64>(new MultibodyPlant64(ts));
  }

  double multibody_plant_calc_total_mass_diagram(const MultibodyPlant64 &plant, const Diagram64 &diagram)
  {
    auto cx = diagram.CreateDefaultContext();
    return plant.CalcTotalMass(plant.GetMyContextFromRoot(*cx));
  }

  const RigidBody64& multibody_plant_add_rigid_body(MultibodyPlant64 &plant, rust::String name, const SpatialInertia &inertia)
  {
    return plant.AddRigidBody(std::string(name), inertia);
  }

  const RevoluteJoint64& multibody_plant_add_revolute_joint(MultibodyPlant64 &plant,  rust::String name, const RigidBody64&  body_a, const RigidBody64&  body_b, double x, double y, double z)
  {
    Eigen::Vector3d v(x, y, z);
    return plant.AddJoint<drake::multibody::RevoluteJoint>(std::string(name), body_a, {}, body_b, {}, v);
  }

  void multibody_plant_finalize(MultibodyPlant64 &plant)
  {
    plant.Finalize();
  }

  std::unique_ptr<DiagramBuilder64> new_diagram_builder_64()
  {
    return std::unique_ptr<DiagramBuilder64>(new DiagramBuilder64());
  }

  std::unique_ptr<Integrator> new_integrator()
  {
    return std::make_unique<Integrator>(2.);
  }

  void diagram_builder_add_system_integrator(DiagramBuilder64 &builder, std::unique_ptr<Integrator> integrator)
  {
    builder.AddSystem(std::move(integrator));
  }

  MultibodyPlant64 *diagram_builder_add_system_multibody_plant(DiagramBuilder64 &builder, std::unique_ptr<MultibodyPlant64> plant)
  {
    return builder.AddSystem(std::move(plant));
  }

  std::unique_ptr<Diagram64> diagram_builder_build(DiagramBuilder64 &builder)
  {
    return std::unique_ptr(builder.Build());
  }

  rust::string diagram_get_graphviz_string(const Diagram64 &diagram)
  {
    return diagram.GetGraphvizString();
  }



  std::unique_ptr<SpatialInertia> new_spatial_inertia()
  {
    return std::make_unique<SpatialInertia>();
  }

  std::unique_ptr<SpatialInertia> new_spatial_inertia_point_mass(double mass, double x, double y, double z)
  {
    Eigen::Vector3d com(x, y, z);

    return std::make_unique<SpatialInertia>(SpatialInertia::PointMass(mass, com));
  }



  const Frame64& revolute_joint_frame_on_parent(const RevoluteJoint64 &joint)
  {
    return joint.frame_on_parent();
  }
}
