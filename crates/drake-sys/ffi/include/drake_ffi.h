#pragma once
#include "rust/cxx.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/integrator.h"
#include "Eigen/Core"
#include "drake/multibody/tree/revolute_joint.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>

namespace drake_bridge
{
  using MultibodyPlant64 = drake::multibody::MultibodyPlant<double>;

  using Diagram64 = drake::systems::Diagram<double>;
  using DiagramBuilder64 = drake::systems::DiagramBuilder<double>;

  using RigidBody64 = drake::multibody::RigidBody<double>;
  using Frame64 = drake::multibody::Frame<double>;
  using RevoluteJoint64 = drake::multibody::RevoluteJoint<double>;

  using InverseKinematics  = drake::multibody::InverseKinematics;

  std::unique_ptr<MultibodyPlant64> new_multibody_plant_64(double ts);

  double multibody_plant_calc_total_mass_diagram(const MultibodyPlant64 &plant, const Diagram64 &diagram);

  
  
  using SpatialInertia = drake::multibody::SpatialInertia<double>;

  std::unique_ptr<SpatialInertia> new_spatial_inertia();

  std::unique_ptr<SpatialInertia> new_spatial_inertia_point_mass(double mass, double x, double y, double z);



  const RigidBody64& multibody_plant_add_rigid_body(MultibodyPlant64 &plant, rust::string name, const SpatialInertia &inertia);

  const RevoluteJoint64& multibody_plant_add_revolute_joint(MultibodyPlant64 &plant, rust::String name, const RigidBody64&  body_a, const RigidBody64&  body_b, double x, double y, double z);

  void multibody_plant_finalize(MultibodyPlant64 &plant);

  std::unique_ptr<DiagramBuilder64> new_diagram_builder_64();

  std::unique_ptr<Diagram64> diagram_builder_build(DiagramBuilder64 &builder);

  using Integrator = drake::systems::Integrator<double>;

  std::unique_ptr<Integrator> new_integrator();

  void diagram_builder_add_system_integrator(DiagramBuilder64 &builder, std::unique_ptr<Integrator> integrator);

  MultibodyPlant64 *diagram_builder_add_system_multibody_plant(DiagramBuilder64 &builder, std::unique_ptr<MultibodyPlant64> plant);

  rust::string diagram_get_graphviz_string(const Diagram64 &diagram);



  /* -------------------------------Frame------------------------------- */

  const Frame64& revolute_joint_frame_on_parent(const RevoluteJoint64 &joint);

  /* -------------------------InverseKinematics------------------------- */

  std::unique_ptr<InverseKinematics> new_inverse_kinematics(const MultibodyPlant64& plant);
}
