#pragma once
#include "rust/cxx.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/integrator.h"

namespace drake_bridge
{
  using MultibodyPlant64 = drake::multibody::MultibodyPlant<double>;

  using Diagram64 = drake::systems::Diagram<double>;

  using DiagramBuilder64 = drake::systems::DiagramBuilder<double>;

  std::unique_ptr<MultibodyPlant64> new_multibody_plant_64(double ts);

  double multibody_plant_calc_total_mass_diagram(const MultibodyPlant64 &plant, const Diagram64 &diagram);

  void multibody_plant_finalize(MultibodyPlant64 &plant);

  std::unique_ptr<DiagramBuilder64> new_diagram_builder_64();

  std::unique_ptr<Diagram64> diagram_builder_build(DiagramBuilder64 &builder);

  using Integrator = drake::systems::Integrator<double>;

  std::unique_ptr<Integrator> new_integrator();

  void diagram_builder_add_system_integrator(DiagramBuilder64 &builder, std::unique_ptr<Integrator> integrator);

  MultibodyPlant64* diagram_builder_add_system_multibody_plant(DiagramBuilder64 &builder, std::unique_ptr<MultibodyPlant64> plant);

  rust::string diagram_get_graphviz_string(const Diagram64 &diagram);
}
