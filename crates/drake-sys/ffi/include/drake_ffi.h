#pragma once
#include "rust/cxx.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake_bridge
{
  using MultibodyPlant64 = drake::multibody::MultibodyPlant<double>;

  std::unique_ptr<MultibodyPlant64> new_multibody_plant_64(double ts);

  void multibody_plant_finalize(MultibodyPlant64 &plant);

  using Diagram64 = drake::systems::Diagram<double>;

  using DiagramBuilder64 = drake::systems::DiagramBuilder<double>;

  std::unique_ptr<DiagramBuilder64> new_diagram_builder_64();

  std::unique_ptr<Diagram64> diagram_builder_build(DiagramBuilder64 &builder);

  rust::string diagram_get_graphviz_string(const Diagram64 &diagram);
}
