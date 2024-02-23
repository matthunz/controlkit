#pragma once
#include "rust/cxx.h"
#include <memory>
#include "drake/multibody/plant/multibody_plant.h"

namespace drake_bridge
{
  using MultibodyPlant64 = drake::multibody::MultibodyPlant<double>;

  std::unique_ptr<MultibodyPlant64> new_multibody_plant_64(double ts);

  void multibody_plant_finalize(MultibodyPlant64& plant);
}
