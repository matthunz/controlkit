#include "drake-sys/ffi/include/drake_ffi.h"
#include "drake-sys/src/main.rs.h"
#include <algorithm>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>
#include "drake/common/drake_assert.h"

namespace drake_bridge
{
  std::unique_ptr<MultibodyPlant64> new_multibody_plant_64(double ts)
  {
    return std::unique_ptr<MultibodyPlant64>(new MultibodyPlant64(ts));
  }

  void multibody_plant_finalize(MultibodyPlant64& plant) {
    plant.Finalize();
  }
}
