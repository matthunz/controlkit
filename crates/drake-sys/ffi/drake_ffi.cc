#include "drake-sys/ffi/include/drake_ffi.h"
#include "rust/cxx.h"

namespace drake_bridge
{
  std::unique_ptr<MultibodyPlant64> new_multibody_plant_64(double ts)
  {
    return std::unique_ptr<MultibodyPlant64>(new MultibodyPlant64(ts));
  }

  void multibody_plant_finalize(MultibodyPlant64 &plant)
  {
    plant.Finalize();
  }

  std::unique_ptr<DiagramBuilder64> new_diagram_builder_64()
  {
    return std::unique_ptr<DiagramBuilder64>(new DiagramBuilder64());
  }

  std::unique_ptr<Diagram64> diagram_builder_build(DiagramBuilder64 &builder)
  {
    return std::unique_ptr(builder.Build());
  }

  rust::string diagram_get_graphviz_string(const Diagram64 &diagram)
  {
    return diagram.GetGraphvizString();
  }
}
