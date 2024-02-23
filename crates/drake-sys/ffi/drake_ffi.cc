#include "drake-sys/ffi/include/drake_ffi.h"
#include "rust/cxx.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "drake/multibody/tree/revolute_joint.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/multibody/parsing/parser.h>

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

  void multibody_plant_add_urdf(MultibodyPlant64 &plant, rust::String urdf)
  {
     drake::multibody::Parser(&plant).AddModelsFromString(std::string(urdf), "urdf");
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

  std::unique_ptr<SpatialInertia> new_spatial_inertia_solid_cylinder_with_mass(double mass, double radius, double length, double x, double y, double z)
  {
    Eigen::Vector3d v(x, y, z);
    return std::make_unique<SpatialInertia>(SpatialInertia::SolidCylinderWithMass(mass, radius, length, v));
  }

  const Frame64& revolute_joint_frame_on_parent(const RevoluteJoint64 &joint)
  {
    return joint.frame_on_parent();
  }



  /* -------------------------InverseKinematics------------------------- */

  std::unique_ptr<InverseKinematics> new_inverse_kinematics(const MultibodyPlant64& plant)
  {
    return std::make_unique<InverseKinematics>(plant, false);
  }

  rust::Vec<double> inverse_kinematics_solve(const InverseKinematics& ik, const MultibodyPlant64& plant) {
    // Set the initial guess for the joint positions (adjust as needed)
    Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(plant.num_positions());

    drake::solvers::SnoptSolver solver;
    drake::solvers::MathematicalProgramResult result = solver.Solve(ik.prog());

    Eigen::VectorXd joint_positions = result.GetSolution().transpose();

    rust::Vec<double> data;
    for (int i = 0; i < joint_positions.size(); ++i) {
        data.push_back(joint_positions[i]);
    }

    return data;
  }
}
