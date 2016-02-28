#include <iostream>
#include <chrono>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "descartes_benchmarks/kinematics/ur5_robot_model.h"
#include "descartes_benchmarks/trajectories/create_lemniscate_curve.h"
#include "descartes_planner/dense_planner.h"
#include "descartes_trajectory/axial_symmetric_pt.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_benchmarks");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  auto robot_model = descartes_core::RobotModelPtr(new descartes_benchmarks::UR5RobotModel());
  ROS_ASSERT(robot_model->initialize("robot_description", "manipulator", "world", "tool"));

  descartes_planner::DensePlanner planner;
  ROS_ASSERT(planner.initialize(robot_model));

  EigenSTL::vector_Affine3d poses;
  ROS_ASSERT(descartes_benchmarks::createLemniscateCurve(0.07, 0.1, 200, 4,
                                                         Eigen::Vector3d(0, 0.5, 0), poses));

  // Translate to Descartes
  using DescartesTraj = std::vector<descartes_core::TrajectoryPtPtr>;

  DescartesTraj input, output;
  for (const auto& pose : poses)
  {
    auto pt = descartes_core::TrajectoryPtPtr( new descartes_trajectory::AxialSymmetricPt(pose,
                                                     M_PI/4,
                                                     descartes_trajectory::AxialSymmetricPt::Z_AXIS,
                                                     descartes_core::TimingConstraint(1.0)) );
    input.push_back(pt);
  }

  ROS_ASSERT(planner.planPath(input));
  ROS_ASSERT(planner.getPath(output));

  // Finish time


  return 0;
}
