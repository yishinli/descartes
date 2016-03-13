#include <iostream>
#include <chrono>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "descartes_benchmarks/kinematics/ur5_robot_model.h"
#include "descartes_benchmarks/trajectories/create_lemniscate_curve.h"
#include "descartes_planner/dense_planner.h"
#include "descartes_trajectory/axial_symmetric_pt.h"

#define DESCARTES_EXPECTS(v) do { if (!(v)) abort(); } while (false);

void escape(void* p)
{
  asm volatile("" : : "g"(p) : "memory");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_benchmarks");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  auto robot_model = descartes_core::RobotModelPtr(new descartes_benchmarks::UR5RobotModel());
  DESCARTES_EXPECTS(robot_model->initialize("robot_description", "manipulator", "world", "tool"));
  robot_model->setCheckCollisions(false);

  descartes_planner::DensePlanner planner;
  DESCARTES_EXPECTS(planner.initialize(robot_model));

  EigenSTL::vector_Affine3d poses;
  DESCARTES_EXPECTS(descartes_benchmarks::createLemniscateCurve(0.07, 0.1, 200, 6,
                                                         Eigen::Vector3d(0, 0.5, 0), poses));

  // Translate to Descartes
  using DescartesTraj = std::vector<descartes_core::TrajectoryPtPtr>;

  DescartesTraj input, output;
  for (const auto& pose : poses)
  {
    auto pt = descartes_core::TrajectoryPtPtr( new descartes_trajectory::AxialSymmetricPt(pose,
                                                     M_PI/4,
                                                     descartes_trajectory::AxialSymmetricPt::Z_AXIS,
                                                     descartes_core::TimingConstraint(0.1)) );
    input.push_back(pt);
  }

  auto start_tm = std::chrono::steady_clock::now();

  DESCARTES_EXPECTS(planner.planPath(input));
  DESCARTES_EXPECTS(planner.getPath(output));

  escape(output.data());

  // Finish time
  auto finish_tm = std::chrono::steady_clock::now();
  auto dt =  std::chrono::duration_cast<std::chrono::milliseconds>(finish_tm - start_tm);
  std::cout << "Total time (ms): " << dt.count () << std::endl;
  std::cout << "Trajectory size: " << output.size() << std::endl;

  return 0;
}
