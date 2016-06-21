#ifndef CREATE_LEMNISCATE_CURVE_H
#define CREATE_LEMNISCATE_CURVE_H

#include "eigen_stl_containers/eigen_stl_containers.h"

namespace descartes_benchmarks
{
  bool createLemniscateCurve(double foci_distance, double sphere_radius,
                             int num_points, int num_lemniscates,
                             const Eigen::Vector3d& sphere_center,
                             EigenSTL::vector_Affine3d& poses);
}

#endif
