/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * planning_graph.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 *      Author: Jonathan Meyer
 */

#include "descartes_planner/planning_graph.h"
#include "descartes_planner/ladder_graph_dijkstras.h"

#include <ros/console.h>

using namespace descartes_core;
using namespace descartes_trajectory;

namespace descartes_planner
{

PlanningGraph::PlanningGraph(RobotModelConstPtr model, CostFunction cost_function_callback)
  : graph_(model->getDOF()), robot_model_(std::move(model)), custom_cost_function_(cost_function_callback)
{
}

bool PlanningGraph::insertGraph(const std::vector<TrajectoryPtPtr>& points)
{
  if (points.size() < 2)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": must provide at least 2 input trajectory points.");
    return false;
  }

  // generate solutions for this point
  std::vector<std::vector<std::vector<double>>> all_joint_sols;
  if (!calculateJointSolutions(points.data(), points.size(), all_joint_sols))
  {
    return false;
  }

  // insert into graph as vertices
  graph_.allocate(points.size());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    graph_.assignRung(i, points[i]->getID(), points[i]->getTiming(), all_joint_sols[i]);
  }

  // now we have a graph with data in the 'rungs' and we need to compute the edges
  for (std::size_t i = 0; i < graph_.size() - 1; ++i)
  {
    // compute edges for pair 'i' and 'i+1'
    const auto& joints1 = graph_.getRung(i).data;
    const auto& joints2 = graph_.getRung(i+1).data;
    const auto& tm = graph_.getRung(i + 1).timing;

    std::vector<LadderGraph::EdgeList> edges = calculateEdgeWeights(joints1, joints2, robot_model_->getDOF(), tm);

    graph_.assignEdges(i, std::move(edges));
  }

  return true;
}

bool PlanningGraph::addTrajectory(TrajectoryPtPtr point, TrajectoryPt::ID previous_id, TrajectoryPt::ID next_id)
{
  return false; // TODO
}

bool PlanningGraph::modifyTrajectory(TrajectoryPtPtr point)
{
  return false; // TODO
}

bool PlanningGraph::removeTrajectory(TrajectoryPtPtr point)
{
  // Remove a point from the graph
  auto s = graph_.indexOf(point->getID());
  if (!s.second) return false;

  // remove the vertices associated with this point

  // remove all out edges from previous rung, if applicable
  // recompute edges from previous rung to new rung, if applicable

  // TODO
  return false;
}

bool PlanningGraph::getShortestPath(double& cost, std::list<JointTrajectoryPt>& path)
{
  DijkstrasSearch search (graph_);
  auto min_cost = search.run();
  auto path_idxs = search.shortestPath();
  cost = min_cost;
  const auto dof = graph_.dof();

  for (size_t i = 0; i < path_idxs.size(); ++i)
  {
    const auto idx = path_idxs[i];
    const auto* data = graph_.dataAt(i, idx);
    const auto& tm = graph_.getRung(i).timing;

    auto pt = JointTrajectoryPt(std::vector<double>(data, data + dof), tm);
    path.push_back(std::move(pt));
  }

  ROS_INFO("Computed path of length %lu with cost %lf", path_idxs.size(), cost);

  return cost != std::numeric_limits<double>::infinity();
}

bool PlanningGraph::calculateJointSolutions(const TrajectoryPtPtr* points, const std::size_t count,
                                            std::vector<std::vector<std::vector<double>>>& poses) const
{
  poses.resize(count);

  for (std::size_t i = 0; i < count; ++i)
  {
    std::vector<std::vector<double>> joint_poses;
    points[i]->getJointPoses(*robot_model_, joint_poses);

    if (joint_poses.empty())
    {
      ROS_ERROR_STREAM(__FUNCTION__ << ": IK failed for input trajectory point with ID = " << points[i]->getID());
     return false;
    }

    poses[i] = std::move(joint_poses);
  }

  return true;
}

std::vector<LadderGraph::EdgeList> PlanningGraph::calculateEdgeWeights(const std::vector<double>& start_joints,
                                         const std::vector<double>& end_joints, size_t dof, const TimingConstraint& tm) const
{
  const auto from_size = start_joints.size();
  const auto to_size = end_joints.size();
  const auto n_start_points = from_size / dof;
  const auto n_end_points = to_size / dof;

  std::vector<LadderGraph::EdgeList> edges (n_start_points);

  LadderGraph::EdgeList edge_scratch (n_end_points);

  for (size_t i = 0; i < from_size; i += dof) // from rung
  {
    size_t count = 0;
    unsigned idx = 0;

    for (size_t j = 0; j < to_size; j += dof) // to rung
    {
      if (!robot_model_->isValidMove(start_joints.data() + i, end_joints.data() + j, tm.upper))
      {
        idx++;
        continue;
      }

      double cost = 0.0;
      for (size_t k = 0; k < dof; ++k)
      {
        cost += std::abs(start_joints[i + k] - end_joints[j + k]);
      }
      edge_scratch[count++] = {cost, idx};
      idx++;
    }

    edges[i/dof] = LadderGraph::EdgeList(edge_scratch.begin(), edge_scratch.begin() + count);
  }

  return edges;
}

} /* namespace descartes_planner */
