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
 */

#include "descartes_planner/planning_graph.h"

#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <utility>
#include <algorithm>
#include <fstream>

#include <ros/console.h>

#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace descartes_core;
using namespace descartes_trajectory;
namespace descartes_planner
{
PlanningGraph::PlanningGraph(RobotModelConstPtr model)
  : robot_model_(std::move(model)), custom_cost_function_(NULL), graph_(model->getDOF())
{
}

PlanningGraph::PlanningGraph(RobotModelConstPtr model, CostFunction cost_function_callback)
  : robot_model_(std::move(model)), custom_cost_function_(cost_function_callback), graph_(model->getDOF())
{
}

PlanningGraph::~PlanningGraph()
{
}


descartes_core::RobotModelConstPtr PlanningGraph::getRobotModel()
{
  return robot_model_;
}

bool PlanningGraph::insertGraph(const std::vector<TrajectoryPtPtr>* points)
{
  // validate input
  if (!points)
  {
    // one or both are null
    ROS_ERROR_STREAM("points == null. Cannot initialize graph with null list.");
    return false;
  }
  if (points->size() < 2)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": must provide at least 2 input trajectory points.");
    return false;
  }

  // generate solutions for this point
  std::vector<std::vector<std::vector<double>>> all_joint_sols;
  if (!calculateJointSolutions(points->data(), points->size(), all_joint_sols))
  {
    return false;
  }

  // insert into graph as vertices
  graph_.allocate(points->size());
  for (std::size_t i = 0; i < points->size(); ++i)
  {
    graph_.assignRung(i, (*points)[i]->getID(), (*points)[i]->getTiming(), all_joint_sols[i]);
  }

  // now we have a graph with data in the 'rungs' and we need to compute the edges
  for (std::size_t i = 0; i < graph_.size() - 1; ++i)
  {
    // compute edges for pair 'i' and 'i+1'
    const auto& joints1 = graph_.getRung(i).data;
    const auto& joints2 = graph_.getRung(i+1).data;

    std::vector<LadderGraph::EdgeList> edges = calculateEdgeWeights(joints1, joints2, robot_model_->getDOF());

    graph_.assignEdges(i, std::move(edges));
  }

  return true;
}

bool PlanningGraph::addTrajectory(TrajectoryPtPtr point, TrajectoryPt::ID previous_id, TrajectoryPt::ID next_id)
{
  return false;
}

bool PlanningGraph::modifyTrajectory(TrajectoryPtPtr point)
{
  return false;
}

bool PlanningGraph::removeTrajectory(TrajectoryPtPtr point)
{
  return false;
}

bool PlanningGraph::getShortestPath(double& cost, std::list<JointTrajectoryPt>& path)
{
  return false;
}

bool PlanningGraph::calculateJointSolutions(const TrajectoryPtPtr* points, const std::size_t count,
                                            std::vector<std::vector<std::vector<double>>>& poses)
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
                                         const std::vector<double>& end_joints, size_t dof)
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

PlanningGraph::EdgeWeightResult PlanningGraph::edgeWeight(const JointTrajectoryPt& start,
                                                          const JointTrajectoryPt& end) const
{
  EdgeWeightResult result;
  result.first = false;

  const std::vector<double>& start_vector = start.nominal();
  const std::vector<double>& end_vector = end.nominal();
  if (start_vector.size() == end_vector.size())
  {
    // Check to see if time is specified and if so, check to see if the
    // joint motion is possible in the window provided
    if (end.getTiming().isSpecified() && !robot_model_->isValidMove(start_vector, end_vector, end.getTiming().upper))
    {
      return result;
    }

    if (custom_cost_function_)
    {
      result.second = custom_cost_function_(start_vector, end_vector);
    }
    else
    {
      double vector_diff = 0;
      for (unsigned i = 0; i < start_vector.size(); i++)
      {
        double joint_diff = std::abs(end_vector[i] - start_vector[i]);
        vector_diff += joint_diff;
      }
      result.second = vector_diff;
    }

    result.first = true;
    return result;
  }
  else
  {
    ROS_WARN_STREAM("unequal joint pose vector lengths: " << start_vector.size() << " != " << end_vector.size());
  }

  return result;
}

} /* namespace descartes_planner */
