#ifndef DESCARTES_LADDER_GRAPH_H
#define DESCARTES_LADDER_GRAPH_H

#include "descartes_core/trajectory_id.h"
#include "descartes_core/trajectory_timing_constraint.h"

namespace descartes_planner
{

struct Rung
{
  descartes_core::TrajectoryID id;
  descartes_core::TimingConstraint timing;
  std::vector<double> data; // joint values
};

struct __attribute__ ((__packed__)) Edge
{
  double cost;
  unsigned idx; // from THIS rung to 'idx' into the NEXT rung
};

/**
 * @brief The LadderGraph class
 */
class LadderGraph
{
public:
  using size_type = std::size_t;
  using EdgeList = std::vector<Edge>;

  explicit LadderGraph(size_type dof) noexcept
    : dof_(dof)
  {
    assert(dof != 0);
  }

  void allocate(size_type n_rungs)
  {
    rungs_.resize(n_rungs);
    edges_.resize(n_rungs);
  }

  Rung& getRung(size_type index) noexcept
  {
    return rungs_[index];
  }

  const Rung& getRung(size_type index) const noexcept
  {
    return rungs_[index];
  }

  std::vector<EdgeList>& getEdges(size_type index) noexcept
  {
    return edges_[index];
  }

  const std::vector<EdgeList>& getEdges(size_type index) const noexcept
  {
    return edges_[index];
  }

  size_type rungSize(size_type index) const noexcept
  {
    return getRung(index).data.size() / dof_;
  }

  size_type numVertices() const noexcept
  {
    size_type count = 0;
    for (const auto& rung : rungs_)
    {
      count += rung.data.size() / dof_;
    }
    return count;
  }

  size_type size() const noexcept
  {
    return rungs_.size();
  }

  size_type dof() const noexcept
  {
    return dof_;
  }

  // Mutate edges
  void assignEdgeList(size_type rung, size_type index, EdgeList&& out_edges) // noexcept?
  {
    getEdges(rung)[index] = std::move(out_edges);
  }

  void assignEdges(size_type rung, std::vector<EdgeList>&& edges) // noexcept?
  {
    getEdges(rung) = std::move(edges);
  }

  // Mutate Vertices
  void assignRung(size_type index, descartes_core::TrajectoryID id, descartes_core::TimingConstraint time,
                  const std::vector<std::vector<double>>& sols)
  {
    Rung& r = getRung(index);
    r.id = id;
    r.timing = time;
    r.data.reserve(sols.size() * dof_);
    for (const auto& sol : sols)
    {
      r.data.insert(r.data.end(), sol.cbegin(), sol.cend());
    }
    // Given this new vertex set, build an edge list for each
    getEdges(index).resize(r.data.size());
  }

  // TODO: Implement clear rung / remove rung

private:
  const size_type dof_;
  std::vector<Rung> rungs_;
  std::vector<std::vector<EdgeList>> edges_;
};

} // descartes_planner
#endif
