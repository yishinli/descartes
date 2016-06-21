#ifndef DESCARTES_LADDER_GRAPH_DIJKSTRAS_H
#define DESCARTES_LADDER_GRAPH_DIJKSTRAS_H

#include "descartes_planner/ladder_graph.h"

namespace descartes_planner
{

struct VD // Vertex Descriptor
{
  unsigned rung;
  unsigned index;
};

class DijkstrasSearch
{
public:
  DijkstrasSearch(LadderGraph& graph);

  std::pair<unsigned, double> run();

private:
  const LadderGraph& graph_;

  enum Color {WHITE, BLACK, GRAY};

  size_t index(VD a) const
  {
    return solution_[a.rung].n_start + a.index;
  }

  double& distance(VD v)
  {
    return solution_[v.rung].distance[v.index];
  }

  VD& predecessor(VD v)
  {
    return solution_[v.rung].predecessor[v.index];
  }

  Color& color(VD v)
  {
    return solution_[v.rung].colors[v.index];
  }

  struct SolutionRung
  {
    size_t n_start;
    std::vector<double> distance;
    std::vector<VD> predecessor;
    std::vector<Color> colors;
  };

  std::vector<SolutionRung> solution_;
  std::size_t N;
};


} // descartes_planner
#endif
