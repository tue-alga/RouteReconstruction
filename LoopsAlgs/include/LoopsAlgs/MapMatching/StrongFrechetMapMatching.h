#ifndef LOOPSALGS_MAPMATCHING_STRONGFRECHETMAPMATCHING_H
#define LOOPSALGS_MAPMATCHING_STRONGFRECHETMAPMATCHING_H
#include <LoopsLib/DS/BaseGraph.h>
#include "LoopsAlgs/Frechet/FrechetHelpers.h"
#include "LoopsAlgs/Frechet/FrechetOnGraph.h"

namespace LoopsAlgs::MapMatching
{
    class StrongFrechetMapMatching
    {
        LoopsLib::DS::EmbeddedGraph* m_graph = nullptr;
    public:
        using MapMatchedPath = std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>;

        StrongFrechetMapMatching() {}
        void setTargetGraph(LoopsLib::DS::EmbeddedGraph* graph)
        {
            m_graph = graph;
        }
        StrongFrechetMapMatching(LoopsLib::DS::EmbeddedGraph* graph):m_graph(graph){}

        void apply(const Frechet::Trajectory& trajectory, Frechet::NT lowerBound, Frechet::NT precision,
                   std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>& mapmatchedPath,
                   Frechet::NT& epsilon);
        bool applyFixed(const Frechet::Trajectory& trajectory, Frechet::NT epsilon,
            std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>& mapmatchedPath);
    };
}
#endif