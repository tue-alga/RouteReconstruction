#ifndef FRECHET_ON_GRAPH_H
#define FRECHET_ON_GRAPH_H
#include <LoopsAlgs/Frechet/FrechetHelpers.h>
#include "LoopsLib/Helpers/Logger.h"


namespace LoopsAlgs::Frechet
{
    class FrechetOnGraph;
}
namespace LoopsLib::Helpers
{
    template<>
    constexpr int logLevel<LoopsAlgs::Frechet::FrechetOnGraph>()
    {
        return LogLevel::Info;
    }
    template<> inline std::string ClassLogLevel<LoopsAlgs::Frechet::FrechetOnGraph>::Prefix() {
        static const char* nm = "FrechetOnGraph"; return nm;
    }
}

namespace LoopsAlgs::Frechet
{
    struct FrechetLogging
    {
        static constexpr bool LogDP = true;
        static constexpr bool AssertCorrectPointers = true;
    };


    /**
     * \brief Sets up structure for the decision problem
     * Based on https://www2.cs.arizona.edu/~alon/papers/gFrechet.pdf
     */
    class FrechetOnGraph
    {

        // The graph to build on
        Graph* m_graph;

        // The epsilon to us
        LoopsLib::NT m_epsilon;


        struct Hooks
        {
            LoopsLib::DS::BaseGraph::Id_t endVertex = -1;
            // Potentially preset the edge if we have a degenerate case
            long long fixedEdge = -1;
            Frechet::WenkSweeplineResult beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* edge,
                                                           const Frechet::PrioQueueNode& node,
                                                           const Interval& lrInterval,
                                                           const StrongFrechetGraphData& data,
                                                            const std::vector<std::vector<PathPointer>>& pathPointers);
        };
        void reconstructPath(const StrongFrechetGraphData& data, Graph::Id_t vId,
            const std::vector<std::vector<PathPointer>>& pathPointers,
            const Hooks& hooks,
            std::vector<Graph::Id_t>& edgePath);
    public:
        FrechetOnGraph(Graph* graph);

        void setTargetGraph(Graph* graph);

        void compute(const std::vector<Graph::Id_t>& path, LoopsLib::NT epsilon,
                     std::vector<Graph::Id_t>& outputPath);
        void compute(const Trajectory& trajectory, LoopsLib::NT epsilon,
            std::vector<Graph::Id_t>& outputPath);
    };
}
#endif