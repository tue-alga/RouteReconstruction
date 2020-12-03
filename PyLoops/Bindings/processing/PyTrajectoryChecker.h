#ifndef PYLOOPS_PROCESSING_PYTRAJECTORYCHECKER_H
#define PYLOOPS_PROCESSING_PYTRAJECTORYCHECKER_H
#include <pybind11/pybind11.h>
#include "core/gps.hpp"
#include <LoopsAlgs/Frechet/CleanAltSweepline.h>
#include <Helpers/Logger.h>

namespace PyLoops::processing
{
    struct DeepLogger{};
}
namespace LoopsLib::Helpers
{
    template<>
    constexpr inline int logLevel<PyLoops::processing::DeepLogger>()
    {
        return LoopsLib::Helpers::LogLevel::Trace;
    }
}

namespace PyLoops::processing {
    class PyTrajectoryChecker{
        void computeFrechetData(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj, LoopsLib::DS::EmbeddedGraph* g,
                                LoopsLib::NT epsilon,
                                LoopsAlgs::Frechet::StrongFrechetGraphData& data);

        void getPotentialEndpoints(const LoopsAlgs::Frechet::StrongFrechetGraphData& data,
                                   std::set<LoopsLib::DS::BaseGraph::Id_t>& potentialEndpoints);

        struct EndpointChecker
        {
            const std::set<LoopsLib::DS::BaseGraph::Id_t>& potEnd;
            std::set<LoopsLib::DS::BaseGraph::Id_t> seenEnd;
            LoopsLib::DS::Heap<LoopsAlgs::Frechet::PrioQueueNode>& heap;
            int counter = 0;

            EndpointChecker(const std::set<LoopsLib::DS::BaseGraph::Id_t>& potEnd, LoopsLib::DS::Heap<LoopsAlgs::Frechet::PrioQueueNode>& heap) : 
            potEnd(potEnd),
            heap(heap)
            {
                
            }
            std::stringstream erroStream;

            LoopsAlgs::Frechet::WenkSweeplineResult beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* edge,
                                                                      const LoopsAlgs::Frechet::PrioQueueNode& node,
                                                                      const LoopsLib::Geometry::Interval& lrInterval,
                                                                      const LoopsAlgs::Frechet::StrongFrechetGraphData&
                                                                      data);

            bool anySeen() const;
        };
        public:

            std::string checkGraphData(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj,
                LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon);

            std::string checkEndspointsSeen(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj,
                                            LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon, const std::string& deepLogOutput);

            std::string checkEndpointsRandomizedStart(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj,
                LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon);
            std::string checkEndpointsFullIntervalStart(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& traj,
                LoopsLib::DS::EmbeddedGraph* g, LoopsLib::NT epsilon);

            static void registerPy(pybind11::module& mod);
    };
}
#endif