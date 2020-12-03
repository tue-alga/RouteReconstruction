#ifndef LOOPSALGS_FLOW_UNCAPTURABLEFLOW_H
#define LOOPSALGS_FLOW_UNCAPTURABLEFLOW_H
#include "LoopsLib/DS/EmbeddedGraph.h"
#include "LoopsLib/DS/GraphView.h"

namespace LoopsAlgs::Flow
{
    class UncapturableFlow
    {
    public:
        using NT = LoopsLib::MovetkGeometryKernel::NT;
        using Trajectory = std::vector<LoopsLib::MovetkGeometryKernel::MovetkPoint>;

        NT compute(const LoopsLib::DS::EmbeddedGraph& graph, const std::vector<NT>& field,
                   const std::vector<Trajectory>& representatives, NT epsilon);
    };
}
#endif