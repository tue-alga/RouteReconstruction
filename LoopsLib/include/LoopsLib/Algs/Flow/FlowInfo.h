#ifndef FLOW_INFO_H
#define FLOW_INFO_H
#include <LoopsLib/Algs/Types.h>

namespace LoopsLib::Flow
{
    struct ProperFlowInfo
    {
        bool isProper = false;
        std::vector<int> violatingVertices;
    };

    template<typename GraphType, typename Scalar>
    struct FlowInfo
    {
        std::vector<Scalar>* m_flowField;
        GraphType* m_graph;

        FlowInfo(std::vector<Scalar>* flowField, GraphType* graph) :
            m_flowField(flowField),
            m_graph(graph) {}

        /**
         * \brief Checks if the given ''flow'' is actually a proper flow.
         * \param sourceVertices The designated source vertices
         * \param sinkVertices The designated sink vertices
         * \return Whether the flow is an actual proper flow.
         */
        bool isProperFlow(const std::set<int>& sourceVertices, const std::set<int>& sinkVertices)
        {
            std::vector<double> flowVals(m_graph->num_vertices(), 0.0);
            for (int i = 0; i < m_graph->num_edges(); i++)
            {
                //m_graph->
                // Find the vertices it connects to
                // Add flow for both: flow leaving the node is considered positive, incoming negative.

            }
            auto sz = flowVals.size();
            for (int i = 0; i < sz; i++)
            {
                if (!isApprox(flowVals[i], 0.0))
                {
                    if (flowVals[i] > 0 && sourceVertices.find(i) == sourceVertices.end()) return false;
                    if (flowVals[i] < 0 && sinkVertices.find(i) == sinkVertices.end()) return false;
                }
            }
            return true;
        }
    };
}

#endif