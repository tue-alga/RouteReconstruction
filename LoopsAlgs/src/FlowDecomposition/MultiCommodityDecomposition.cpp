#include <LoopsAlgs/FlowDecomposition/MultiCommodityDecomposition.h>
#include <LoopsLib/DS/BoostInterface.h>
#include "LoopsAlgs/Flow/MultiCommodityMinCostFlow.h"
#include "LoopsAlgs/Flow/DecomposeFlow.h"

void verifyFlow(LoopsLib::DS::EmbeddedGraph* graph, const std::vector<LoopsLib::NT>& flow, const std::set<long long>& srcs, const std::set<long long>& sinks, LoopsLib::NT tolerance)
{
    for(int vId = 0; vId < graph->number_of_vertices(); ++vId)
    {
        LoopsLib::NT total = 0.0;
        for(auto* e : graph->vertex(vId)->m_outEdges)
        {
            total += flow[e->id()];
        }
        for (auto* e : graph->vertex(vId)->m_inEdges)
        {
            total -= flow[e->id()];
        }
        if(srcs.find(vId) != srcs.end())
        {
            if (sinks.find(vId) != sinks.end()) continue;
            if (total < -tolerance) std::cout << "Src " << vId << " violated flow constraints with " << total << std::endl;
        }
        else if (sinks.find(vId) != sinks.end())
        {
            if (total > tolerance) std::cout << "Sink " << vId << " violated flow constraints with " << total << std::endl;
        }
        else
        {
            if (std::abs(total) > tolerance) std::cout << "Vert " << vId << " violated flow constraints with " << total << std::endl;
        }
    }
}

void LoopsAlgs::FlowDecomposition::MultiCommodityDecomposition::findNewBasisElements()
{
    Flow::MultiCommodityMinCostFlow mcf;
    std::vector<std::set<LoopsLib::DS::EmbeddedGraph::NodeIndex>> srcs;
    std::vector<std::set<LoopsLib::DS::EmbeddedGraph::NodeIndex>> sinks;
    // Compute sources and sinks
    for(const auto& repr: decompositionObject()->m_relatedInstance->m_representatives)
    {
        srcs.emplace_back();
        sinks.emplace_back();
        auto& index = graph()->getIndex();
        std::vector<long long> els;
        index.containedInDisk(repr.front().x(), repr.front().y(), decompositionObject()->m_relatedInstance->m_epsilon, els);
        std::transform(els.begin(), els.end(), std::inserter(srcs.back(),srcs.back().end()),[](auto el)
        {
            return (LoopsLib::DS::EmbeddedGraph::NodeIndex)el;
        });
        els.clear();
        index.containedInDisk(repr.back().x(), repr.back().y(), decompositionObject()->m_relatedInstance->m_epsilon, els);
        std::transform(els.begin(), els.end(), std::inserter(sinks.back(), sinks.back().end()), [](auto el)
        {
            return (LoopsLib::DS::EmbeddedGraph::NodeIndex)el;
        });
    }
    struct HasEdge
    {
        LoopsLib::DS::EmbeddedGraph* m_graph;
        std::vector<std::set<LoopsLib::DS::EmbeddedGraph::VertexIndex>> vertsPerCommodity;

        HasEdge(LoopsLib::DS::EmbeddedGraph* graph, const std::vector<LoopsLib::MovetkGeometryKernel::Trajectory>& representatives, NT epsilon):
        m_graph(graph)
        {
            vertsPerCommodity.resize(representatives.size(), {});
            for(int i = 0; i < representatives.size(); ++i)
            {
                graph->getIndex().containedInBuffer(representatives[i], epsilon, vertsPerCommodity[i]);
            }
        }
        std::size_t commodityCount() const
        {
            return vertsPerCommodity.size();
        }
        bool hasVert(int representative, LoopsLib::DS::EmbeddedGraph::VertexIndex v) const
        {
            return vertsPerCommodity[representative].find(v) != vertsPerCommodity[representative].end();
        }

        bool operator()(int representative,std::size_t edge) const
        {
            // TODO: fix edges that are not within buffer.
            auto v0 = m_graph->edge(edge)->m_source->id();
            auto v1 = m_graph->edge(edge)->m_sink->id();
            return hasVert(representative, v0) && hasVert(representative, v1);
        }
    };

    // Compute commodity flows
    HasEdge hasEdge(graph(), this->decompositionObject()->m_relatedInstance->m_representatives, this->decompositionObject()->m_relatedInstance->m_epsilon);
    std::vector<std::map<std::size_t, NT>> commodityFlow;
    mcf.computeSquaredDifferenceObjective(*graph(), srcs, sinks, field(), hasEdge.commodityCount(), hasEdge, commodityFlow);

    int perc = 10;

    // Decompose each commodity separately
    for(int i = 0; i < commodityFlow.size(); ++i)
    {
        std::vector<NT> decomposeField(field().size(),0);
        for(const auto& el: commodityFlow[i])
        {
            decomposeField[el.first] = el.second;
        }
        std::set<long long> srcIn;
        std::set<long long> sinkIn;
        std::transform(srcs[i].begin(), srcs[i].end(), std::inserter(srcIn, srcIn.end()), [](auto el) {return (long long)el; });
        std::transform(sinks[i].begin(), sinks[i].end(), std::inserter(sinkIn, sinkIn.end()), [](auto el) {return (long long)el; });
        verifyFlow(graph(), decomposeField, srcIn, sinkIn, 0.001);
        // Then decompose
        Flow::DecomposeFlow<NT, Graph_t> decomposer;
        decltype(decompositionObject()->m_basis) newBasisEls;
        decltype(decompositionObject()->m_decompositionCoeffs) newCoeffs;
        auto resid = decomposer.computeNonSimplePathsBasic(*graph(), decomposeField, srcIn, sinkIn, newBasisEls, newCoeffs);
        std::cout << "[MultiCommodityDecomposition] Non simple path decompose residual from MCF " << resid << std::endl;
        decompositionObject()->m_basis.insert(decompositionObject()->m_basis.end(), newBasisEls.begin(), newBasisEls.end());
        decompositionObject()->m_decompositionCoeffs.insert(decompositionObject()->m_decompositionCoeffs.end(), newCoeffs.begin(), newCoeffs.end());
        if(i > commodityFlow.size() / 100 * perc)
        {
            std::cout << "[MultiCommodityDecomposition] At " << perc << "%" << std::endl;
            perc += 10;
        }
    }
}

bool LoopsAlgs::FlowDecomposition::MultiCommodityDecomposition::isDone()
{
    // One shot
    return m_currentIteration == 1;
}
