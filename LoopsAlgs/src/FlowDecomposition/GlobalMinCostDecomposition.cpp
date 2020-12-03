#include <LoopsAlgs/FlowDecomposition/GlobalMinCostDecomposition.h>
#include <LoopsLib/DS/BoostInterface.h>
#include <LoopsAlgs/Flow/MinCostFlow.h>
#include "LoopsAlgs/Flow/DecomposeFlow.h"

LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::
GlobalMinCostDecomposition(DecompositionResult_t* decompObj): IFlowDecomposition(
    "GlobalMinCostDecomposition", decompObj)
{
}

LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::GlobalMinCostDecomposition(): IFlowDecomposition(
    "GlobalMinCostDecomposition")
{
}

void LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::verifyFlow(
    const std::set<typename boost::graph_traits<Graph_t>::vertex_descriptor>& sources, 
    const std::set<typename boost::graph_traits<Graph_t>::vertex_descriptor>& sinks,
    const std::vector<NT>& flow)
{
    //using namespace LoopsLib::Helpers;
    //// Add flow conservation
    //for (auto vId : Iterators::PairIterable(boost::vertices(*this->graph())))
    //{
    //    const bool isSource = sources.find(vId) != sources.end();
    //    const bool isSink = sinks.find(vId) != sinks.end();

    //    // No reasonable flow conservation property
    //    if (isSource && isSink) continue;

    //    // Ignore source/sink vertices for the commodity
    //    if (isSource)
    //    {
    //        NT outSum = 0.0;
    //        for (const auto& e : Iterators::PairIterable(boost::out_edges(vId, graph)))
    //        {
    //            outSum += flow[e];
    //        }
    //        if(outSum < 0)
    //        {
    //            
    //        }
    //    }
    //    else if (isSink)
    //    {
    //        IloExpr inSum(env);
    //        for (const auto& e : Iterators::PairIterable(boost::in_edges(vId, graph)))
    //        {
    //            inSum += flowVars[e]; //TODO: fix for arbitrary graphs
    //        }
    //        model.add(inSum >= 0);
    //    }
    //    else
    //    {
    //        // Conserve flow for non-source/sink vertices
    //        IloExpr inSum(env), outSum(env);
    //        for (const auto& e : Iterators::PairIterable(boost::in_edges(vId, graph)))
    //        {
    //            inSum += flowVars[e]; //TODO: fix for arbitrary graphs
    //        }
    //        for (const auto& e : Iterators::PairIterable((boost::out_edges(vId, graph))))
    //        {
    //            outSum += flowVars[e];
    //        }
    //        model.add(inSum == outSum);
    //    }
    //}
}

void LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::findNewBasisElements()
{
    using Graph_T = LoopsLib::DS::EmbeddedGraph;

    Flow::MinCostFlow mcf;
    using VertexIndex = typename boost::graph_traits<Graph_t>::vertex_descriptor;
    std::set<VertexIndex> sources, sinks;
    for(const auto& repr: decompositionObject()->m_relatedInstance->m_representatives)
    {
        std::vector<long long> out;
        graph()->getIndex().containedInDisk(repr[0].x(), repr[0].y(), decompositionObject()->m_relatedInstance->m_epsilon, out);
        for (const auto& v : out) sources.insert(static_cast<VertexIndex>(v));
        out.clear();
        graph()->getIndex().containedInDisk(repr.back().x(), repr.back().y(), decompositionObject()->m_relatedInstance->m_epsilon, out);
        for (const auto& v : out) sinks.insert(static_cast<VertexIndex>(v));
    }
    // First compute the flow
    std::decay_t<decltype(field())> outFlow;
    mcf.compute(*this->graph(), sources, sinks, field(), outFlow);

    std::cout << "Out flow: " << std::accumulate(outFlow.begin(), outFlow.end(), (NT)0.0) << std::endl;

    // Then decompose
    Flow::DecomposeFlow<NT,Graph_t> decomposer;
    auto resid = decomposer.computeNonSimplePathsBasic(*graph(), outFlow, sources, sinks, decompositionObject()->m_basis, decompositionObject()->m_decompositionCoeffs);
    std::cout << "Non simple path decompose residual from MCF " << resid << std::endl;

    for(const auto& el: decompositionObject()->m_decompositionCoeffs)
    {
        if (std::isnan(el)) throw std::runtime_error("Nan after non simple paths computation");
        if (std::isinf(el)) throw std::runtime_error("Inf after non simple paths computation");
        if (el < 0) throw std::runtime_error("Negative coefficeint " + std::to_string(el));
    }
}

std::map<std::string, std::string> LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::metaData() const
{
    return {};
}

void LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::setFromMetaData(
    const std::map<std::string, std::string>& metData)
{
}

std::string LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::paramDescription() const
{
    return "";
}

bool LoopsAlgs::FlowDecomposition::GlobalMinCostDecomposition::isDone()
{
    return m_currentIteration == 1;
}
