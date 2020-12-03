#include <LoopsAlgs/Flow/DecomposeFlow.h>
#include <LoopsLib/Helpers/Iterators.h>
#include <LoopsLib/DS/BoostInterface.h>
#include "LoopsLib/GraphAlgs/WidestPath.h"
#include "LoopsLib/DS/GraphHelpers/GraphModificationGuard.h"
#include "LoopsLib/GraphAlgs/CycleFinder.h"

LoopsAlgs::Flow::DecomposeFlow::NT LoopsAlgs::Flow::DecomposeFlow::computeSimplePaths(
    LoopsLib::DS::EmbeddedGraph& graph, const std::vector<NT>& flow,
    const std::set< LoopsLib::DS::EmbeddedGraph::NodeIndex>& sources,
    const std::set< LoopsLib::DS::EmbeddedGraph::NodeIndex>& sinks,
    std::vector<std::vector<LoopsLib::DS::EmbeddedGraph::EdgeIndex>>& paths, std::vector<NT>& coeffs)
{
    using Graph_t = LoopsLib::DS::EmbeddedGraph;
    std::vector<NT> residual = flow;

    // Create supersource and supersink
    LoopsLib::DS::GraphHelpers::GraphModificationGuard g(&graph);
    auto* superSrc = g.addVertex();
    auto* superSink = g.addVertex();
    for(auto src: sources)
    {
        g.addEdge(superSrc, graph.vertex(src));
    }
    for (auto sink : sinks)
    {
        g.addEdge(graph.vertex(sink), superSink);
    }
    // Referenced residual since we are updating it continuously 
    LoopsLib::GraphAlgs::EdgeIdToWeightMapper<Graph_t, std::vector<NT>&> mapper(residual);
    LoopsLib::GraphAlgs::WidestPath<std::decay_t<decltype(graph)>, decltype(mapper)> wp;
    while (true)
    {
        NT outValue;
        std::vector<LoopsLib::DS::EmbeddedGraph::EdgeHandle> outPath;
        wp.compute(graph, mapper, superSrc, superSink, outPath, outValue);
        if (outPath.empty()) break;
        paths.emplace_back();
        std::transform(outPath.begin(), outPath.end(), std::back_inserter(paths.back()), [](auto* e) {return e->id(); });
        coeffs.push_back(outValue);
        // Update residual
        for(const auto& el :paths.back())
        {
            residual[el] -= outValue;
        }
    }
    return std::accumulate(residual.begin(), residual.end(), (NT)0.0);
}

LoopsAlgs::Flow::DecomposeFlow::NT LoopsAlgs::Flow::DecomposeFlow::computeNonSimplePaths(
    LoopsLib::DS::EmbeddedGraph& graph, const std::vector<NT>& flow, const std::set<NodeIndex>& sources,
    const std::set<NodeIndex>& sinks, std::vector<std::vector<EdgeIndex>>& paths, std::vector<NT>& coeffs)
{
    using namespace LoopsLib::Helpers;

    std::multimap<NodeIndex, std::size_t> pathToVertexMap;
    std::vector<std::vector<EdgeIndex>> cycles;
    std::vector<NT> cycleCoeffs;
    (void)computePathsCycles(graph, flow, sources, sinks, paths, coeffs, cycles, cycleCoeffs);

    const std::size_t originalSimplePathsNum = paths.size();
    // Try attaching stuff
    for(std::size_t i = 0; i < originalSimplePathsNum; ++i)
    {
        if(paths[i].empty())
        {
            std::cout << "Empty path?" << std::endl;
            continue;
        }
        auto vStart = boost::source(paths[i].front(), graph);
        pathToVertexMap[vStart] = i;
        for(const auto& e: paths[i])
        {
            pathToVertexMap[boost::target(e, graph)] = i;
        }
    }
    
    for(std::size_t c = 0; c < cycles.size(); ++c)
    {
        // Try to greedily attach to a path
        const auto ownCoeff = cycleCoeffs[c];
        // Check where a vertex is part of a path
        for(std::size_t cE = 0; cE < cycles[c].size(); ++cE)
        {
            const auto targetV = boost::target(cycles[c][cE], graph);
            // No connecting path found for this vertex.
            if (pathToVertexMap.find(targetV) == pathToVertexMap.end()) continue;

            auto itPair = pathToVertexMap.equal_range(targetV);

            // Get first element. Note that equal_range iterators dereference to pairs.
            const std::size_t pathId = itPair.first->first;
            const auto &path = paths[pathId];

            // Determine coefficient for the new non-simple path
            const auto pathCoeff = coeffs[pathId];

            // Determine the amount of times to wrap around the cycle to capture it.
            int wraps = -1;
            NT coeff = 0;
            if (ownCoeff > pathCoeff)
            {
                const auto frac = std::ceil(ownCoeff / pathCoeff);
                wraps = (int)wraps;
                coeff = ownCoeff / frac;
            }
            else
            {
                wraps = 1;
                coeff = ownCoeff;
            }
            // Find insertion location in path and create the non-simple path
            paths.emplace_back(std::vector<EdgeIndex>{});
            std::vector<EdgeIndex>& nonSimplePath = paths.back();
            if(targetV == boost::source(paths[pathId].front(),graph))
            {
                Iterators::circular_it<std::vector<EdgeIndex>> start(cycles[c], cycles[c].begin() + cE, 0);
                Iterators::circular_it<std::vector<EdgeIndex>> end(cycles[c], cycles[c].begin() + cE, wraps);
                std::copy(start, end, std::back_inserter(nonSimplePath));
                std::copy(path.begin(), path.end(), std::back_inserter(nonSimplePath));
            }
            else
            {
                for(std::size_t i = 0; i < path.size(); ++i)
                {
                    const auto& e = path[i];
                    nonSimplePath.push_back(e);
                    if(boost::target(e, graph) == targetV)
                    {
                        Iterators::circular_it<std::vector<EdgeIndex>> start(cycles[c], cycles[c].begin() + cE, 0);
                        Iterators::circular_it<std::vector<EdgeIndex>> end(cycles[c], cycles[c].begin() + cE, wraps);
                        std::copy(start, end, std::back_inserter(nonSimplePath));
                        std::copy(path.begin() + i + 1, path.end(), std::back_inserter(nonSimplePath));
                        break;
                    }
                }
            }
            // Update coefficients 
            coeffs.push_back(coeff);
            coeffs[pathId] -= coeff;

            const auto newIndex = paths.size() - 1;
            // Add new path to map
            {
                auto vStart = boost::source(nonSimplePath.front(), graph);
                pathToVertexMap[vStart] = newIndex;
                for (const auto& e : nonSimplePath)
                {
                    pathToVertexMap[boost::target(e, graph)] = newIndex;
                }
            }

            // Done with this cycle
            break;
        }
    }
}

LoopsAlgs::Flow::DecomposeFlow::NT LoopsAlgs::Flow::DecomposeFlow::computePathsCycles(
    LoopsLib::DS::EmbeddedGraph& graph, const std::vector<NT>& flow, const std::set<NodeIndex>& sources,
    const std::set<NodeIndex>& sinks, std::vector<std::vector<EdgeIndex>>& paths, std::vector<NT>& pathCoeffs,
    std::vector<std::vector<EdgeIndex>>& cycles, std::vector<NT>& cycleCoeffs)
{
    // Determine simple paths and compute residual
    (void)computeSimplePaths(graph, flow, sources, sinks, paths, pathCoeffs);
    std::vector<NT> residual = flow;
    for(auto i = 0; i < paths.size(); ++i)
    {
        const auto coeff = pathCoeffs[i];
        for(const auto& e: paths[i])
        {
            residual[e] -= coeff;
        }
    }
    // Find cycles in the residual graph.
    LoopsLib::GraphAlgs::FindCycles<LoopsLib::DS::EmbeddedGraph> cycleFinder;
    cycleFinder.cyclesFromCirculation(graph, residual, (NT)0.0, cycles, cycleCoeffs);
}
