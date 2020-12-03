#ifndef LOOPSALGS_FLOW_DECOMPOSEFLOW_H
#define LOOPSALGS_FLOW_DECOMPOSEFLOW_H
#include "LoopsLib/DS/GraphView.h"
#include <LoopsLib/DS/BoostModificationGraph.h>
#include <boost/graph/graph_traits.hpp>
#include "LoopsLib/GraphAlgs/WidestPath.h"
#include "LoopsLib/GraphAlgs/CycleFinder.h"
#include <boost/graph/breadth_first_search.hpp>
#include <utility>

namespace LoopsAlgs::Flow
{
    namespace detail
    {
        template<typename...Iterables>
        struct Zip
        {
            template<typename Iterable>
            using It = decltype(std::declval<const Iterable&>().begin());
            using ItTuple = std::tuple<It<Iterables>...>;
            using IndexSeq = std::index_sequence_for<Iterables...>;
            ItTuple its;
            ItTuple ends;
            Zip()
            {
                
            }
            Zip(const Iterables&...inIts):
            its(std::make_tuple(inIts.begin()...)),
            ends(std::make_tuple(inIts.end()...)) {}

            class iterator
            {
                friend struct Zip;
                typename Zip::ItTuple its;

                iterator(typename Zip::ItTuple its) : its(its){}

                template<std::size_t...Is>
                void upOne(std::index_sequence<Is...>)
                {
                    (++std::get<Is>(its), ...);
                }
            public:

                iterator& operator++()
                {
                    upOne(IndexSeq{});
                    return *this;
                }
                bool operator!=(const iterator& other) const
                {
                    return its != other.its;
                }
                bool operator==(const iterator& other) const
                {
                    return its == other.its;
                }
                ItTuple operator*() const
                {
                    return its;
                }
            };

            iterator begin()
            {
                return iterator(its);
            }
            iterator end()
            {
                return iterator(ends);
            }
        };
        template<typename...Iterables>
        auto zip(const Iterables&...its)
        {
            return Zip < Iterables...>(its...);
        }
    }
    template<typename NT, typename Graph_t>
    class DecomposeFlow
    {
    private:
        NT m_zeroThreshold = 1e-8;
        // Maximum number of wraps around cycle allowed.
        int m_wrapsThreshold = 20;
    public:
        using GTraits = typename boost::graph_traits<Graph_t>;
        using edge_descriptor = typename boost::graph_traits<Graph_t>::edge_descriptor;
        using vertex_descriptor = typename boost::graph_traits<Graph_t>::vertex_descriptor;
        using EdgePath = std::vector<edge_descriptor>;

        void demandSupplyFromFlow(const Graph_t& graph, const std::vector<NT>& flow, const std::set<vertex_descriptor>& sources,
            const std::set<vertex_descriptor>& sinks, std::unordered_map<vertex_descriptor, NT>& sourceSupply,
            std::unordered_map<vertex_descriptor, NT>& sinkDemand)
        {
            using namespace LoopsLib::Helpers;
            for (const auto& v : sources)
            {
                if (sinks.find(v) != sinks.end())
                {
                    NT total = 0;
                    for (const auto& e : Iterators::PairIterable(boost::out_edges(v, graph)))
                    {
                        total += flow[e];
                    }
                    for (const auto& e : Iterators::PairIterable(boost::in_edges(v, graph)))
                    {
                        total -= flow[e];
                    }
                    if (total > 0)
                    {
                        sourceSupply[v] = total;
                        sinkDemand[v] = 0;
                    }
                    else
                    {
                        sourceSupply[v] = 0;
                        sinkDemand[v] = -total;
                    }
                }
                else
                {
                    NT total = 0;
                    for (const auto& e : Iterators::PairIterable(boost::out_edges(v, graph)))
                    {
                        total += flow[e];
                    }
                    for (const auto& e : Iterators::PairIterable(boost::in_edges(v, graph)))
                    {
                        total -= flow[e];
                    }
                    sourceSupply[v] = total;
                }
            }
            for (const auto& v : sinks)
            {
                if (sources.find(v) != sources.end()) continue;
                NT total = 0;
                for (const auto& e : Iterators::PairIterable(boost::in_edges(v, graph)))
                {
                    total += flow[e];
                }
                for (const auto& e : Iterators::PairIterable(boost::out_edges(v, graph)))
                {
                    total -= flow[e];
                }
                // Should be positive
                sinkDemand[v] = total;
            }
        }

        NT computeSimplePaths(  const Graph_t& graph, const std::vector<NT>& flow,
                                const std::set<vertex_descriptor>& sources,
                                const std::set<vertex_descriptor>& sinks,
                                const std::unordered_map<vertex_descriptor, NT>& sourceSupply,
                                const std::unordered_map<vertex_descriptor, NT>& sinkDemand,
                                std::vector<EdgePath>& paths, std::vector<NT>& coeffs
        )
        {
            std::vector<NT> residual = flow;

            // Create supersource and supersink
            LoopsLib::DS::AddOnlyModifyGraph<Graph_t> modGraph(graph);
            auto superSrc = boost::add_vertex(modGraph);
            auto superSink = boost::add_vertex(modGraph);
            for (auto src : sources)
            {
                boost::add_edge(superSrc, src, modGraph);
                residual.push_back(sourceSupply.at(src)); // Make sure there is some weight for the added edges, but so they do not participate 
            }
            for (auto sink : sinks)
            {
                boost::add_edge(sink, superSink, modGraph);
                residual.push_back(sinkDemand.at(sink));
            }
            // Referenced residual since we are updating it continuously 
            LoopsLib::GraphAlgs::EdgeToWeightMapper<Graph_t, std::vector<NT>&> mapper(residual);
            LoopsLib::GraphAlgs::WidestPath<std::decay_t<decltype(modGraph)>, decltype(mapper)> wp;
            while (true)
            {
                NT outValue;
                std::vector<typename decltype(wp)::Edge_t> outPath;
                wp.compute(modGraph, mapper, superSrc, superSink, outPath, outValue);
                if (outPath.empty()) {
                    std::cout << "[DecomposeFlow] No more widest paths available" << std::endl;
                    break;
                }
                if (outValue <= m_zeroThreshold)
                {
                    std::cout << "[DecomposeFlow] Reached threshold in simple paths finding " << std::endl;
                    break;
                }
                paths.push_back({});
                // Remove virtual first and last edge
                std::transform(std::next(outPath.begin()), std::prev(outPath.end()), std::back_inserter(paths.back()), [](auto e) {return e; });
                coeffs.push_back(outValue);
                // Update residual
                for (const auto& el : outPath)
                {
                    residual[el] -= outValue;
                }
            }
            return std::accumulate(residual.begin(), residual.end(), (NT)0.0);
        }

        /**
         * \brief Returns the simple paths in the regular flow decomposition algorithm. Ignores all potential cycle flows (=circulation)
         * \param graph The graph to use
         * \param flow The flow to decompose
         * \param sources Source vertices
         * \param sinks Sink vertices.
         * \param paths Output flow paths
         * \param coeffs Coefficients associated to each flow path
         * \return Residual total flow
         */
        NT computeSimplePaths(const Graph_t& graph, const std::vector<NT>& flow,
                              const std::set<vertex_descriptor>& sources,
                              const std::set<vertex_descriptor>& sinks, std::vector<EdgePath>& paths, std::vector<NT>& coeffs)
        {
            std::unordered_map<vertex_descriptor, NT> sourceSupply, sinkDemand;
            using namespace LoopsLib::Helpers;
            demandSupplyFromFlow(graph, flow, sources, sinks, sourceSupply, sinkDemand);
            return computeSimplePaths(graph, flow, sources, sinks, sourceSupply, sinkDemand, paths, coeffs);
        }

        NT computeNonSimplePathsBasic(const Graph_t& graph, const std::vector<NT>& flow,
            const std::set<vertex_descriptor>& sources,
            const std::set<vertex_descriptor>& sinks, std::vector<std::vector<edge_descriptor>>&paths, std::vector<NT>& coeffs)
        {
            using namespace LoopsLib::Helpers;

            std::unordered_map<vertex_descriptor, std::set<std::size_t>> pathToVertexMap;
            auto addPathToVert = [&pathToVertexMap](const vertex_descriptor& v, const std::size_t& pId)
            {
                if (pathToVertexMap.find(v) == pathToVertexMap.end()) pathToVertexMap[v] = {};
                pathToVertexMap[v].insert(pId);
            };
            auto removePathFromVert = [&pathToVertexMap](const vertex_descriptor& v, const std::size_t& pId)
            {
                if (pathToVertexMap.find(v) == pathToVertexMap.end()) return;
                pathToVertexMap[v].erase(pId);
            };
            std::vector<std::vector<edge_descriptor>> cycles;
            std::vector<NT> cycleCoeffs;
            // Should be close to zero if the flow was an actual flow.
            NT resid = computePathsCycles(graph, flow, sources, sinks, paths, coeffs, cycles, cycleCoeffs);

            // Verify positivity of coefficients
            for(const auto& coeff: coeffs)
            {
                if (coeff < 0) throw std::runtime_error("[DecomposeFlow::computeNonSimplePathsBasic] Negative path coefficient");
            }
            for (const auto& coeff : cycleCoeffs)
            {
                if (coeff < 0) throw std::runtime_error("[DecomposeFlow::computeNonSimplePathsBasic] Negative cycle coefficient");
            }

            const std::size_t originalSimplePathsNum = paths.size();
            // Try attaching stuff
            for (std::size_t i = 0; i < originalSimplePathsNum; ++i)
            {
                if (paths[i].empty())
                {
                    std::cout << "Empty path?" << std::endl;
                    continue;
                }
                auto vStart = boost::source(paths[i].front(), graph);
                addPathToVert(vStart, i);
                for (const auto& e : paths[i])
                {
                    addPathToVert(boost::target(e, graph), i);
                }
            }
            NT perc = 0.1;
            for (std::size_t c = 0; c < cycles.size(); ++c)
            {
                // Try to greedily attach to a path
                const auto cycleCoeff = cycleCoeffs[c];
                // Check where a vertex is part of a path
                std::size_t cE = 0;

                // Ignore the cycle when the coefficient is too small
                if (cycleCoeff < m_zeroThreshold) continue;

                if(c > perc * cycles.size())
                {
                    perc += 0.1;
                    std::cout << "[DecomposeFlow] Cycles at " << perc * 100 << " percent" << std::endl;
                }

                for (; cE < cycles[c].size(); ++cE)
                {
                    const auto targetV = boost::source(cycles[c][cE], graph);
                    // No connecting path found for this vertex.
                    if (pathToVertexMap.find(targetV) == pathToVertexMap.end()) continue;

                    // Determine the amount of times to wrap around the cycle to capture it.
                    int wraps = std::numeric_limits<int>::max(); // Makes sure to ignore the cycle when no connecting path was found
                    NT coeff = 0;
                    bool higherThanPath = false;
                    std::size_t pathId = 0;
                    NT pathCoeff = 0;
                    for (const auto& pId : pathToVertexMap[targetV])
                    {
                        // Determine coefficient for the new non-simple path
                        pathCoeff = coeffs[pathId];
                        if (pathCoeff < m_zeroThreshold) continue;

                        if (cycleCoeff > pathCoeff)
                        {
                            const auto frac = std::ceil(cycleCoeff / pathCoeff);
                            int currWraps = (int)frac;
                            if (currWraps >= wraps) continue;
                            wraps = currWraps;
                            coeff = cycleCoeff / (NT)wraps;
                            higherThanPath = true;
                            pathId = pId;
                        }
                        else
                        {
                            wraps = 1;
                            coeff = cycleCoeff;
                            pathId = pId;
                            break;
                        }
                    }
                    // Determine coefficient for the new non-simple path
                    pathCoeff = coeffs[pathId];
                    if (coeff > pathCoeff) continue;

                    // Put a cap on the number of allowed wraps
                    if(wraps > m_wrapsThreshold || wraps < 0)
                    {
                        continue;
                    }
                    //std::cout << "Connecting with " << wraps << " wraps of cycle " << cycles[c].size() << " to path " << paths[pathId].size() << std::endl;
                    if(pathCoeff - coeff < 0 || coeff < 0)
                    {
                        std::cout << "Somehow determined to go negative, coeff is " << coeff << ", path coeff " << pathCoeff << std::endl;
                        continue;
                    }



                    // Find insertion location in path and create the non-simple path
                    paths.emplace_back(std::vector<edge_descriptor>{});
                    // The path to attach to
                    const auto &path = paths[pathId];

                    std::vector<edge_descriptor>& nonSimplePath = paths.back();
                    if (targetV == boost::source(paths[pathId].front(), graph))
                    {
                        Iterators::circular_it<std::vector<edge_descriptor>> start(cycles[c], cycles[c].begin() + cE, 0);
                        Iterators::circular_it<std::vector<edge_descriptor>> end(cycles[c], cycles[c].begin() + cE, wraps);
                        std::copy(start, end, std::back_inserter(nonSimplePath));
                        std::copy(path.begin(), path.end(), std::back_inserter(nonSimplePath));
                    }
                    else
                    {
                        for (std::size_t i = 0; i < path.size(); ++i)
                        {
                            const auto& e = path[i];
                            nonSimplePath.push_back(e);
                            if (boost::target(e, graph) == targetV)
                            {
                                Iterators::circular_it<std::vector<edge_descriptor>> start(cycles[c], cycles[c].begin() + cE, 0);
                                Iterators::circular_it<std::vector<edge_descriptor>> end(cycles[c], cycles[c].begin() + cE, wraps);
                                std::copy(start, end, std::back_inserter(nonSimplePath));
                                std::copy(path.begin() + i + 1, path.end(), std::back_inserter(nonSimplePath));
                                break;
                            }
                        }
                    }
                    // Update coefficients 
                    coeffs.push_back(coeff);
                    coeffs[pathId] -= coeff;
                    if(coeff < 0 || coeffs[pathId] < 0 )
                    {
                        throw std::runtime_error("Somehow determined to go negative, coeff is " + std::to_string(coeff) + ", path coeff " + std::to_string(pathCoeff));
                    }

                    // Path was reassigned to the non-simple path. Make sure we do not 
                    // connect other cycles to this path that now has coefficient 0.
                    if (higherThanPath)
                    {
                        auto vStart = boost::source(nonSimplePath.front(), graph);
                        removePathFromVert(vStart, pathId);
                        for (const auto& e : path)
                        {
                            removePathFromVert(boost::target(e, graph), pathId);
                        }
                    }

                    const auto newIndex = paths.size() - 1;
                    // Add new path to map
                    {
                        auto vStart = boost::source(nonSimplePath.front(), graph);
                        addPathToVert(vStart, newIndex);
                        for (const auto& e : nonSimplePath)
                        {
                            addPathToVert(boost::target(e, graph), newIndex);
                        }
                    }

                    // Done with this cycle
                    break;
                }
                // Update non-decomposed cycles.
                if (cE == cycles[c].size())
                {
                    resid += cycleCoeff * cycles[c].size();
                }
            }
            return resid;
        }

        //TODO !!!
        //NT computeNonSimplePaths(const Graph_t& graph, const std::vector<NT>& flow,
        //    const std::set<vertex_descriptor>& sources,
        //    const std::set<vertex_descriptor>& sinks, std::vector<std::vector<edge_descriptor>>&paths, std::vector<NT>& coeffs)
        //{
        //    using namespace LoopsLib::Helpers;

        //    std::unordered_map<vertex_descriptor, std::set<std::size_t>> pathToVertexMap;
        //    auto addPathToVert = [&pathToVertexMap](const vertex_descriptor& v, const std::size_t& pId)
        //    {
        //        if (pathToVertexMap.find(v) == pathToVertexMap.end()) pathToVertexMap[v] = {};
        //        pathToVertexMap[v].insert(pId);
        //    };
        //    auto removePathFromVert = [&pathToVertexMap](const vertex_descriptor& v, const std::size_t& pId)
        //    {
        //        if (pathToVertexMap.find(v) == pathToVertexMap.end()) return;
        //        pathToVertexMap[v].erase(pId);
        //    };
        //    std::vector<std::vector<edge_descriptor>> cycles;
        //    std::vector<NT> cycleCoeffs;
        //    // Should be close to zero if the flow was an actual flow.
        //    NT resid = computePathsCycles(graph, flow, sources, sinks, paths, coeffs, cycles, cycleCoeffs);

        //    const std::size_t originalSimplePathsNum = paths.size();
        //    // Try attaching stuff
        //    for (std::size_t i = 0; i < originalSimplePathsNum; ++i)
        //    {
        //        if (paths[i].empty())
        //        {
        //            std::cout << "Empty path?" << std::endl;
        //            continue;
        //        }
        //        auto vStart = boost::source(paths[i].front(), graph);
        //        addPathToVert(vStart, i);
        //        for (const auto& e : paths[i])
        //        {
        //            addPathToVert(boost::target(e, graph), i);
        //        }
        //    }

        //    boost::adjacency_list<boost::setS, boost::vecS, boost::undirected_tag, boost::vertex_index_t> connectGraph(paths.size() + cycles.size());
        //    for (std::size_t i = 0; i < cycles.size(); ++i)
        //    {
        //        const auto cycleId = i + paths.size();
        //        if (cycles[i].empty())
        //        {
        //            std::cout << "Empty path?" << std::endl;
        //            continue;
        //        }
        //        for (const auto& e : cycles[i])
        //        {
        //            const auto v = boost::target(e, graph);
        //            if(pathToVertexMap.find(v) != pathToVertexMap.end())
        //            {
        //                for(const auto& el: pathToVertexMap[v])
        //                {
        //                    boost::add_edge(el, cycleId, connectGraph);
        //                }
        //            }
        //            addPathToVert(v, cycleId);
        //        }
        //    }

        //    //TODO setup visitor that connects elements.
        //    //boost::bfs_visitor<>

        //    // Run BFS and connect all cycles when possible.
        //    for(auto i = 0; i < paths.size(); ++i)
        //    {

        //        // Connect cycles whenever they can be attached to a path
        //        boost::breadth_first_search(connectGraph, i, boost::visitor(connectVisitor));
        //    }

        //    for (std::size_t c = 0; c < cycles.size(); ++c)
        //    {
        //        // Try to greedily attach to a path
        //        const auto ownCoeff = cycleCoeffs[c];
        //        // Check where a vertex is part of a path
        //        std::size_t cE = 0;
        //        for (; cE < cycles[c].size(); ++cE)
        //        {
        //            const auto targetV = boost::source(cycles[c][cE], graph);
        //            // No connecting path found for this vertex.
        //            if (pathToVertexMap.find(targetV) == pathToVertexMap.end()) continue;

        //            // Determine the amount of times to wrap around the cycle to capture it.
        //            int wraps = std::numeric_limits<int>::max();
        //            NT coeff = 0;
        //            bool higherThanPath = false;
        //            std::size_t pathId = 0;
        //            for(const auto& pId : pathToVertexMap[targetV])
        //            {
        //                // Determine coefficient for the new non-simple path
        //                const auto pathCoeff = coeffs[pathId];
        //                if (ownCoeff > pathCoeff)
        //                {
        //                    const auto frac = std::ceil(ownCoeff / pathCoeff);
        //                    int currWraps = (int)frac;
        //                    if (currWraps >= wraps) continue;
        //                    wraps = currWraps;
        //                    coeff = pathCoeff;
        //                    higherThanPath = true;
        //                    pathId = pId;
        //                }
        //                else
        //                {
        //                    wraps = 1;
        //                    coeff = ownCoeff;
        //                    pathId = pId;
        //                    break;
        //                }
        //            }

        //            
        //            // Determine coefficient for the new non-simple path
        //            const auto pathCoeff = coeffs[pathId];

        //            
        //            // Find insertion location in path and create the non-simple path
        //            paths.emplace_back(std::vector<edge_descriptor>{});
        //            // The path to attach to
        //            const auto &path = paths[pathId];

        //            std::vector<edge_descriptor>& nonSimplePath = paths.back();
        //            if (targetV == boost::source(paths[pathId].front(), graph))
        //            {
        //                Iterators::circular_it<std::vector<edge_descriptor>> start(cycles[c], cycles[c].begin() + cE, 0);
        //                Iterators::circular_it<std::vector<edge_descriptor>> end(cycles[c], cycles[c].begin() + cE, wraps);
        //                std::copy(start, end, std::back_inserter(nonSimplePath));
        //                std::copy(path.begin(), path.end(), std::back_inserter(nonSimplePath));
        //            }
        //            else
        //            {
        //                for (std::size_t i = 0; i < path.size(); ++i)
        //                {
        //                    const auto& e = path[i];
        //                    nonSimplePath.push_back(e);
        //                    if (boost::target(e, graph) == targetV)
        //                    {
        //                        Iterators::circular_it<std::vector<edge_descriptor>> start(cycles[c], cycles[c].begin() + cE, 0);
        //                        Iterators::circular_it<std::vector<edge_descriptor>> end(cycles[c], cycles[c].begin() + cE, wraps);
        //                        std::copy(start, end, std::back_inserter(nonSimplePath));
        //                        std::copy(path.begin() + i + 1, path.end(), std::back_inserter(nonSimplePath));
        //                        break;
        //                    }
        //                }
        //            }
        //            // Update coefficients 
        //            coeffs.push_back(coeff);
        //            coeffs[pathId] -= coeff;

        //            // Path was reassigned to the non-simple path. Make sure we do not 
        //            // connect other cycles to this path that now has coefficient 0.
        //            if(higherThanPath)
        //            {
        //                auto vStart = boost::source(nonSimplePath.front(), graph);
        //                removePathFromVert(vStart, pathId);
        //                for(const auto& e : path)
        //                {
        //                    removePathFromVert(boost::target(e, graph), pathId);
        //                }
        //            }

        //            const auto newIndex = paths.size() - 1;
        //            // Add new path to map
        //            {
        //                auto vStart = boost::source(nonSimplePath.front(), graph);
        //                addPathToVert(vStart, newIndex);
        //                for (const auto& e : nonSimplePath)
        //                {
        //                    addPathToVert(boost::target(e, graph), newIndex);
        //                }
        //            }

        //            // Done with this cycle
        //            break;
        //        }
        //        // Update non-decomposed cycles.
        //        if(cE == cycles[c].size())
        //        {
        //            resid += ownCoeff * cycles[c].size();
        //        }
        //    }
        //    return resid;
        //}

        NT computePathsCycles(const Graph_t& graph, const std::vector<NT>& flow,
            const std::set<vertex_descriptor>& sources,
            const std::set<vertex_descriptor>& sinks,
            std::vector<EdgePath>&paths, std::vector<NT>& pathCoeffs,
            std::vector<EdgePath>&cycles, std::vector<NT>& cycleCoeffs)
        {
            // Determine simple paths and compute residual
            (void)computeSimplePaths(graph, flow, sources, sinks, paths, pathCoeffs);
            std::vector<NT> residual = flow;
            for (auto i = 0; i < paths.size(); ++i)
            {
                const auto coeff = pathCoeffs[i];
                for (const auto& e : paths[i])
                {
                    residual[e] -= coeff;
                }
            }
            auto totalResid = std::accumulate(residual.begin(), residual.end(), (NT)0.0);
            std::cout << "[DecomposeFlow] Residual after paths: " << totalResid << std::endl;

            if(totalResid < m_zeroThreshold)
            {
                std::cout << "[DecomposeFlow] Skipping cycles due to threshold" << std::endl;
                return totalResid;
            }
            // Find cycles in the residual graph.
            LoopsLib::GraphAlgs::FindCycles<Graph_t> cycleFinder;
            cycleFinder.setVerifyCirculation(true);
            cycleFinder.setVerifyCycle(true);
            cycleFinder.cyclesFromCirculation(graph, residual, (NT)1e-8, cycles, cycleCoeffs);

            for(const auto& els: detail::zip(cycles, cycleCoeffs))
            {
                for(const auto& e: *std::get<0>(els))
                {
                    residual[e] -= *std::get<1>(els);
                }
            }
            NT newResid = std::accumulate(residual.begin(), residual.end(), (NT)0.0);
            std::cout << "[DecomposeFlow] Residual after cycles: " << newResid << std::endl;
            return newResid;
        }
    };
}
#endif