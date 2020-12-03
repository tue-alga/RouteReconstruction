#ifndef LOOPSLIB_GRAPHALGS_GRAPHPREDICATES_H
#define LOOPSLIB_GRAPHALGS_GRAPHPREDICATES_H
#include <unordered_set>
#include <LoopsLib/Helpers/Iterators.h>

namespace LoopsLib::GraphAlgs
{
    template<typename Graph_t, typename Iterable, bool IsVertex>
    struct IsSimplePath
    {
        bool operator()(const Graph_t& graph, Iterable start, Iterable end) const
        {
            using Idx = typename Graph_t::Idx_t;
            std::unordered_set<Idx> seenVerts;
            if constexpr(IsVertex)
            {
                for(const Idx& vId: Helpers::Iterators::Iterable(start,end))
                {
                    if (seenVerts.find(vId) != seenVerts.end())return false;
                    seenVerts.insert(vId);
                }
                return true;
            }
            else
            {
                seenVerts.insert(graph.edge(*start)->sourceVertex()->id());
                for (const Idx& eId : Helpers::Iterators::Iterable(start, end))
                {
                    auto vId = graph.edge(eId)->sinkVertex()->id();
                    if (seenVerts.find(vId) != seenVerts.end())return false;
                    seenVerts.insert(vId);
                }
                return true;
            }
        }
    };
    template<typename Graph_t, typename Iterator, bool IsVertex>
    struct IsEdgeDisjoint
    {
        bool operator()(const Graph_t& graph, Iterator start, Iterator end) const
        {
            using Idx = typename Graph_t::Idx_t;
            std::unordered_set<Idx> seenEdges;
            if constexpr (IsVertex)
            {
                Iterator it = std::next(start);
                Idx prevV = *start;
                for(const Idx& vId: Helpers::Iterators::Iterable(it,end))
                {
                    auto eId= graph.vertex(prevV)->findOutEdge(vId)->id();
                    if (seenEdges.find(eId) != seenEdges.end())return false;
                    prevV = vId;
                }
                return true;
            }
            else
            {
                for (const Idx& eId : Helpers::Iterators::Iterable(start, end))
                {
                    if (seenEdges.find(eId) != seenEdges.end())return false;
                    seenEdges.insert(eId);
                }
                return true;
            }
        }
    };
}
#endif