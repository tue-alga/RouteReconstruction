#ifndef LOOPSLIB_DS_BOOSTMODIFICATIONGRAPH_H
#define LOOPSLIB_DS_BOOSTMODIFICATIONGRAPH_H
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <LoopsLib/Helpers/Iterators.h>
namespace LoopsLib::DS
{
    template<typename Graph_t>
    struct VertexFilter
    {
        using vertex_descriptor = typename boost::graph_traits<Graph_t>::vertex_descriptor;
        std::set<vertex_descriptor>* availableSet = nullptr;
        VertexFilter(){}
        VertexFilter(std::set<vertex_descriptor>* available):availableSet(available){}

        bool operator()(const vertex_descriptor& v) const
        {
            return availableSet->find(v) != availableSet->end();
        }
    };
    template<typename Graph_t>
    struct FilterEdgesOnVertex
    {
        using vertex_descriptor = typename boost::graph_traits<Graph_t>::vertex_descriptor;
        using edge_descriptor = typename boost::graph_traits<Graph_t>::edge_descriptor;
        std::set<vertex_descriptor>* availableSet = nullptr;
        const Graph_t* graph = nullptr;
        FilterEdgesOnVertex() {}
        FilterEdgesOnVertex(std::set<vertex_descriptor>* available, const Graph_t* graph) :
        availableSet(available),
        graph(graph){}

        bool operator()(const edge_descriptor& v) const
        {
            auto src = boost::source(v, *graph);
            auto target = boost::target(v, *graph);
            return (availableSet->find(src) != availableSet->end()) && (availableSet->find(target) != availableSet->end());
        }
    };

    template<typename Graph_t>
    struct AddOnlyModifyGraph
    {
    public:
        using vertex_descriptor = typename boost::graph_traits<Graph_t>::vertex_descriptor;
        using edge_descriptor = typename boost::graph_traits<Graph_t>::edge_descriptor;
        std::set<vertex_descriptor> m_newVertices;
        std::set<edge_descriptor> m_newEdges;

        vertex_descriptor anyVertInGraph;

        // New edges
        std::map<vertex_descriptor, std::vector<edge_descriptor>> m_outEdges;
        std::map<vertex_descriptor, std::vector<edge_descriptor>> m_inEdges;
        std::map<edge_descriptor, std::pair<vertex_descriptor, vertex_descriptor>> m_edges;

        const Graph_t& m_graphRef;

        bool isNewVertex(vertex_descriptor v) const
        {
            return m_newVertices.find(v) != m_newVertices.end();
        }
        bool isNewEdge(edge_descriptor e) const
        {
            return m_newEdges.find(e) != m_newEdges.end();
        }

        bool hasExtraOutEdges(edge_descriptor e) const
        {
            return m_outEdges.find(e) != m_outEdges.end();
        }
        bool hasExtraInEdges(edge_descriptor e) const
        {
            return m_inEdges.find(e) != m_inEdges.end();
        }
        std::size_t extraInEdgeNum(vertex_descriptor v) const
        {
            return hasExtraInEdges(v) ? m_inEdges.at(v).size() : 0;
        }

        std::size_t extraOutEdgeNum(vertex_descriptor v) const
        {
            return hasExtraOutEdges(v) ? m_outEdges.at(v).size() : 0;
        }
        auto outEdges(vertex_descriptor v) const
        {
            if (hasExtraOutEdges(v)) return std::make_pair(m_outEdges.at(v).begin(), m_outEdges.at(v).end());
            typename std::vector<edge_descriptor>::const_iterator triv;
            return std::make_pair(triv, triv);
        }
        auto inEdges(vertex_descriptor v) const
        {
            if (hasExtraInEdges(v)) return std::make_pair(m_inEdges.at(v).begin(), m_inEdges.at(v).end());
            typename std::vector<edge_descriptor>::const_iterator triv;
            return std::make_pair(triv,triv);
        }

    public:
        using wrapped_graph_edge_in_it = typename boost::graph_traits<Graph_t>::in_edge_iterator;
        using wrapped_graph_edge_out_it = typename boost::graph_traits<Graph_t>::out_edge_iterator;
        auto emptyInEdgePair() const
        {
            return std::make_pair(wrapped_graph_edge_in_it{}, wrapped_graph_edge_in_it{});
        }
        auto emptyOutEdgePair() const
        {
            return std::make_pair(wrapped_graph_edge_out_it{}, wrapped_graph_edge_out_it{});
        }

        AddOnlyModifyGraph(const Graph_t& graphRef) :m_graphRef(graphRef) {}
        const Graph_t& ref() const
        {
            return m_graphRef;
        }
    };
}
namespace boost
{

    template<typename Graph_t>
    struct graph_traits<LoopsLib::DS::AddOnlyModifyGraph<Graph_t>> : public graph_traits<Graph_t>
    {
        // Add mutable graph stuff

        using parent_gt = boost::graph_traits<Graph_t>;

        using out_edge_iterator = LoopsLib::Helpers::Iterators::consecutive_iterator<typename parent_gt::out_edge_iterator, typename std::vector<typename parent_gt::edge_descriptor>::const_iterator>;
        using in_edge_iterator = LoopsLib::Helpers::Iterators::consecutive_iterator<typename parent_gt::in_edge_iterator, typename std::vector<typename parent_gt::edge_descriptor>::const_iterator>;
        using edge_iterator = LoopsLib::Helpers::Iterators::consecutive_iterator<typename parent_gt::edge_iterator, typename std::vector<typename parent_gt::edge_descriptor>::const_iterator>;
        using vertex_iterator = LoopsLib::Helpers::Iterators::consecutive_iterator<typename parent_gt::vertex_iterator, typename std::vector<typename parent_gt::vertex_descriptor>::const_iterator>;
    };

    // Incidence graph implementation
    template<typename Graph_t>
    inline auto out_edges(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor vertex, const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        typename boost::graph_traits<LoopsLib::DS::AddOnlyModifyGraph<Graph_t>>::out_edge_iterator it(
            graph.isNewVertex(vertex) ? graph.emptyOutEdgePair() : boost::out_edges(vertex,graph.m_graphRef),
            graph.outEdges(vertex)
        );
        return std::make_pair(it, it.end());
    }
    template<typename Graph_t>
    inline auto source(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::edge_descriptor edge, const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        if (graph.isNewEdge(edge)) return graph.m_edges.at(edge).first;
        return boost::source(edge, graph.m_graphRef);
    }
    template<typename Graph_t>
    inline auto target(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::edge_descriptor edge, const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        if (graph.isNewEdge(edge)) return graph.m_edges.at(edge).second;
        return boost::target(edge, graph.m_graphRef);
    }
    template<typename Graph_t>
    inline auto out_degree(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor vertex, const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        return (!graph.isNewVertex(vertex) ? boost::out_degree(vertex, graph.m_graphRef) : 0 ) + graph.extraOutEdgeNum(vertex);
    }
    // BidirectionalGraph
    template<typename Graph_t>
    inline auto in_edges(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor vertex, const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        LoopsLib::Helpers::Iterators::consecutive_iterator it(
            graph.isNewVertex(vertex) ? graph.emptyInEdgePair() : boost::in_edges(vertex,graph.m_graphRef),
            graph.inEdges(vertex)
        );
        return std::make_pair(it, it.end());
    }
    template<typename Graph_t>
    inline auto in_degree(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor vertex, const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        return (!graph.isNewVertex(vertex) ? boost::in_degree(vertex, graph.m_graphRef) : 0) + graph.extraInEdgeNum(vertex);
    }
    template<typename Graph_t>
    inline auto degree(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor vertex, const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        return in_degree(vertex, graph) + out_degree(vertex, graph);
    }
    // EdgeListGraph
    template<typename Graph_t>
    inline auto edges(const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        using It = typename boost::graph_traits<LoopsLib::DS::AddOnlyModifyGraph<Graph_t>>::edge_iterator;
        It it(boost::edges(graph.m_graphRef), std::make_pair(graph.m_newEdges.begin(), graph.m_newEdges.end()));
        return std::make_pair(it, it.end());
    }
    template<typename Graph_t>
    inline auto num_edges(const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        return boost::num_edges(graph.m_graphRef) + graph.m_newEdges.size();
    }
    //source() and target() already defined
    // VertexListGraph
    template<typename Graph_t>
    inline auto vertices(const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        using It = typename boost::graph_traits<LoopsLib::DS::AddOnlyModifyGraph<Graph_t>>::vertex_iterator;
        It it(boost::vertices(graph.m_graphRef), std::make_pair(graph.m_newVertices.begin(), graph.m_newVertices.end()));
        return std::make_pair(it, it.end());
    }
    template<typename Graph_t>
    inline auto num_vertices(const LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& graph)
    {
        return boost::num_vertices(graph.m_graphRef) + graph.m_newVertices.size();
    }
    // Mutable graph
    template<typename Graph_t>
    inline auto add_edge(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor u, typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor v, 
        LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::edge_descriptor e = boost::num_edges(g.m_graphRef) + g.m_newEdges.size();
        g.m_newEdges.insert(e);
        g.m_outEdges[u].push_back(e);
        g.m_inEdges[v].push_back(e);
        g.m_edges[e] = std::make_pair(u, v);
        return e;
    }
    template<typename Graph_t>
    inline auto remove_edge(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor u, typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor v,
        LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }
    template<typename Graph_t>
    inline auto remove_edge(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::edge_descriptor e, LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }
    template<typename Graph_t>
    inline auto remove_edge(typename boost::graph_traits<LoopsLib::DS::AddOnlyModifyGraph<Graph_t>>::edge_iterator eIt, LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }
    template<typename Graph_t, typename Pred>
    inline auto remove_edge_if(Pred&& p, LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }
    template<typename Graph_t, typename Pred>
    inline auto remove_out_edge_if(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor u, Pred&& p, LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }
    template<typename Graph_t, typename Pred>
    inline auto remove_in_edge_if(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor u, Pred&& p, LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }
    template<typename Graph_t>
    inline auto add_vertex(LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor v = boost::num_vertices(g.m_graphRef) + g.m_newVertices.size();
        g.m_newVertices.insert(v);
        return v;
    }
    template<typename Graph_t>
    inline auto clear_vertex(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor u, LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }
    template<typename Graph_t>
    inline auto remove_vertex(typename LoopsLib::DS::AddOnlyModifyGraph<Graph_t>::vertex_descriptor u, LoopsLib::DS::AddOnlyModifyGraph<Graph_t>& g)
    {
        static_assert("Not implemented!");
    }

}
#endif