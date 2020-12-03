#ifndef LOOPSLIB_DS_GRAPHINTERFACE_H
#define LOOPSLIB_DS_GRAPHINTERFACE_H
#include <tuple>
#include <boost/graph/graph_traits.hpp>
namespace boost::gi
{
#define GT_type(typeName) typename boost::graph_traits<Graph_t>::typeName
#define GT_typePair(typeName,typeName2) std::pair<GT_type(typeName),GT_type(typeName2)>
#define TEMPL_GRAPH_T template<typename Graph_t>
#define UNIMPL {static_assert(false,"Unimplemented");}

    // IncidenceGraph
    TEMPL_GRAPH_T GT_typePair(out_edge_iterator, out_edge_iterator) out_edges(GT_type(vertex_descriptor) v, const Graph_t& g) UNIMPL
    TEMPL_GRAPH_T GT_type(vertex_descriptor) source(GT_type(edge_descriptor) e) UNIMPL
    TEMPL_GRAPH_T GT_type(vertex_descriptor) target(GT_type(edge_descriptor) e) UNIMPL
    TEMPL_GRAPH_T GT_type(degree_size_type) out_degree(GT_type(vertex_descriptor) v, const Graph_t& g) UNIMPL
    // BidirectionalGraph
    TEMPL_GRAPH_T GT_typePair(in_edge_iterator, in_edge_iterator) in_edges(GT_type(vertex_descriptor) v, const Graph_t& g) UNIMPL
    TEMPL_GRAPH_T GT_type(degree_size_type) in_degree(GT_type(vertex_descriptor) v, const Graph_t& g) UNIMPL
    TEMPL_GRAPH_T GT_type(degree_size_type) degree(GT_type(vertex_descriptor) v, const Graph_t& g) UNIMPL
    // AdjacencyGraph



#undef GT_type(typeName)
#undef TEMPL_GRAPH_T
#undef UNIMPL
}
#endif
