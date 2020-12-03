#ifndef LOOPSLIB_DS_BOOSTINTERFACE_H
#define LOOPSLIB_DS_BOOSTINTERFACE_H
#include "BaseGraph.h"
#include "EmbeddedGraph.h"
#include <boost/graph/graph_traits.hpp>
namespace boost
{
    template<typename BaseIt, typename ConverterFunc>
    struct ConverterIt
    {
        LOOPS_COPY_IT_TYPES_EXCEPT_VALUES(BaseIt)
        using value_type = decltype(std::declval<ConverterFunc&>()(*std::declval<BaseIt>()));
        using reference = value_type;
        using pointer = const value_type*;
        BaseIt m_it;

        ConverterFunc m_func;

        mutable value_type m_curr;
        
        ConverterIt(){}
        ConverterIt(const ConverterFunc& func):m_func(func) {}
        ConverterIt(BaseIt it, const ConverterFunc& func): m_it(it), m_func(func) {}

        reference operator*() const
        {
            m_curr = m_func(*m_it);
            return m_curr;
        }
        ConverterIt& operator++()
        {
            ++m_it;
            return *this;
        }
        ConverterIt& operator++(int i)
        {
            ++m_it;
            return *this;
        }
        difference_type operator-(const ConverterIt& other) const
        {
            return m_it - other.m_it;
        }
               
        bool operator!=(const BaseIt& other) const
        {
            return m_it != other;
        }
        bool operator!=(const ConverterIt<BaseIt,ConverterFunc>& other) const
        {
            return m_it != other.m_it;
        }
        bool operator==(const ConverterIt<BaseIt, ConverterFunc>& other) const
        {
            return m_it == other.m_it;
        }
    };

    template<typename...Tags>
    struct TagHolder
    {
        using tup = std::tuple<Tags...>;
        template<int I, typename T>
        struct IsSame
        {
            static constexpr bool value = std::is_same_v<std::tuple_element_t<I, tup>, T> || 
                std::conditional_t<I+1 < sizeof...(Tags), IsSame<I+1,T>, std::false_type>::value;
        };
        template<typename T>
        static constexpr bool HasTag = IsSame<0, T>::value;

        template<typename T, typename = std::enable_if_t<HasTag<T>,void>>
        operator T()
        {
            return T{};
        }
    };
    template<typename T, typename...Ts>
    
    auto inline extend(TagHolder<T>, TagHolder<Ts...>)
    {
        return TagHolder<T, Ts...>{};
    }
    template<typename T1, typename T2>
    using extended_tagholder = decltype(extend(T1{}, T2{}));

    namespace detail::loops
    {
        template<typename Element>
        struct ToIdConverter
        {
            using ElementBase = std::remove_reference_t<std::remove_pointer_t<std::decay_t<Element>>>;

            using value_type = std::invoke_result_t<decltype(&ElementBase::id), ElementBase>;

            value_type operator()(Element el) const
            {
                auto ret = std::invoke(&ElementBase::id, el);
                return ret;
                /*if constexpr(std::is_pointer_v<Element>){
                    return el->id();
                }
                if constexpr (!std::is_pointer_v<Element>) {
                    return el.id();
                }
                static_assert(false, "Failed to specialize");*/
            }
        };

        template<typename BeginIt, typename EndIt, typename Converter>
        inline auto make_converted_itrange(BeginIt begin, EndIt end, Converter&& converterFunc)
        {
            return std::make_pair(ConverterIt<BeginIt, Converter>(begin, converterFunc), ConverterIt<BeginIt, Converter>(end, converterFunc));
        }
    }

    // Specialize 
    template<>
    struct graph_traits<LoopsLib::DS::BaseGraph>
    {
        using vertex_descriptor = LoopsLib::DS::BaseGraph::VertexIndex;
        using edge_descriptor = LoopsLib::DS::BaseGraph::EdgeIndex;
        using directed_category = directed_tag;
        using edge_parallel_category = disallow_parallel_edge_tag;

        // Should be convertible to the right tags
        using traversal_category = TagHolder<incidence_graph_tag,edge_list_graph_tag, bidirectional_graph_tag, vertex_list_graph_tag>;
        // Incidence graph
        using degree_size_type = std::size_t;
        using OutEdgeIt = decltype(std::declval<LoopsLib::DS::BaseGraph::Vertex&>().m_outEdges.begin());
        using out_edge_iterator = ConverterIt<OutEdgeIt, detail::loops::ToIdConverter<typename OutEdgeIt::value_type>>;
        //Bidirectional graph
        using InEdgeIt = decltype(std::declval<LoopsLib::DS::BaseGraph::Vertex&>().m_inEdges.begin());
        using in_edge_iterator = ConverterIt<InEdgeIt, detail::loops::ToIdConverter<typename InEdgeIt::value_type>>;
        // EdgeListGraph
        using EdgeIt = decltype(std::declval<const LoopsLib::DS::BaseGraph&>().edges().begin());
        using edge_iterator = ConverterIt<EdgeIt, detail::loops::ToIdConverter<typename EdgeIt::value_type>>;
        using edges_size_type = std::size_t;
        // VertexListGraph
        using VertIt = decltype(std::declval<const LoopsLib::DS::BaseGraph&>().vertices().begin());
        using vertex_iterator = ConverterIt<VertIt, detail::loops::ToIdConverter<typename VertIt::value_type>>;
        using vertices_size_type = std::size_t;

        static inline vertex_descriptor null_vertex()
        {
            return -1;
        }
    };
    
    // Make embeddedgraph inherit basegraph traits. Since it is a subclass, all methods are still callable.
    template<>
    struct graph_traits<LoopsLib::DS::EmbeddedGraph> : public graph_traits<LoopsLib::DS::BaseGraph>
    {
        
    };

    // Incidence graph implementation
    inline auto out_edges(LoopsLib::DS::BaseGraph::VertexIndex vertex, const LoopsLib::DS::BaseGraph& graph)
    {
        auto* v = graph.vertex(vertex);
        using EdgeIt = decltype(v->m_inEdges.begin());
        using Edge = typename EdgeIt::value_type;
        return detail::loops::make_converted_itrange(v->m_outEdges.begin(), v->m_outEdges.end(), detail::loops::ToIdConverter<Edge>());
    }
    inline auto source(LoopsLib::DS::BaseGraph::EdgeIndex edge, const LoopsLib::DS::BaseGraph& graph)
    {
        return graph.edge(edge)->m_source->id();
    }
    inline auto target(LoopsLib::DS::BaseGraph::EdgeIndex edge, const LoopsLib::DS::BaseGraph& graph)
    {
        return graph.edge(edge)->m_sink->id();
    }
    inline auto out_degree(LoopsLib::DS::BaseGraph::VertexIndex vertex, const LoopsLib::DS::BaseGraph& graph)
    {
        return graph.vertex(vertex)->m_outEdges.size();
    }
    // BidirectionalGraph
    inline auto in_edges(LoopsLib::DS::BaseGraph::VertexIndex vertex, const LoopsLib::DS::BaseGraph& graph)
    {
        auto* v = graph.vertex(vertex);
        using EdgeIt = decltype(v->m_inEdges.begin());
        using Edge = typename EdgeIt::value_type;
        return detail::loops::make_converted_itrange(v->m_inEdges.begin(), v->m_inEdges.end(), detail::loops::ToIdConverter<Edge>());
    }
    inline auto in_degree(LoopsLib::DS::BaseGraph::VertexIndex vertex, const LoopsLib::DS::BaseGraph& graph)
    {
        return graph.vertex(vertex)->m_inEdges.size();
    }
    inline auto degree(LoopsLib::DS::BaseGraph::VertexIndex vertex, const LoopsLib::DS::BaseGraph& graph)
    {
        return in_degree(vertex, graph) + out_degree(vertex, graph);
    }
    // EdgeListGraph
    inline auto edges(const LoopsLib::DS::BaseGraph& graph)
    {
        using EdgeIt = decltype(graph.edges().begin());
        using Edge = typename EdgeIt::value_type;
        return detail::loops::make_converted_itrange(graph.edges().begin(), graph.edges().end(), detail::loops::ToIdConverter<Edge>());
    }
    inline auto num_edges(const LoopsLib::DS::BaseGraph& graph)
    {
        return graph.edges().size();
    }
    //source() and target() already defined
    // VertexListGraph
    inline auto vertices(const LoopsLib::DS::BaseGraph& graph)
    {
        using VertexIt = decltype(graph.vertices().begin());
        using Vertex = typename VertexIt::value_type;
        return detail::loops::make_converted_itrange(graph.vertices().begin(), graph.vertices().end(), detail::loops::ToIdConverter<Vertex>());
    }
    inline auto num_vertices(const LoopsLib::DS::BaseGraph& graph)
    {
        return graph.number_of_vertices();
    }

}

#endif