#ifndef LOOPSLIB_DS_COMPACTGRAPH_H
#define LOOPSLIB_DS_COMPACTGRAPH_H
#include <vector>
#include <LoopsLib/Helpers/Iterators.h>
#include <limits>
namespace LoopsLib::DS
{
#define STATIC_ASSERT_WITH_EXPR(expr, msg) static_assert((expr), msg #expr )
    class GraphConcept
    {
        //https://riptutorial.com/cplusplus/example/3778/void-t
        struct details {
            template<template<typename...>typename Z, class = void, typename...Ts>
            struct can_apply :
                std::false_type
            {};
            template<template<typename...>typename Z, typename...Ts>
            struct can_apply<Z, std::void_t<Z<Ts...>>, Ts...> :
                std::true_type
            {};
        };
        
        template<typename T>
        using begin_t = decltype(std::declval<T>().begin());
        template<typename T>
        using end_t = decltype(std::declval<T>().end());
        template<typename T>
        using Idx_t = typename T::Idx_t;

        template<typename T>
        using VertexHandle_t = typename T::VertexHandle;
        template<typename T>
        using EdgeHandle_t = typename T::VertexHandle;
        
        template<typename T>
        using allocateVertices_m = decltype(std::declval<T&>().allocateVertices(std::declval<Idx_t<T>>()));
        
        template<typename T>
        using allocateEdges_m = decltype(std::declval<T&>().allocateEdges(std::declval<Idx_t<T>>()));

        template<typename VH>
        using outEdges_m = decltype(std::declval<VH>()->outEdges());

        template<typename VH>
        using inEdges_m = decltype(std::declval<VH>()->inEdges());
        
        template<typename T, typename U>
        using equalityComparable_t = decltype(std::declval<const T&>() == std::declval<const U&>());

        template<typename VH, typename T>
        void verifyVertexHandle()
        {

            static_assert(details::can_apply<outEdges_m,VH>::value, "GraphConcept::VertexHandle: no outEdges() function specified for VertexHandle");
            static_assert(details::can_apply<inEdges_m, VH>::value, "GraphConcept::VertexHandle: no inEdges() function specified for VertexHandle");
        }
    public:

        template<typename T>
        void verifyFullGraph()
        {
            STATIC_ASSERT_WITH_EXPR((details::can_apply<Idx_t, T>::value), "GraphConcept: No Idx_t type specified, ");
            static_assert(details::can_apply<VertexHandle_t, T>::value, "GraphConcept: No VertexHandle type specified");
            static_assert(details::can_apply<EdgeHandle_t, T>::value, "GraphConcept: No EdgeHandle type specified");

            using VH = VertexHandle_t<T>;
            using EH = EdgeHandle_t<T>;


            // Check allocation methods
            STATIC_ASSERT_WITH_EXPR((details::can_apply<allocateVertices_m, T>::value), "GraphConcept: No allocateVertices(Idx_t) method specified, ");
            STATIC_ASSERT_WITH_EXPR((details::can_apply<allocateEdges_m, T>::value), "GraphConcept: No allocateEdges(Idx_t) method specified, ");
            // Access methods
            static_assert(std::is_same_v<decltype(std::declval<T&>().vertex(std::declval<Idx_t<T>>())),VertexHandle_t<T>>, 
                "GraphConcept: No 'vertex(Idx_t)' function returning VertexHandle specified");
            static_assert(std::is_same_v<decltype(std::declval<T&>().edge(std::declval<Idx_t<T>>())), EdgeHandle_t<T>>, 
                "GraphConcept: No 'edge(Idx_t)' function returning EdgeHandle specified");

            // Specify EdgeHandle type
            static_assert(details::can_apply<equalityComparable_t, EdgeHandle_t<T>, EdgeHandle_t<T>>::value, 
                "GraphConcept: no operator== defined between const EdgeHandle& elements");
            // Specify VertexHandle type
        }
        template<typename T>
        void verifyGraphView()
        {
            STATIC_ASSERT_WITH_EXPR((details::can_apply<Idx_t, T>::value), "GraphConcept: No Idx_t type specified, ");
            static_assert(details::can_apply<VertexHandle_t, T>::value, "GraphConcept: No VertexHandle type specified");
            static_assert(details::can_apply<EdgeHandle_t, T>::value, "GraphConcept: No EdgeHandle type specified");

            static_assert(std::is_same_v<decltype(std::declval<T&>().numberOfEdges()), Idx_t<T>>, "GraphView: No numberOfEdges() method specified returning Idx_t");
            static_assert(std::is_same_v<decltype(std::declval<T&>().numberOfVertices()), Idx_t<T>>, "GraphView: No numberOfVertices() method specified returning Idx_t");
            //Verify vertexhandle is of correct shape
            verifyVertexHandle<VertexHandle_t<T>, T>();
        }
    };

    class CompactGraph
    {
    public:
        using Idx_t = std::size_t;
    protected:
        friend class VertexHandle;
        friend class EdgeHandle;

        struct Vertex
        {
            // Outgoing edges
            std::vector<Idx_t> m_outEdges;
            // Incoming edges
            std::vector<Idx_t> m_inEdges;
        };
        struct Edge
        {
            // Source vertex
            Idx_t srcVertex = std::numeric_limits<std::size_t>::max();
            // Sink vertex
            Idx_t sinkVertex = std::numeric_limits<std::size_t>::max();
        };
        // Vertices in the graph
        std::vector<Vertex> m_vertices;
        // Edges in the graph
        std::vector<Edge> m_edges;
    public:
        static inline auto INVALID_IDX = std::numeric_limits<Idx_t>::max();
        class EdgeHandle;

        /**
         * \brief Functor to convert edge ID to handle
         */
        class IdToEdgeHandle
        {
            CompactGraph& m_graph;
        public:
            IdToEdgeHandle(CompactGraph& graph);

            EdgeHandle operator()(Idx_t eId) const;
        };
        class VertexHandle
        {
            friend class CompactGraph;
            CompactGraph& m_parent;
            Idx_t m_vertexId;
            VertexHandle(CompactGraph& parent, Idx_t id);
        public:
            // To edge handle iterator for transforming internal representation to edge handles
            using ToEHIt = Helpers::Iterators::transform_iterator<std::vector<Idx_t>, IdToEdgeHandle>;

            bool isValid() const
            {
                return m_vertexId < m_parent.m_vertices.size();
            }

            /**
             * \brief Index of the vertex in the graph
             * \return The index
             */
            Idx_t id() const;

            bool operator==(const VertexHandle& other) const;

            VertexHandle* operator->();

            EdgeHandle findOutEdge(Idx_t targetVertex);

            EdgeHandle findInEdge(Idx_t targetVertex);

            Helpers::Iterators::Iterable<ToEHIt> outEdges() const;

            Helpers::Iterators::Iterable<ToEHIt> inEdges() const;
        };
        class EdgeHandle
        {
            friend class CompactGraph;
            CompactGraph& m_parent;
            Idx_t m_edgeId;
            EdgeHandle(CompactGraph& parent, Idx_t id);
        public:
            bool isValid() const
            {
                return m_edgeId < m_parent.m_edges.size();
            }
            Idx_t id() const;

            bool operator==(const EdgeHandle& other) const;

            EdgeHandle* operator->();

            VertexHandle sourceVertex() const;

            VertexHandle sinkVertex() const;
        };


        CompactGraph();
        CompactGraph(Idx_t numberOfVertices);

        bool isEdgeValid(Idx_t eId) const;

        void allocateVertices(Idx_t numberOfVertices);

        void allocateEdges(Idx_t numberOfEdges);

        void setEdgeConnection(Idx_t edge, Idx_t src, Idx_t sink);

        VertexHandle vertex(Idx_t vId);

        EdgeHandle edge(Idx_t eId);

        std::size_t numberOfEdges() const;

        std::size_t numberOfVertices() const;
    };
}
#endif