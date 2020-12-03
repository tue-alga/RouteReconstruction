#ifndef LOOPSLIB_DS_GRAPHCONCEPTS_H
#define LOOPSLIB_DS_GRAPHCONCEPTS_H
#include <vector>
#include <type_traits>
#define STATIC_ASSERT_WITH_EXPR(expr, msg) static_assert((expr), msg #expr )
namespace LoopsLib::DS
{
    namespace detail
    {
        //https://riptutorial.com/cplusplus/example/3778/void-t
        // Does not work for MSVC
        template<template<typename...>typename Z, typename = std::nullptr_t, typename...Ts>
        struct can_apply :
            std::false_type
        {};
        template<template<typename...>typename Z, typename...Ts>
        struct can_apply<Z, std::void_t<Z<Ts...>>, Ts...> :
            std::true_type
        {};

        template<typename T>
        constexpr bool is_void_v = std::is_void<T>::value;
        template<typename T>
        constexpr bool is_iterable_v = std::negation_v
            <std::disjunction_v<
                std::is_void<decltype(std::declval<T>().begin())>, 
                std::is_void<decltype(std::declval<T>().end())>
            >
        >;
    }

    template<typename T, typename...Concepts>
    struct ConceptVerifier
    {
        ConceptVerifier()
        {
            (Concepts().template verify<T>(), ...);
        }
    };

    struct VertexHandleConcept
    {
        template<typename T, typename U>
        using equalityComparable_t = decltype(std::declval<const T&>() == std::declval<const U&>());

        template<typename VH>
        using outEdges_m = decltype(std::declval<VH>()->outEdges());

        template<typename VH>
        using inEdges_m = decltype(std::declval<VH>()->inEdges());

        template<typename T>
        using begin_t = decltype(std::declval<T>().begin());
        template<typename T>
        using end_t = decltype(std::declval<T>().end());


        template<typename VH, typename Graph>
        void verify()
        {
            static_assert(!std::is_void_v<outEdges_m<VH>>, "GraphConcept::VertexHandle: no outEdges() function specified for VertexHandle");
            static_assert(!std::is_void_v<inEdges_m<VH>>, "GraphConcept::VertexHandle: no inEdges() function specified for VertexHandle");
            // Iterability of edge collections
            static_assert(!std::is_void_v<begin_t<outEdges_m<VH>>>, "VertexHandle: outEdges() not iterble");
            static_assert(!std::is_void_v<end_t<outEdges_m<VH>>>, "VertexHandle: outEdges() not iterble");
            /*static_assert(detail::can_apply<begin_t, outEdges_m<VH>>::value, "GraphConcept::VertexHandle: outEdges not iterable");
            static_assert(detail::can_apply<end_t, outEdges_m<VH>>::value, "GraphConcept::VertexHandle: outEdges not iterable");
            static_assert(detail::can_apply<begin_t, inEdges_m<VH>>::value, "GraphConcept::VertexHandle: inEdges not iterable");
            static_assert(detail::can_apply<end_t, inEdges_m<VH>>::value, "GraphConcept::VertexHandle: inEdges not iterable");*/
            //static_assert(detail::can_apply<equalityComparable_t,VH,VH>::value, "GraphConcept::VertexH"
        }
    };

    struct GraphViewConcept
    {
        template<typename T>
        using Idx_t = typename T::Idx_t;

        template<typename T>
        using VertexHandle_t = typename T::VertexHandle;
        template<typename T>
        using EdgeHandle_t = typename T::EdgeHandle;

        template<typename T, typename Idx>
        using notPresent_m = std::invoke_result_t<decltype(&T::no), T, Idx>;

        template<typename T>
        void verify()
        {
            // Verify typenames are specified
            //STATIC_ASSERT_WITH_EXPR((detail::can_apply<Idx_t, T>::value), "GraphView: No Idx_t type specified, ");
            //static_assert(detail::can_apply<VertexHandle_t, T>::value, "GraphView: No VertexHandle type specified");
            //static_assert(detail::can_apply<EdgeHandle_t, T>::value, "GraphView: No EdgeHandle type specified");

            using VH = VertexHandle_t<T>;
            using EH = EdgeHandle_t<T>;
            using Idx = Idx_t<T>;
            // Counts 
            static_assert(std::is_same_v<decltype(std::declval<const T&>().numberOfEdges()), Idx>, "GraphView: No numberOfEdges() method specified returning Idx_t");
            static_assert(std::is_same_v<decltype(std::declval<const T&>().numberOfVertices()), Idx>, "GraphView: No numberOfVertices() method specified returning Idx_t");
            
            //Verify vertexhandle is of correct shape
            VertexHandleConcept().verify<VertexHandle_t<T>,T>();

            // Access methods
            static_assert(std::is_same_v<decltype(std::declval<T&>().vertex(std::declval<Idx>())), VH>,
                "GraphView: No 'vertex(Idx_t)' function returning VertexHandle specified");
            static_assert(std::is_same_v<decltype(std::declval<T&>().edge(std::declval<Idx>())), EH>,
                "GraphView: No 'edge(Idx_t)' function returning EdgeHandle specified");
        }
    };

    struct GraphConcept
    {
        
        template<typename T, typename Idx>
        using allocateVertices_m = std::is_void<std::invoke_result_t<decltype(&T::allocateVertices),T, Idx>>;

        template<typename T, typename Idx>
        using notPresent_m = std::invoke_result_t<decltype(&T::no),T, Idx>;

        template<typename T, typename Idx>
        using allocateEdges_m = std::is_void<std::invoke_result_t<decltype(&T::allocateEdges), T, Idx>>;

        template<typename T, typename Idx>
        using setEdgeConnection_m = std::is_void<std::invoke_result_t<decltype(&T::setEdgeConnection), T, Idx,Idx,Idx>>;

        template<typename T>
        void verify()
        {
            GraphViewConcept().verify<T>();

            using VH = GraphViewConcept::VertexHandle_t<T>;
            using EH = GraphViewConcept::EdgeHandle_t<T>;
            using Idx = GraphViewConcept::Idx_t<T>;

            // Check allocation methods
            static_assert(allocateVertices_m<T,Idx>::value, "GraphConcept: No allocateVertices(Idx_t) method specified");
            static_assert(allocateEdges_m<T, Idx>::value, "GraphConcept: No allocateEdges(Idx_t) method specified, ");
            static_assert(setEdgeConnection_m<T, Idx>::value, "GraphConcept: No setEdgeConnection(Idx_t,Idx_t,Idx_t) method specified, ");
            //static_assert(!std::is_void<notPresent_m<T,Idx>>::value, "GraphConcept: NO test");
        }
    };
}
#endif