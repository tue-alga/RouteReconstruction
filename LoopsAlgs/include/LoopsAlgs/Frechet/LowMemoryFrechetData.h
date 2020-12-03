#ifndef HELPERS_LOWMEMORYFRECHET_H
#define HELPERS_LOWMEMORYFRECHET_H
#include <tuple>
#include "FrechetHelpers.h"

namespace LoopsAlgs::Frechet::LowMemory
{
    using Kernel = LoopsLib::MovetkGeometryKernel;
    using Segment = typename Kernel::MovetkSegment;
    using Point = typename Kernel::MovetkPoint;
    using NT = typename LoopsLib::NT;

    // A polyline, represented as a list of segments
    using Polyline = std::vector<Segment>;
    using Interval = LoopsLib::Geometry::Interval;
    using GraphView = LoopsLib::DS::GraphView;
    using Graph = LoopsLib::DS::EmbeddedGraph;

    /**
     * \brief Leftright pointer including inclusion
     */
    struct LeftRightPointers
    {
        long long minWi = -1;
        long long maxWi = -1;
    };

    // Contains left/right pointers, indexed on sink vertex of an edge.
    // Boolean records whether the rightpointer is the rightmost point possible on FD_j.
    using PointerMap = std::map<Graph::Id_t, LeftRightPointers>;

    /**
     * \brief Path pointer structure.
     */
    struct PathPointer
    {
        Graph::Id_t edge = -1;
        Interval srcInterval;
        bool wasProcessed = false;
        LoopsLib::NT lowestPointAtSelf = std::numeric_limits<LoopsLib::NT>::max();

        bool isSet() const
        {
            return edge >= 0;
        }

    };

    struct EndPointView
    {
        const std::vector<Interval>* m_intervals;
        EndPointView(const std::vector<Interval>* intervals):m_intervals(intervals){}

        struct iterator
        {
            long long m_current;
            const std::vector<Interval>* m_intervals;
            struct result
            {
                NT value = 0;
                long long interId = -1;
                bool isMax = false;
            };
            // Iterator traits
            using difference_type = long long;
            using value_type = result;
            using pointer = const result*;
            using reference = const result&;
            using iterator_category = std::random_access_iterator_tag;

            
            
            result m_currentResult;

            void updateCurrentResult()
            {
                long long el = m_current >> 1;
                m_currentResult.interId = el;
                m_currentResult.value = el & 1 ? (*m_intervals)[el].max : (*m_intervals)[el].min;
                m_currentResult.isMax = m_current & 1;
            }

            iterator(const std::vector<Interval>* intervals, std::size_t current) :m_intervals(intervals), m_current(current) {}


            iterator& operator++()
            {
                m_current++;
                updateCurrentResult();
                return *this;
            }
            iterator& operator+=(long long amount)
            {
                m_current+= amount;
                updateCurrentResult();
                return *this;
            }
            iterator& operator-=(long long amount)
            {
                m_current -= amount;
                updateCurrentResult();
                return *this;
            }
            const result& operator*() const
            {
                return m_currentResult;
            }
            const result* operator->() const
            {
                return &m_currentResult;
            }
            long long operator-(const iterator& other) const
            {
                return m_current - other.m_current;
            }
            iterator& operator--()
            {
                --m_current;
                return *this;
            }
            bool operator!=(const iterator& other) const
            {
                return m_current != other.m_current;
            }
            bool operator==(const iterator& other) const
            {
                return m_current == other.m_current;
            }
            bool operator<(const iterator& other) const
            {
                return m_current < other.m_current;
            }
            bool operator>(const iterator& other) const
            {
                return m_current > other.m_current;
            }
            bool operator<=(const iterator& other) const
            {
                return m_current <= other.m_current;
            }
            bool operator>=(const iterator& other) const
            {
                return m_current >= other.m_current;
            }
        };
        iterator begin()
        {
            return iterator(m_intervals, 0);
        }
        iterator end()
        {
            return iterator(m_intervals, m_intervals->size() * 2 );
        }
        struct IteratorLessComparator
        {
            bool operator()(NT value, const iterator::result& res) const
            {
                return value < res.value;
            }
            bool operator()(const iterator::result&  value, const iterator::result& res) const
            {
                return value.value < res.value;
            }
            bool operator()(NT value, NT res) const
            {
                return value < res;
            }
            bool operator()(const iterator::result&  value, NT res) const
            {
                return value.value < res;
            }
        };
        iterator lower_bound(NT value)
        {
            return std::lower_bound(begin(), end(), value, IteratorLessComparator());
        }
        iterator upper_bound(NT value)
        {
            return std::upper_bound(begin(), end(), value, IteratorLessComparator());
        }
    };


    /**
     * \brief Struct with strong frechet matching on graph for a single path.
     */
    struct LowMemoryFrechetGraphData
    {
        const Graph* m_graph = nullptr;
        //std::decay_t<decltype(m_graph->locations())>* m_locations = nullptr;
        // The polyline under consideration
        Polyline polyline;

        //Per vertex a list of full white intervals (possibly spanning multiple cells),
        // ordered by endpoints (they are disjoint, so well-defined).
        std::vector<std::vector<Interval>> WhiteIntervals;

        // View on the total graph that only contains vertices relevant for the given polyline.
        GraphView m_view;

        // Left and right pointers.
        // For every vertex, for every white interval.
        std::vector<std::vector<PointerMap>> lrPointers;

        LoopsLib::NT m_epsilon;

        void clear();


        /**
         * \brief Returns the range intervals for an lr pointer
         * \param targetVert The target vertex
         * \param interval The matching space interval of the lr-pointer at the vertex
         * \return The indices of the intervals intersected by the lr-pointer
         */
        std::pair<std::size_t, std::size_t> intervalRangeForLrPointer(std::size_t targetVert,
            const Interval& interval) const;

        /**
         * Get interval ID. Throws when no white interval is at the given position
         */
        std::size_t intervalForVertexLocation(std::size_t targetVert, NT pos) const;

        /**
         * \brief Returns the first white interval that contains or is above the given position.
         * Returns -1 if no such white interval exists
         * \param targetVert The target vertex
         * \param pos The position
         * \return ID of the white interval, or -1 if no such interval exists.
         */
        long long firstIntervalAboveLocation(std::size_t targetVert, NT pos) const;

        std::size_t polylineEdgeCount() const {
            return polyline.size();
        }
        /**
         * \brief Returns the number of vertices in the subgraph, as given by the graph view.
         * \return The number of vertices
         */
        std::size_t number_of_vertices() const;

        Interval& getWhiteInterval(std::size_t targetVert, const NT& lowEndpoint);

        const Interval& getWhiteInterval(std::size_t targetVert, const NT& lowEndpoint) const;

        bool isPotentialEndpoint(std::size_t vert, std::size_t interval) const;

        bool isPotentialStartPoint(std::size_t vert) const
        {
            return !WhiteIntervals[vert].empty() && WhiteIntervals[vert].front().containsApprox(0,0.00001);
        }

        /**
         * \brief Preallocates data for the matching graph 
         * \param graph The roadnetwork
         * \param lineLocations Locations of the polyline to match to
         */
        void setup(const Graph* graph, const std::vector<Kernel::MovetkPoint>& lineLocations);

        template<typename It>
        void setup(const Graph* graph, It vertBegin, It vertEnd)
        {
            std::vector<Point> locations;
            std::transform(vertBegin, vertEnd, std::back_inserter(locations), [&graph](const Graph::Id_t& vId)
            {
                return graph->locations()[vId];
            });
            setup(graph, locations);
        }

    };

    class FrechetGraphComputations
    {
        double m_fdiTimeMs = 0;
        double m_lrPointersMs = 0;
        NT m_epsilon = 0;
    public:
        FrechetGraphComputations(NT epsilon) :m_epsilon(epsilon) {}

        double fdiTimeMs() const;

        double lrPointersTimeMs() const;


        /**
         * \brief Constructs the view, stored in data.m_view, for the current assigned polyline.
         * This makes sure that not the entire graph will be taken when not absolutely necessary
         * \param data The output data object
         * \param epsilon The epsilon (max Frechet distance) to take into account
         */
        void constructFrechetView(LowMemoryFrechetGraphData& data) const;

        /**
         * \brief Computes the [0,1] intersection interval at the given point for the given segment.
         * \param seg The segment
         * \param p The point
         * \param eps The epsilon to use
         * \return A pair containing the interval and a structure flagging whether 0 and 1 are included in the interval.
         */
        static Interval computeIntersectionInterval(
            const Segment& seg, const Point& p, const NT& eps);
        static Interval computeIntersectionInterval(
            const Point& s0, const Point& s1, const Point& p, const NT& eps);

        void computeFDi(LowMemoryFrechetGraphData& data);

        void computeLeftRightPointersBF(LowMemoryFrechetGraphData& data);
    };

    // Helper SFINAE structs for determining hooks in the supplied hook object.
    namespace detail{
        template<typename T, typename = std::void_t<>>
        struct has_beforeEdgeProcess : std::false_type { };

        template<typename T>
        struct has_beforeEdgeProcess<T, std::void_t<decltype(
            std::declval<T&>().beforeEdgeProcess(
                std::declval<Graph::Edge*>(),
                std::declval<const PrioQueueNode&>(),
                std::declval<const Interval&>(),
                std::declval<const LowMemoryFrechetGraphData&>()))>
        > : std::true_type {};

        template<typename T, typename = std::void_t<>>
        struct has_onPathPointerUpdate : std::false_type { };

        template<typename T>
        struct has_onPathPointerUpdate <T, std::void_t<decltype(
            std::declval<T&>().onPathPointerUpdate(
                //Graph::Id_t targetVert, const PrioQueueNode& node, const Graph::Id_t usedEdge, int startInterval, int endInterval, const StrongFrechetGraphData& graphData
                std::declval<Graph::Id_t>(),
                std::declval<const PrioQueueNode&>(),
                std::declval<const Graph::Id_t>(),
                std::declval<int>(),
                std::declval<int>(),
                std::declval<const LowMemoryFrechetGraphData&>()
            )
            )>
        > : std::true_type{};
    }
}
#endif