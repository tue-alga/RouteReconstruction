#ifndef HELPERS_FRECHETHELPERS_H
#define HELPERS_FRECHETHELPERS_H
#include <tuple>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Geometry/Interval.h>
#include <LoopsLib/Helpers/Timer.h>
#include <LoopsLib/DS/EmbeddedGraph.h>
#include <LoopsLib/Geometry/Index/PointIndex.h>
#include <LoopsLib/Math/Vector.h>
#include <LoopsLib/DS/GraphView.h>

namespace LoopsAlgs::Frechet
{
    struct LeftRightInclusion {
        bool leftIncluded = false;
        bool rightIncluded = false;
        LeftRightInclusion(){}
        LeftRightInclusion(bool leftIncluded, bool rightIncluded):leftIncluded(leftIncluded), rightIncluded(rightIncluded){}

    };
    using Kernel = LoopsLib::MovetkGeometryKernel;
    using Segment = typename Kernel::MovetkSegment;
    using Point = typename Kernel::MovetkPoint;
    using NT = typename LoopsLib::NT;
    // A polyline, represented as a list of segments
    using Polyline = std::vector<Segment>;
    using Interval = LoopsLib::Geometry::Interval;
    using GraphView = LoopsLib::DS::GraphView;
    using Graph = LoopsLib::DS::EmbeddedGraph;
    using Trajectory = LoopsLib::MovetkGeometryKernel::Polyline;

    /**
     * \brief Leftright pointer including inclusion
     */
    struct LeftRightPointers
    {
        Interval pointers;
        LeftRightInclusion inclusion;
        int minWi = -1;
        int maxWi = -1;
    };

    // Contains left/right pointers, indexed on sink vertex of an edge.
    // Boolean records whether the rightpointer is the rightmost point possible on FD_j.
    using PointerMap = std::map<Graph::Id_t, LeftRightPointers>;


    // Debug flags for the computation
    struct StrongFrechetComputationDebug
    {
        static constexpr bool VerifyCorrectPointers = true;
    };

    /**
     * \brief Path pointer structure.
     */
    struct PathPointer
    {
        Graph::Id_t edge = -1;
        Interval srcInterval;
        bool wasProcessed = false;

        bool isSet() const
        {
            return edge >= 0;
        }

        LoopsLib::NT lowestPointAtSelf = std::numeric_limits<LoopsLib::NT>::max();
    };

    struct IntervalPolynomial
    {
        // Smallest epsilon required to have a finite interval
        NT minEps = -1;
        // Length of associated segment
        NT segLen = -1;
        // Parallel projection length of point on segment.
        NT proj = -1;
        NT perp = -1;

        // Polynomial type
        char type = 'n'; //'a','i' or 'b'. 'n' for 'not set'

        void compute(const LoopsLib::Math::Vec2<NT>& seg0, const LoopsLib::Math::Vec2<NT>& seg1,
                     const LoopsLib::Math::Vec2<NT>& point);
        NT lowerUnclamped(NT eps) const;
        NT upperUnclamped(NT eps) const;
        LoopsAlgs::Frechet::Interval getIntervalUnclamped(NT eps) const;

        static inline NT sqrtDiff(NT val, NT minusVal)
        {
            return std::sqrt(std::max(val - minusVal, (NT)0));
        }

        static inline NT clamp(NT val, NT min, NT max)
        {
            return val < min ? min : (val > max ? max : val);
        }
        static inline NT sq(NT val)
        {
            return val * val;
        }

        bool containsLeft(NT eps) const;

        bool containsRight(NT eps) const;

        NT lower(NT eps) const;

        NT upper(NT eps) const;

        Interval getInterval(NT eps) const;
    };



    /**
     * \brief Struct with strong frechet matching on graph for a single path.
     */
    struct StrongFrechetGraphData
    {
        const Graph* m_graph = nullptr;
        //std::decay_t<decltype(m_graph->locations())>* m_locations = nullptr;
        // The polyline under consideration
        Polyline polyline;

        // The one dimensional white intervals per FD_i for every vertex.
        // Indexed by vertex, then cell index.
        // TODO: compress to only intervals. Find way of indexing that is appropriate.
        std::vector<std::vector<Interval>> FDiWhiteIntervals;
        //Per vertex a list of full white intervals (possibly spanning multiple cells)
        std::vector<std::vector<Interval>> WhiteIntervals;
        // Left right endpoint inclusions per cell.
        std::vector<std::vector<LeftRightInclusion>> IntervalInclusions;
        // Maps cell to interval ID. TODO find more efficient way when needed
        std::vector<std::vector<int>> CellToIntervalId;
        // Interval ID to start cell
        std::vector<std::vector<int>> IntervalIdToCell;

        // View on the total graph that only contains vertices relevant for the given polyline.
        GraphView m_view;

        // Left and right pointers.
        // For every vertex, for every cell.
        std::vector<std::vector<PointerMap>> lrPointers;

        // Epsilon to work with
        LoopsLib::NT m_epsilon;

        /**
         * \brief Estimate of used memory in bytes
         * \return The (estimated) used memory in bytes
         */
        std::size_t byteSizeEstimate() const;

        void clear();


        /**
         * \brief Get the range of free intervals that intersect the lr-pointer
         * \param targetVert The vertex to get the intervals from
         * \param interval The lr interval range
         * \return Pair of start and end index of the reachable intervals
         */
        std::pair<std::size_t, std::size_t> intervalRangeForLrPointer(std::size_t targetVert,
                                                                      const Interval& interval) const;

        /**
         * Get interval ID. Throws when no white interval is at the given position
         */
        std::size_t intervalForVertexLocation(std::size_t targetVert, NT pos) const;

        /**
         * \brief Returns the index of the first interval that is above the position, or the
         * interval that contains the position. Returns -1 if no such interval exists
         * \param targetVert The vertex for which to search the white intervals
         * \param pos The position
         * \return The index of the white interval, or -1 if not found.
         */
        int firstIntervalAbove(std::size_t targetVert, NT pos) const;

        std::size_t polylineEdgeCount() const;

        bool isWhitePoint(std::size_t targetVert, NT pos) const;

        std::size_t positionToCell(NT freeSpacePos) const;

        /**
         * \brief Returns the number of vertices in the subgraph, as given by the graph view.
         * \return The number of vertices
         */
        std::size_t number_of_vertices() const;

        Interval& getWhiteInterval(std::size_t targetVert, const NT& lowEndpoint);

        const Interval& getWhiteInterval(std::size_t targetVert, const NT& lowEndpoint) const;

        bool isPotentialEndpoint(std::size_t vert, std::size_t interval) const;

        bool isPotentialStartPoint(std::size_t vert) const;

        // Path pointers for path reconstruction
        // Per vertex, per interval, a path pointer
        // Path pointer consists of vertex and interval, pointing to the previous vertex and associated interval
        // that lead to a path to the given vertex and interval.
        //std::vector<std::vector<std::pair<DS::BaseGraph::Id_t, Helpers::Geometry::Interval>>> pathPointers;

        StrongFrechetGraphData();

        void setup(const Graph* graph);

        void setup(const Graph* graph, const std::vector<Kernel::MovetkPoint>& lineLocations);

        void setup(const Graph* graph, const std::vector<Graph::Edge*>& lineEdges);
        void setup(const Graph* graph, const std::vector<Graph::Id_t>& lineEdges);

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
        FrechetGraphComputations(NT epsilon):m_epsilon(epsilon){}

        double fdiTimeMs() const;

        double lrPointersTimeMs() const;


        /**
         * \brief Constructs the view, stored in data.m_view, for the current assigned polyline.
         * This makes sure that not the entire graph will be taken when not absolutely necessary
         * \param data The output data object
         * \param epsilon The epsilon (max Frechet distance) to take into account
         */
        void constructFrechetView(StrongFrechetGraphData& data) const;

        /**
         * \brief Computes the [0,1] intersection interval at the given point for the given segment.
         * \param seg The segment
         * \param p The point
         * \param eps The epsilon to use
         * \return A pair containing the interval and a structure flaggin whether 0 and 1 are included in the interval.
         */
        static std::pair<Interval, LeftRightInclusion> computeIntersectionInterval(
            const Segment& seg, const Point& p, const NT& eps);

        static std::pair<Interval, LeftRightInclusion> computeIntersectionInterval(
            const Point& s0, const Point& s1, const Point& p, const NT& eps);

        void computeFDi(StrongFrechetGraphData& data);

        void computeLeftRightPointersBF(StrongFrechetGraphData& data);
    };

    // SWEEPLINE ALGORITHMS DATA

    /**
     * \brief Priority queue node in the Wenk sweepline algorithm
     *
     */
    struct PrioQueueNode
    {
        // The interval in the node
        Interval inter;
        // The vertex associated to the vertex
        Graph::Id_t vertex;

        //TODO unused?
        int interIndex = -1;

        /**
         * \brief Compare based on the minimum interval coordinate
         * \param other The other node
         * \return Whether this node is smaller than the other, based on the interval
         */
        bool operator<(const PrioQueueNode& other) const
        {
            if (inter.min != other.inter.min) return inter.min < other.inter.min; //Arbitrary tie breaking on order
            return vertex < other.vertex;
        }
        bool operator>(const PrioQueueNode& other) const
        {
            //return (inter.min == other.inter.min && vertex < other.vertex) || inter.min < other.inter.min;
            if(inter.min != other.inter.min) return inter.min > other.inter.min; //Arbitrary tie breaking on order
            return vertex > other.vertex;
        }

        bool operator==(const PrioQueueNode& other) const
        {
            return (inter.min == other.inter.min && vertex == other.vertex);
        }

        PrioQueueNode() {}
        PrioQueueNode(const PrioQueueNode& other) :inter(other.inter), vertex(other.vertex), interIndex(other.interIndex) {}

        PrioQueueNode(Graph::Id_t vertex) :vertex(vertex) {}

        PrioQueueNode(const Interval& interval, Graph::Id_t vertex, int intervalIndex) :inter(interval), vertex(vertex), interIndex(intervalIndex) {}
    };

    // Helper SFINAE structs for determining hooks in the supplied hook object.
    namespace detail {
        template<typename T, typename = std::void_t<>>
        struct has_beforeEdgeProcess : std::false_type { };

        template<typename T>
        struct has_beforeEdgeProcess<T, std::void_t<decltype(
            std::declval<T&>().beforeEdgeProcess(
                std::declval<Graph::Edge*>(),
                std::declval<const PrioQueueNode&>(),
                std::declval<const Interval&>(),
                std::declval<const StrongFrechetGraphData&>()))>
        > : std::true_type {};

        template<typename T, typename = std::void_t<>>
        struct has_beforeEdgeProcessExtended : std::false_type { };

        template<typename T>
        struct has_beforeEdgeProcessExtended <T, std::void_t<
            decltype(
                std::declval<T&>().beforeEdgeProcess(
                    std::declval<Graph::Edge*>(),
                    std::declval<const PrioQueueNode&>(),
                    std::declval<const Interval&>(),
                    std::declval<const StrongFrechetGraphData&>(),
                    std::declval<const std::vector<std::vector<PathPointer>>&>()
                )
            )>
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
                std::declval<const StrongFrechetGraphData&>()
            )
            )>
        > : std::true_type {};
    }

    enum class WenkSweeplineResult
    {
        SkipRest,
        Done,
        ContinueIt,
        DoneAfterUpdates
    };
}
#endif