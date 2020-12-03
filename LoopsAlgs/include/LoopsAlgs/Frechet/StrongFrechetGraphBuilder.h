#ifndef LOOPSALGS_FRECHET_STRONGFRECHETGRAPHBUILDER
#define LOOPSALGS_FRECHET_STRONGFRECHETGRAPHBUILDER
#include <LoopsAlgs/Frechet/WenkSweepline.h>
#include <LoopsLib/Helpers/Logger.h>

namespace LoopsAlgs::Frechet
{
    class StrongFrechetGraphBuilder;
}
namespace LoopsLib::Helpers
{
    template<>
    constexpr int logLevel<LoopsAlgs::Frechet::StrongFrechetGraphBuilder>()
    {
        return LogLevel::Info;
    }
    template<> inline std::string ClassLogLevel<LoopsAlgs::Frechet::StrongFrechetGraphBuilder>::Prefix() {
        static const char* nm = "StrongFrechetGraphBuilder"; return nm;
    }
}

namespace LoopsAlgs::Frechet
{
    struct WeightedStrongFrechetLogging
    {
        static constexpr bool LogDP = true;
        static constexpr bool AssertCorrectPointers = true;
    };


    /**
     * \brief Sets up structure for the decision problem
     * Based on https://www2.cs.arizona.edu/~alon/papers/gFrechet.pdf
     */
    class StrongFrechetGraphBuilder
    {
    public:
        using NT = LoopsLib::NT;
        using Segment = typename LoopsLib::MovetkGeometryKernel::MovetkSegment;
        using Point = typename LoopsLib::MovetkGeometryKernel::MovetkPoint;
    private:
        // Logger
        LoopsLib::Helpers::LogFactory<StrongFrechetGraphBuilder> m_log;

        using FrechetData = Frechet::StrongFrechetGraphData;
        using Interval = LoopsLib::Geometry::Interval;

        // The graph to build on
        LoopsLib::DS::EmbeddedGraph* m_graph;

        // The epsilon to us
        LoopsLib::NT m_epsilon;

        // Maximum path reconstruction search time, in seconds
        double m_maxSearchTimeS = 60;

        // Increase the bottom of an lrpointer a bit 
        bool m_artificiallyInflate = true;

        // Create segment in Movetk
        using MakeSegment = typename movetk_core::MakeSegment<LoopsLib::MovetkGeometryKernel>;

        // Contains left/right pointers, indexed on sink vertex of an edge.
        // Boolean records whether the rightpointer is the rightmost point possible on FD_j.
        using PointerMap = std::map<LoopsLib::DS::BaseGraph::Id_t, std::pair<double, bool>>;

        // A polyline, represented as a list of segments
        using Polyline = std::vector<Segment>;


        // Keep track of all intervals with weights
        struct WeightInterval
        {
            // Lower endpoint of the associated interval. Note that all locations in
            // the white interval above this value can be reached with the given weight
            NT lowEndpoint;
            // The source vertex causing this weight
            LoopsLib::DS::BaseGraph::Id_t src;
            // 
            NT srcEndpoint;
            // The weight
            NT weight;

            // Lexicographically sort on lowEndpoint, src and weight
            bool operator<(const WeightInterval& other) const
            {
                if (lowEndpoint == other.lowEndpoint) {
                    if (src == other.src) return weight < other.weight;
                    return src < other.src;
                }
                return lowEndpoint < other.lowEndpoint;
            }
        };
        // Sorted list of weight intervals
        struct WeightIntervalList
        {
            std::set<WeightInterval> m_inters;

            auto begin()
            {
                return m_inters.begin();
            }

            bool empty() const
            {
                return m_inters.empty();
            }

            auto end()
            {
                return m_inters.end();
            }
            auto rbegin()
            {
                return m_inters.rbegin();
            }
            auto rend()
            {
                return m_inters.rend();
            }
            std::size_t size() const
            {
                return m_inters.size();
            }

            void fixWeightMonotonicity()
            {
                // Dumb linear reconstruction
                std::set<WeightInterval> newInters;
                NT currMax = std::numeric_limits<NT>::lowest();
                for (const auto& el : m_inters)
                {
                    if (el.weight > currMax) {
                        newInters.insert(el);
                        currMax = el.weight;
                    }
                }
                m_inters = std::move(newInters);
            }

            void push_front(const WeightInterval& wi)
            {
                if (m_inters.empty()) m_inters.insert(wi);
                else if (m_inters.begin()->weight > wi.weight) {
                    return;
                }
                else
                {
                    m_inters.insert(wi);
                    fixWeightMonotonicity();
                }
            }
            void insert(const WeightInterval& wi)
            {
                if (m_inters.empty()) m_inters.insert(wi);
                else
                {
                    m_inters.insert(wi);
                    fixWeightMonotonicity();
                }
            }
            void insertMultiple(const std::vector<WeightInterval>& wi)
            {
                m_inters.insert(wi.begin(), wi.end());
                fixWeightMonotonicity();
            }
        };

        // Per vertex, per white interval, the set of weighted origin intervals
        std::vector<std::vector<WeightIntervalList>> m_weightIntervals;

        /**
         * \brief Returns iterator range of overlapping weight intervals with the given
         * interval. The second iterator (end) is exclusive
         * \param vertex
         * \param intervalId
         * \param interval
         * \return
         */
        std::pair<std::set<WeightInterval>::iterator, std::set<WeightInterval>::iterator>
            findOverlappingWeightIntervals(std::size_t vertex, std::size_t intervalId,
                const Interval& interval);

        // Potential end vertices.
        std::set<LoopsLib::DS::BaseGraph::Id_t> m_potentialEndpoints;
        std::set<LoopsLib::DS::BaseGraph::Id_t> m_seenEndpoints;

        // Reference to be set at compute. Used in computing the weighing.
        const std::vector<NT>* m_weights = nullptr;

        void reconstructPath(FrechetData& data, LoopsLib::DS::BaseGraph::Id_t startVert, std::vector<LoopsLib::DS::BaseGraph::Id_t>& path);

        /**
         * \brief Reconstruct a path from
         * \param data
         * \param paths
         * \param weights
         */
        void reconstructPaths(FrechetData& data, std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& paths, std::vector<NT>& weights);


    public:
        /**
         * \brief Hook for the Wenk sweepline algorithm. Called before processing edges
         * \param edge The edge to be processed
         * \param node Prioqueue node with source vertex and active interval
         * \param lrInterval Lr pointer interval for the active interval
         * \param data The Frechet data for the graph
         * \return WenkSweeplineResult that denotes the next action to take for the algorithm.
         */
        Frechet::WenkSweeplineResult beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* edge, const Frechet::PrioQueueNode& node, const Interval& lrInterval, const FrechetData& data);

        WeightedStrongFrechet(LoopsLib::DS::EmbeddedGraph* graph);

        const std::set<LoopsLib::DS::BaseGraph::Id_t>& potentialEndpoints() const;

        LoopsLib::NT epsilon() const;

        const std::vector<std::vector<WeightIntervalList>>& weightIntervals() const;

        void setMaxSearchTimeS(double maxSearchTime);

        double maxSearchTimeS() const;

        void precompute(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path,
            const std::vector<Point>& locations, NT epsilon, Frechet::StrongFrechetGraphData& data);

        void compute(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path, const std::vector<Point>& locations,
            NT epsilon, const std::vector<NT>& weights, std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& outputPath);

        void compute(Frechet::StrongFrechetGraphData& data,
            const std::vector<NT>& weights,
            std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& outputPath);
    };
}
#endif