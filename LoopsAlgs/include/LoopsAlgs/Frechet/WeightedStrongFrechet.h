#ifndef WEIGHTED_STRONG_FRECHET_H
#define WEIGHTED_STRONG_FRECHET_H
#include <LoopsAlgs/Frechet/WenkSweepline.h>
#include <LoopsLib/Helpers/Logger.h>
#include <movetk/geom/GeometryInterface.h>

namespace LoopsAlgs::Frechet
{
    class WeightedStrongFrechet;
}
namespace LoopsLib::Helpers
{
    template<>
    constexpr int logLevel<LoopsAlgs::Frechet::WeightedStrongFrechet>()
    {
        return LogLevel::Info;
    }
    template<> inline std::string ClassLogLevel<LoopsAlgs::Frechet::WeightedStrongFrechet>::Prefix() {
        static const char* nm = "WeightedStrongFrechet"; return nm;
    }
}

namespace LoopsAlgs::Frechet
{
    /**
     * \brief Sets up structure for the decision problem
     * Based on https://www2.cs.arizona.edu/~alon/papers/gFrechet.pdf
     */
    class WeightedStrongFrechet
    {
    public:
        using NT = LoopsLib::NT;
        using Segment = typename LoopsLib::MovetkGeometryKernel::MovetkSegment;
        using Point = typename LoopsLib::MovetkGeometryKernel::MovetkPoint;
    private:
        // Logger
        LoopsLib::Helpers::LogFactory<WeightedStrongFrechet> m_log;

        using FrechetData = Frechet::StrongFrechetGraphData;
        using Interval = LoopsLib::Geometry::Interval;

        // The graph to build on
        const LoopsLib::DS::EmbeddedGraph* m_graph;

        // The epsilon to use
        LoopsLib::NT m_epsilon;

        // Maximum path reconstruction search time, in seconds
        double m_maxSearchTimeS = 60;

        // Increase the bottom of an lrpointer a bit 
        bool m_artificiallyInflate = false;

        bool m_greedyAttachEnd = false;

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
            // The location at the src white interval that can reach this interval
            NT srcEndpoint;
            // The weight
            NT weight;

            // Lexicographically sort on lowEndpoint, weight and src
            bool operator<(const WeightInterval& other) const;
        };
        // Sorted list of weight intervals
        struct WeightIntervalList
        {
            std::set<WeightInterval> m_inters;

            using It = std::set < WeightInterval>::iterator;
            using ConstIt = std::set < WeightInterval>::const_iterator;
            using RIt = std::set < WeightInterval>::reverse_iterator;
            using ConstRIt = std::set < WeightInterval>::const_reverse_iterator;

            auto begin();

            bool empty() const;

            auto end();

            auto rbegin();

            auto rend();

            std::size_t size() const;

            /**
             * \brief Makes sure that the list has non-decreasing weights. Throws out violating members.
             */
            void fixWeightMonotonicity();

            void insert(const WeightInterval& wi);

            void insertMultiple(const std::vector<WeightInterval>& wi);
        };

        // Per vertex, per white interval, the set of weighted origin intervals
        std::vector<std::vector<WeightIntervalList>> m_weightIntervals;

        std::pair<bool, std::set<WeightInterval>::iterator>
            findHighestOverlappingWeightInterval(std::size_t vertex, std::size_t intervalId,
                const NT& queryValue, const StrongFrechetGraphData& data, bool verbose=false);

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
                                       const Interval& interval, const StrongFrechetGraphData& data);

        // Potential end vertices.
        std::set<LoopsLib::DS::BaseGraph::Id_t> m_potentialEndpoints;
        std::set<LoopsLib::DS::BaseGraph::Id_t> m_seenEndpoints;

        // Reference to be set at compute. Used in computing the weighing.
        const std::vector<NT>* m_weights = nullptr;

        int m_maxNum = 1;

        bool m_slowErrors = false;

        void reconstructPath(const FrechetData& data, LoopsLib::DS::BaseGraph::Id_t startVert, std::vector<LoopsLib::DS::BaseGraph::Id_t>& path);

        /**
         * \brief Reconstruct a path from 
         * \param data 
         * \param paths 
         * \param weights 
         */
        void reconstructPaths(const FrechetData& data, const std::vector<std::vector<Frechet::PathPointer>>& pathPointers, std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& paths, std::vector<NT>& weights);

        void slowError();
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

        WeightedStrongFrechet(const LoopsLib::DS::EmbeddedGraph* graph);

        const std::set<LoopsLib::DS::BaseGraph::Id_t>& potentialEndpoints() const;

        LoopsLib::NT epsilon() const;

        void setGreedyAttachEnd(bool value);

        void setMaximumPerPath(int maxim);

        int setMaximumPerPath() const;
        const std::vector<std::vector<WeightIntervalList>>& weightIntervals() const;

        void setMaxSearchTimeS(double maxSearchTime);

        double maxSearchTimeS() const;

        void precompute(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path,
             NT epsilon, Frechet::StrongFrechetGraphData& data);

        void compute(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path, 
            NT epsilon, const std::vector<NT>& weights, std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& outputPath);

        void precompute(const Trajectory& trajectory,
            NT epsilon, Frechet::StrongFrechetGraphData& data);

        void compute(const Trajectory& trajectory,
            NT epsilon, const std::vector<NT>& weights, std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& outputPath);

        void compute(const Frechet::StrongFrechetGraphData& data,
                     const std::vector<NT>& weights,
                     std::vector<std::vector<LoopsLib::DS::BaseGraph::Id_t>>& outputPath);
    };
}
#endif