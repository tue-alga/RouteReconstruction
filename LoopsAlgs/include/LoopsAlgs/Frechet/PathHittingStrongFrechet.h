#ifndef PATH_HITTING_STRONG_FRECHET_H
#define PATH_HITTING_STRONG_FRECHET_H
#include <LoopsAlgs/Frechet/FrechetHelpers.h>
#include <LoopsLib/Helpers/Logger.h>
#include <movetk/geom/GeometryInterface.h>
namespace LoopsAlgs::Frechet {
    class PathHittingStrongFrechet;
}

namespace LoopsLib::Helpers
{
    template<>
    constexpr int logLevel<LoopsAlgs::Frechet::PathHittingStrongFrechet>()
    {
        return LoopsLib::Helpers::LogLevel::Info;
    }
    template<> inline std::string ClassLogLevel<LoopsAlgs::Frechet::PathHittingStrongFrechet>::Prefix() {
        static const char* nm = "PathHittingStrongFrechet"; return nm;
    }
}

namespace LoopsAlgs::Frechet
{
    /**
     * \brief Sets up structure for the decision problem
     * Based on https://www2.cs.arizona.edu/~alon/papers/gFrechet.pdf
     */
    class PathHittingStrongFrechet
    {
        // The graph to build on
        const LoopsLib::DS::EmbeddedGraph* m_graph;

        // The epsilon to us
        LoopsLib::NT m_epsilon;

        // Create segment in Movetk
        using MakeSegment = typename movetk_core::MakeSegment<LoopsLib::MovetkGeometryKernel>;

        struct PrefixHooks
        {
            // The freespace interval on the target edge to hit (at the start vertex)
            LoopsLib::Geometry::Interval targetEdgeInterval;
            // Lowest location on the target interval for which a Frechet path exists
            NT lowestIntervalPoint = std::numeric_limits<NT>::max();
            // Start vertex of the target edge
            LoopsLib::DS::BaseGraph::Id_t targetEdgeStartVert = -1;
            // Was a path found to the specified interval?
            bool intervalFound = false;

            WenkSweeplineResult beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* e, const PrioQueueNode& node, const LoopsLib::Geometry::Interval& lrPointer,
                const StrongFrechetGraphData& data);
        };

        struct PostfixHooks
        {
            int endIntervalOut = -1;
            LoopsLib::DS::BaseGraph::Id_t endVert = -1;

            WenkSweeplineResult beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* e, const PrioQueueNode& node, const LoopsLib::Geometry::Interval& lrPointer,
                const StrongFrechetGraphData& data);
        };

        /**
         * \brief Search for a prefix path to the target edge
         * \param data Frechet data for the graph
         * \param targetEdgeId The edge to hit
         * \param targetEdgeInterval The edge interval to hit
         * \param lowestInterval Output value for the lowest interval
         * \param lastInterval 
         * \param prefixPathPointers List of pathpointer paths
         * \return Was the prefix found
         */
        bool searchPrefixPath(const StrongFrechetGraphData& data, LoopsLib::DS::BaseGraph::Id_t targetEdgeId, const LoopsLib::Geometry::Interval&
                              targetEdgeInterval, NT& lowestInterval, int
                              & lastInterval, std::vector<std::vector<PathPointer>>& prefixPathPointers);

        bool searchPostfixPath(const StrongFrechetGraphData& data, LoopsLib::DS::BaseGraph::Id_t targetEdgeId, const LoopsLib::Geometry::Interval&
                               sourceInterval, int& endIntervalOut, int& endVert, std::vector<std::vector<PathPointer>>& pathPointers);

        void buildPrefixPath(const StrongFrechetGraphData& data, LoopsLib::DS::BaseGraph::Id_t targetEdgeId, LoopsLib::DS::BaseGraph::Id_t vId,
                             int endInterval, const std::vector<std::vector<PathPointer>>& pathPointers, std::vector<LoopsLib::DS::BaseGraph::Id_t>&
                             edgePath);

        void buildPostfixPath(const StrongFrechetGraphData& data, LoopsLib::DS::BaseGraph::Id_t vId, int endInterval,
                              const LoopsLib::Geometry::Interval& edgeInterval, LoopsLib::DS::BaseGraph::Id_t targetVertexId, std::vector<std::vector
                              <PathPointer>>& pathPointers, std::vector<LoopsLib::DS::BaseGraph::Id_t>& edgePath);

        bool searchPath(const StrongFrechetGraphData& data, LoopsLib::DS::BaseGraph::Id_t targetEdgeId, std::vector<LoopsLib::DS::BaseGraph::Id_t>& path);

    public:
        PathHittingStrongFrechet(const LoopsLib::DS::EmbeddedGraph* graph);

        /**
         * \brief Initializes empty algorithm. Make sure to set the graph later
         */
        PathHittingStrongFrechet();

        /**
         * \brief Sets the graph to use
         * \param graph The graph
         */
        void setTargetGraph(LoopsLib::DS::EmbeddedGraph* graph);

        /**
         * \brief Computes output paths for the given input path
         * \param path The input path
         * \param epsilon Epsilon to use
         * \param targetEdge Edge to hit
         * \param outputPath Output paths
         */
        void compute(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path, LoopsLib::NT epsilon,
            LoopsLib::DS::BaseGraph::Id_t targetEdge,
            std::vector<LoopsLib::DS::BaseGraph::Id_t>& outputPath);

        /**
         * \brief Computes an 
         * \param data Frechet data for the graph
         * \param targetEdge 
         * \param outputPath 
         */
        void computeNext(const StrongFrechetGraphData& data, 
            LoopsLib::DS::BaseGraph::Id_t targetEdge,
            std::vector<LoopsLib::DS::BaseGraph::Id_t>& outputPath);

        /**
         * \brief Precomputes the data needed for the path hitting computation
         * \param path
         * \param locations
         * \param epsilon
         * \param data
         */
        void precompute(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& edgePath, 
            LoopsLib::NT epsilon, StrongFrechetGraphData& data);

        void precompute(const Trajectory& trajectory,
            LoopsLib::NT epsilon, StrongFrechetGraphData& data);

        /**
         * \brief Reuse pathpointers of some initial sweep to speed up prefix finding
         * \param data The frechet data
         * \param targetEdge The target edge to hit
         * \param initialPathPointers The path pointers of the initial sweep
         * \param outputPath The output path
         */
        void computeNextUsingPathPointers(const StrongFrechetGraphData& data,
            LoopsLib::DS::BaseGraph::Id_t targetEdge,
            const std::vector<std::vector<PathPointer>>& initialPathPointers,
            std::vector<LoopsLib::DS::BaseGraph::Id_t>& outputPath);

        /**
         * \brief Precomputes path pointers from the graph data
         * \param data 
         * \param pathPointers 
         */
        void precomputePathPointers(const StrongFrechetGraphData& data,
                                    std::vector<std::vector<PathPointer>>& pathPointers);
    };
}
#endif