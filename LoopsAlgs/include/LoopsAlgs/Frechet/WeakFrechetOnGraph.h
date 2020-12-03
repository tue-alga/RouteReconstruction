#ifndef WEAKFRECHET_ON_GRAPH_H
#define WEAKFRECHET_ON_GRAPH_H
#include <LoopsLib/DS/BaseGraph.h>
#include <LoopsLib/Algs/Types.h>
#include "FrechetHelpers.h"
namespace LoopsAlgs::Frechet
{
    struct WeakFrechetLogging
    {
        static constexpr bool LogDP = false;
        static constexpr bool AssertCorrectPointers = false;
    };


    /**
     * \brief Sets up structure for the decision problem
     * Based on https://www2.cs.arizona.edu/~alon/papers/gFrechet.pdf
     */
    class WeakFrechetOnGraph
    {
    public:
        using NT = LoopsLib::NT;
        using Segment = typename LoopsLib::MovetkGeometryKernel::MovetkSegment;
        using Point = typename LoopsLib::MovetkGeometryKernel::MovetkPoint;
    private:

        // The graph to build on
        LoopsLib::DS::BaseGraph* m_graph;

        // The epsilon to us
        double m_epsilon;

        // Create segment in Movetk
        using MakeSegment = typename movetk_core::MakeSegment<LoopsLib::MovetkGeometryKernel>;
        // Contains left/right pointers, indexed on sink vertex of an edge.
        // Boolean records whether the rightpointer is the rightmost point possible on FD_j.
        using PointerMap = std::map<LoopsLib::DS::BaseGraph::Id_t, std::pair<double,bool>>;
        // A polyline, represented as a list of segments
        using Polyline = std::vector<Segment>;

        std::pair<Interval, LeftRightInclusion> computeIntersectionInterval(
            const Segment& seg, const Point& p, const NT& eps);

        struct Component
        {
            int startInterval;
            int endInterval;
            LoopsLib::DS::BaseGraph::Id_t id = -1;
            LoopsLib::DS::BaseGraph::Id_t edgeSrc = -1;
            LoopsLib::DS::BaseGraph::Id_t componentIndex = -1;
        };

        /**
         * \brief Struct with all computed data.
         */
        struct Data
        {
            // The polyline under consideration
            Polyline polyline;
            const std::vector<Point>& locations;
            // The one dimensional white intervals per FD_i for every vertex.
            // Indexed by vertex, then cell index.
            // Opened intervals
            std::vector<std::vector<bool>> FDiWhiteIntervals;

            // Left right endpoint inclusions per interval.
            std::vector<std::vector<LeftRightInclusion>> IntervalInclusions;

            LoopsLib::DS::BaseGraph ComponentsGraph;

            // Map componentgraph vert to original edge
            std::vector<LoopsLib::DS::BaseGraph::Id_t> ComponentVertToEdge;

            // Maps cell to connected component
            std::vector<std::vector<int>> ConnComponentIds;
            // Map connected component to cell
            std::vector<std::vector<int>> ConnComponentIdToCell;

            // Per edge, define component range at start and end vertex
            std::vector<std::vector<Component>> ComponentMap;

            // Path pointers for path reconstruction
            // Per vertex, per interval, a path pointer
            // Path pointer consists of vertex and interval
            std::vector<std::vector<std::pair<LoopsLib::DS::BaseGraph::Id_t, Interval>>> pathPointers;

            Data(const std::vector<Point>& locations);

            void setup(LoopsLib::DS::BaseGraph* graph);

            Data(const std::vector<Point>& locations, LoopsLib::DS::BaseGraph* graph);
        };

        /**
         * \brief Computes all one dimensional free space surfaces for all vertices in the graph
         * \param polyline The polyline to use for the distance
         * \param locations The locations of vertices, indexed by vertex ID
         * \param epsilon Epsilon to use
         * \param FDiWhiteIntervals Output list of white intervals per vertex.
         */
        void computeFDi(Data& data);

        void computeComponents(Data& data);

        bool computeBastardFrechetPath(Data& data, std::vector<LoopsLib::DS::BaseGraph::Edge*>& path);
        bool applyDynamicProgram(Data& data, LoopsLib::DS::BaseGraph::Id_t& endVert, int& endInterval);

        void reconstructPath(Data& data, LoopsLib::DS::BaseGraph::Id_t vId, int endInterval,
                             std::vector<LoopsLib::DS::BaseGraph::Id_t>& path);


        /**
         * \brief Throw away all data that is not weakly connected to a start or endpoint
         * \param data The data to manipulate
         */
        void pruneViaWeakConnectivity(Data& data);

        /**
         * \brief Check if any of the endpoints is weakly connected to any of the starting points.
         * If not, no solution is possible anyway
         * \param data The computed data
         * \return Is there a potential solution
         */
        bool hasPotentialSolution(Data& data);

    public:
        WeakFrechetOnGraph(LoopsLib::DS::BaseGraph* graph);

        void setTargetGraph(LoopsLib::DS::BaseGraph* graph);

        void compute(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path, const std::vector<Point>& locations, double epsilon,
                     std::vector<LoopsLib::DS::BaseGraph::Id_t>& outputPath);
    };
}
#endif