#ifndef LOOPSALGS_MAPMATCHING_FASTMAPMATCHING_H
#define LOOPSALGS_MAPMATCHING_FASTMAPMATCHING_H
#include "LoopsLib/DS/EmbeddedGraph.h"
#include "Parts/TransitionGraph.h"
#include "LoopsLib/Geometry/Index/EdgeIndex.h"

namespace LoopsAlgs::MapMatching
{

    class FastMapMatching
    {
    public:

        using Kernel = LoopsLib::MovetkGeometryKernel;
        using MapMatchedPath = Kernel::GraphTrajectory;
    private:
        using NT = LoopsLib::NT;
        using Graph_t = LoopsLib::DS::EmbeddedGraph;
        struct Candidate
        {
            Graph_t::EdgeIndex edge;
            LoopsLib::MovetkGeometryKernel::MovetkPoint location;
            NT offsetFromStart; // In spatial units of the embedded graph.
        };
        /**
         * \brief Computation object for returning the edge lengths, which will be memoized to not 
         * repeat the computation
         */
        struct MemoizedEdgeLength
        {
            // The map of edge lengths
            mutable std::unordered_map<LoopsLib::DS::EmbeddedGraph::EdgeIndex, LoopsLib::NT> lengths;
            // The associated graph to compute the edgelengths on.
            LoopsLib::DS::EmbeddedGraph* m_graph;

            MemoizedEdgeLength(LoopsLib::DS::EmbeddedGraph* graph);

            LoopsLib::NT operator[](LoopsLib::DS::EmbeddedGraph::EdgeIndex edge) const;
        };
        

        struct EmissionProbProvider
        {
            const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& trajectory;
            LoopsLib::NT error;

            EmissionProbProvider(LoopsLib::NT error, const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& trajectory):
            trajectory(trajectory),
            error(error)
            {}

            template<typename CandidateIt>
            LoopsLib::NT operator()(std::size_t layer, CandidateIt it)
            {
                auto a = (trajectory[layer].first - it->location).sqLength() / (error*error);
                return std::exp(-0.5 * a);
            }
        };
        
        /**
         * \brief Computes the transmission probability between candidates of subsequent layers in the HMM graph.
         */
        class TransmissionProbProvider
        {
            const std::vector<std::vector<Candidate>>& m_candidates;
            Graph_t* m_graph;
            MemoizedEdgeLength m_edgeLengths;
            const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& m_trajectory;
            NT m_speedBound;
            bool m_verbose = false;
        public:
            TransmissionProbProvider(const std::vector<std::vector<Candidate>>& candidates, Graph_t* graph,
                                     const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& trajectory,
                                     NT speedBound);
            void setVerbose(bool value);

            std::pair<bool, LoopsLib::NT> operator()(std::size_t startLayer, std::size_t candidate,
                                                     std::size_t endLayer, std::size_t endCandidate);
        };
        using Vec2 = LoopsLib::Math::Vec2<LoopsLib::NT>;

        // Number of candidates
        int m_k = 8;
        // Radius to search in
        NT m_radius = 300;
        // Assumed GPS error to be used in Emission probability
        NT m_gpsError = 50;
        // Speedbound on shortest paths
        NT m_speedBound = std::numeric_limits<NT>::max();
        // Set verbose output
        bool m_verbose = true;
        // Save transmission graph to file
        bool m_saveTG = false;
        // Linked embedded graph
        LoopsLib::DS::EmbeddedGraph* m_graph;
        // The HMM graph
        Parts::TransitionGraph<LoopsLib::NT, EmissionProbProvider, TransmissionProbProvider> m_hmmGraph;
        // Index for generating candidates
        LoopsLib::Geometry::Index::EdgeIndex m_index;


        static Vec2 nearestOnSegment(const Vec2& queryPoint, const Vec2& s0, const Vec2& s1);

        Vec2 nearestOnEdge(const Vec2& queryPoint, const LoopsLib::DS::EmbeddedGraph::EdgeIndex& e, LoopsLib::NT& offset) const;
        /**
         * \brief Reconstruct the graph path from the candidate path
         * \param candidatePath The candidate path
         * \param out The output graph path.
         */
        void reconstructPath(const std::vector<Candidate>& candidatePath,
                             MapMatchedPath& out) const;
    public:
        struct SetMatchResultDescription
        {
            std::vector<std::string> matchedTrajectories;
            std::vector<int> matchOffset;
            std::vector<int> matchSize;
            std::vector<int> inputSize;
        };
        explicit FastMapMatching(LoopsLib::DS::EmbeddedGraph* graph);

        FastMapMatching(LoopsLib::DS::EmbeddedGraph* graph, LoopsLib::NT radius, LoopsLib::NT gpsError, int k);

        /**
         * \brief Mapmatch the given timestamepd trajectory
         * \param trajectory The trajectory
         * \param out The mapmatched path, given as edge indices in the graph
         */
        void mapMatch(const Kernel::TimestampedTrajectory& trajectory, MapMatchedPath& out);
        void mapMatch(const Kernel::TimestampedTrajectory& trajectory, MapMatchedPath& out, int& offset, int& size);
        void mapMatchFromWGS84(const Kernel::TimestampedTrajectory& trajectory, MapMatchedPath& out, int& offset, int& size);
        void mapMatch(const Kernel::TimestampedTrajectory& trajectory, const OGRSpatialReference& trajectoryRef, MapMatchedPath& out);
        void mapMatch(const Kernel::TimestampedTrajectory& trajectory, const OGRSpatialReference& trajectoryRef, MapMatchedPath& out, int& offset, int& size);
        void mapMatchSet(const Kernel::TimestampedTrajectorySet& trajectorySet, Kernel::GraphTrajectorySet& out, int count = -1);
        void mapMatchSet(const Kernel::TimestampedTrajectorySet& trajectorySet, Kernel::GraphTrajectorySet& out, SetMatchResultDescription& resultDescription, int count = -1);

        void setRadius(const LoopsLib::NT& radius);

        LoopsLib::NT radius() const;

        const Graph_t* graph() const { return m_graph; }
        Graph_t* graph() { return m_graph; }

        void setK(int k);

        void setSpeedBound(const LoopsLib::NT& speedBound);

        LoopsLib::NT speedBound() const;

        void setVerbose(const bool& verbose);

        bool verbose() const;


        int k() const;

        void setGpsError(LoopsLib::NT error);

        LoopsLib::NT gpsError() const;
    };
}
#endif