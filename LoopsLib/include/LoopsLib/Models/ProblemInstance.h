#ifndef MODELS_PROBLEMINSTANCE_H
#define MODELS_PROBLEMINSTANCE_H
#include <Eigen/Eigen>
#include <LoopsLib/DS/EmbeddedGraph.h>
#include <LoopsLib/Algs/NNLS.h>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Helpers/Logger.h>
#include "FlowField.h"

namespace LoopsLib::Models
{
    struct ProblemInstance;
}

namespace LoopsLib::Helpers
{
    template<>
    constexpr int logLevel<LoopsLib::Models::ProblemInstance>()
    {
        return LogLevel::Info;
    }
}

namespace LoopsLib::Models{

    struct ProblemInstance{
        using BasisElement_t = std::vector<DS::BaseGraph::Id_t>;
        using Basis_t = std::vector<BasisElement_t>;
        using Coeffs_t = std::vector<MovetkGeometryKernel::NT>;
        using Trajectory = std::vector<MovetkGeometryKernel::MovetkPoint>;

        using Point = MovetkGeometryKernel::MovetkPoint;

        // Logger
        Helpers::LogFactory<ProblemInstance> m_logger;

        struct Paths
        {
            // Path to the used graph
            std::string graphPath;
            // Path to the used field
            std::string fieldPath;
            // Path to file of this instance, if already saved
            std::string instancePath;
        } m_savePaths;

        // The field that was being decomposed
        LoopsLib::Models::FlowField* m_field;
        // The used graph
        DS::EmbeddedGraph* m_graph = nullptr;

        /**
         * \brief Describes the source of the trajectory.
         * This is either an actual trajectory form a dataset or a 
         * flow field path, identified by an index.
         */
        struct TrajectorySource
        {
            // Below should be mutual exclusive!

            // ID of the trajectory in the dataset.
            std::string m_trajectoryId;
            // Index of a path in the field
            long long m_pathSource = -1;

            /**
             * \brief Returns true if the source is a flow field path.
             * Otherwise, it is assumed that the source is from a trajectory dataset.
             * \return The source is a flow field path.
             */
            bool hasPathSource() const;

            std::string desc() const;
        };

        // The representatives
        std::vector<Trajectory> m_representatives;
        // 
        std::vector<TrajectorySource> m_representativeSources;

        NT m_epsilon = 0;


        void setRepresentativesFromPaths(const std::set<std::size_t>& paths);
        void setRepresentativesFromPaths(const std::vector<std::size_t>& paths);
        void setRepresentativesFromPaths(const std::set<long long>& paths);
        void setRepresentativesFromPaths(const std::vector<long long>& paths);

        void addRepresentativesFromPaths(const std::set<std::size_t>& paths);

        void addRepresentativeFromPath(const std::size_t& pId);

        void setRepresentativesFromTrajectories(const std::vector<Trajectory>& trajectories,
                                                const std::vector<std::string>& ids);

        void setRepresentativesFromTrajectories(const LoopsLib::MovetkGeometryKernel::TrajectorySet& trajSet, const std::set<long long>& indices, bool convertCrs);
        void setRepresentativesFromTrajectories(const LoopsLib::MovetkGeometryKernel::TrajectorySet& trajSet, const std::vector<long long>& indices, bool convertCrs);

        void addRepresentativesFromTrajectories(const std::vector<Trajectory>& trajectories,
                                                const std::vector<std::string>& ids);

        void addRepresentativeFromTrajectory(const Trajectory& trajectory, const std::string& id);


        /**
         * \brief Returns the vertexlocation of the given vertex for the referenced graph in this problem instance.
         * \param vertex The vertex ID
         * \return Const reference to the vertex location
         */
        const Point& vertexLocation(DS::BaseGraph::Id_t vertex) const;

        const Point& vertexLocation(DS::BaseGraph::Vertex* vertex) const;
        
        void pickRandomAvailablePaths(std::size_t number);

        std::string representativeDescription(size_t index);


        static Eigen::VectorXd getEigenPath(DS::BaseGraph* graph, const std::vector<DS::BaseGraph::Edge*>& edges);

        static Eigen::VectorXd getEigenPath(DS::BaseGraph* graph, const std::vector<DS::BaseGraph::Id_t>& vertices);

        std::vector<DS::BaseGraph::Edge*> toEdgePointers(const std::vector<DS::BaseGraph::Id_t>& vertices) const;
    };
}
#endif