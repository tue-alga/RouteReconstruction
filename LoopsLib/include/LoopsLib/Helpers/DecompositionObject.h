#ifndef HELPERS_DECOMPOSITIONOBJECT_H
#define HELPERS_DECOMPOSITIONOBJECT_H
#include <Eigen/Eigen>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/Algs/NNLS.h>
#include <LoopsLib/Algs/Types.h>
#include "Logger.h"
#include <LoopsLib/DS/EmbeddedGraph.h>
namespace LoopsLib::Helpers{
    struct DecompositionObject;

    template<>
    constexpr int logLevel<DecompositionObject>()
    {
        return LogLevel::Info;
    }

    struct DecompositionObject{
        using BasisElement_t = std::vector<DS::BaseGraph::Edge*>;
        using Basis_t = std::vector<BasisElement_t>;
        using Coeffs_t = std::vector<MovetkGeometryKernel::NT>;

        using Point = MovetkGeometryKernel::MovetkPoint;

        LogFactory<DecompositionObject> m_logger;

        // Name of the graph
        std::string m_graphName;
        // Path of the loaded graph, or empty if generated.
        std::string m_graphPath;
        // Path of the loaded field, or empty if generated
        std::string m_fieldPath;
        // Path of the loaded decomposition, or empty if generated
        std::string m_decompositionPath;

        // Basis stored as rows
        Basis_t m_basis;
        // Coefficient per basis element
        Coeffs_t m_decompositionCoeffs;
        // The field that was being decomposed
        Coeffs_t m_field;
        // The used graph
        DS::EmbeddedGraph* m_graph = nullptr;

        // The paths to use in the decomposition.
        // Given as rowvectors into the edges. Assumed simple.
        std::vector<std::vector<DS::BaseGraph::Edge*>> m_paths;

        // Flow values associated with the paths.
        std::vector<typename MovetkGeometryKernel::NT> m_pathValues;

        // The paths that are ''known''
        std::set<std::size_t> m_availablePaths;

        // Sources of the field
        std::set<DS::BaseGraph::Id_t> m_sources;

        // Sinks of the field.
        std::set<DS::BaseGraph::Id_t> m_sinks;

        // The objective value, retrieved from applying NNLS
        MovetkGeometryKernel::NT m_objectValueNNLS = std::numeric_limits<double>::max();

        /**
         * Methods
         *
         */

        const Point& vertexLocation(DS::BaseGraph::Id_t vertex) const;

        const Point& vertexLocation(DS::BaseGraph::Vertex* vertex) const;

        /**
         * \brief Size of the current basis
         * \return The size of the basis
         */
        std::size_t basisSize() const;

        NT fieldValue() const;


        std::vector<NT> decompositionField() const;

        void pickRandomAvailablePaths(std::size_t number);

        void reconstructFieldFromPaths();

        /**
         * \brief Reconstructs the used sources and sinks from the paths.
         */
        void sourcesAndSinksFromPaths();

        void clearSources();

        void clearSinks();

        bool isSameElement(std::size_t basisElement, const std::vector<DS::BaseGraph::Edge*>& element);

        static bool isSameElement(const std::vector<DS::BaseGraph::Edge*>& e0,
                                  const std::vector<DS::BaseGraph::Edge*>& e1);

        /**
         * \brief Clears the basis.
         * Leaves the paths, graph, locations, source and sink present.
         */
        void clear();
        
        /**
         * \brief Returns whether the basis element at the given index is a simple path
         * \param index The index (row) of the basis element
         * \param isEmpty Output variable set to true if the row is empty
         * \return Whether or not the path is simple. Returns true if the row is empty
         */
        bool isBasisElementSimple(int index, bool& isEmpty);

        static Eigen::VectorXd getEigenPath(DS::BaseGraph* graph, const std::vector<DS::BaseGraph::Edge*>& edges);

        static Eigen::VectorXd getEigenPath(DS::BaseGraph* graph, const std::vector<DS::BaseGraph::Id_t>& vertices);

        std::vector<DS::BaseGraph::Edge*> toEdgePointers(const std::vector<DS::BaseGraph::Id_t>& vertices) const;

        /**
         * \brief Concatenates the extra elements to the basis
         * \param extraElements The extra elements
         */
        void extendBasis(const Basis_t& extraElements);

        void applyNNLSAndPrune(double pruneValue);

        /**
         * \brief Verify simplcity of the basis. 
         * \return The result vector. Values denote: 0=non-simple, 1=simple, 2=empty row.
         */
        Eigen::VectorXi verifySimplicity();

        /**
         * \brief Computes the residual
         * \return The residual field, defined by the field minus the basis multiplied with their coefficients.
         */
        std::vector<NT> residual() const;

        /**
         * \brief Prunes the decomposition by throwing away basis elements with a decomposition coefficient
         * lower than the given value. THIS REQUIRES THE DECOMPOSITION OBJECT TO BE SORTED FIRST!
         * \param minimumDecompCoeffValue The minimum coefficient value to be allowed in the basis
         */
        void prune(double minimumDecompCoeffValue);

        /**
         * \brief Applies non-negative least squares for the current basis.
         */
        void applyNNLSForCoefficients();

        /**
         * \brief Sorts the basis by decomposition coefficient, the largest coming first.
         */
        void sortBasisByDecompCoeff();
    };
}
#endif