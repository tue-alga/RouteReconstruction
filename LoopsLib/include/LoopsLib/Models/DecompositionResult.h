#ifndef MODELS_DECOMPOSITIONRESULT_H
#define MODELS_DECOMPOSITIONRESULT_H
#include <string>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Helpers/Logger.h>
#include <LoopsLib/Helpers/DecompositionObject.h>
#include "ProblemInstance.h"

namespace LoopsLib::Models
{
    struct DecompositionResult;
}
namespace LoopsLib::Helpers
{
    template<>
    constexpr int logLevel<LoopsLib::Models::DecompositionResult>()
    {
        return LogLevel::Info;
    }
}

// 
namespace LoopsLib::Models
{
    struct DecompositionResult
    {
        using GraphPath_t = std::vector<DS::BaseGraph::Id_t>;
        long long currentId = 0;

        using BasisElement_t = GraphPath_t;
        using Basis_t = std::vector<BasisElement_t>;
        using Coeffs_t = std::vector<MovetkGeometryKernel::NT>;

        // This object does not own the problem instance!
        ProblemInstance* m_relatedInstance = nullptr;

        // Logger 
        Helpers::LogFactory<DecompositionResult> m_logger;

        // Path to loaded file, or empty if generated
        std::string m_decompositionPath;
        // Path to associated field file, or empty if not known/existent yet.
        std::string m_fieldPath;

        // Timings of the decomposition algorithm, in ms, for by the algorithm
        // specified categories.
        std::map<std::string, double> m_timings;

        // The objective value, retrieved from applying NNLS
        MovetkGeometryKernel::NT m_objectValueNNLS = std::numeric_limits<double>::max();

        // Basis stored as rows
        Basis_t m_basis;

        std::vector<long long> m_basisSrcRepresentatives;
        std::vector<long long> m_basisIds;

        // Coefficient per basis element
        Coeffs_t m_decompositionCoeffs;

        NT m_distortion = 0;
        NT m_reverseDistortion = 0;

        // Data describing the used algorithm
        std::map<std::string, std::string> m_algMeta;

        DecompositionResult(){}
        DecompositionResult(ProblemInstance* instance): m_relatedInstance(instance){}

        /**
         * \brief Extends the basis and assigns meta data on where the basis came from directly
         * \param srcRepresentative The source representative from which the basis elements were derived
         * \param newElements The new elements
         */
        void extendBasis(long long srcRepresentative, const std::vector<GraphPath_t>& newElements);

        /**
         * \brief Call this when directly adding elements to the basis for a certain src representative
         * \param srcRepresentative The src representative
         */
        void updateRepresentativeIdsForNewElements(long long srcRepresentative);

        /**
         * \brief Clears the entire result. Keeps the pointer to the problem instance.
         */
        void clear();

        /**
         * \brief Size of the current basis
         * \return The size of the basis
         */
        std::size_t basisSize() const;

        void recomputeObjectiveValue();

        void pruneToTopWeighted(int number);

        /**
         * \brief Return the flow field produced by the basis
         * \return The flow field produced by the basis.
         */
        std::vector<NT> decompositionField() const;

        /**
         * \brief Return the list of current basis elements
         * \return The basis elements
         */
        const std::vector<BasisElement_t>& paths() const;

        /**
         * \brief Get the base edge field we are solving
         * \return The field
         */
        const std::vector<NT>& field() const;

        /**
         * \brief Concatenates the extra elements to the basis
         * \param extraElements The extra elements
         */
        void extendBasis(const Basis_t& extraElements);

        void applyNNLSAndPrune(double pruneValue);

        void applyNNLSAndPruneBatched(double pruneValue, std::size_t batchSize);

        bool applyNnlsIncrementally(std::size_t batchSize, std::size_t retainSize, bool retainFraction);


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