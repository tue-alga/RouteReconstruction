#ifndef ALGS_FLOWDECOMPOSITION_IFLOWDECOMPOSITION_H
#define ALGS_FLOWDECOMPOSITION_IFLOWDECOMPOSITION_H
#include <LoopsLib/DS/BaseGraph.h>
//#include "Helpers/DecompositionObject.h"
#include <LoopsLib/Models/DecompositionResult.h>
#include <LoopsLib/Models/ProblemInstance.h>
#include <iostream>
#include <Eigen/Eigen>
#include <LoopsLib/DS/EmbeddedGraph.h>

#define LOOPS_ALG(...)
#define LOOPS_ALG_PARAM(...)

namespace LoopsAlgs {
	namespace FlowDecomposition{

        class IFlowDecomposition;

        class FlowDecompositionHooks
        {
        public:
            virtual ~FlowDecompositionHooks(){}
            virtual void afterIteration(IFlowDecomposition* decomposer, LoopsLib::Models::DecompositionResult* result, int iteration) = 0;
            virtual void beforeStart(IFlowDecomposition* decomposer, LoopsLib::Models::DecompositionResult* result) = 0;
        };

        /**
		 * \brief Abstract base class for any flow decomposition algorithm.
		 */
		LOOPS_ALG(base) class IFlowDecomposition
		{
		public:
            // The types to use for the basis and decomposition coefficients
            using Basis_t = LoopsLib::Models::DecompositionResult::Basis_t;
            using BasisElement_t = LoopsLib::Models::DecompositionResult::BasisElement_t;
            using DecompositionCoeffs_t = LoopsLib::Models::DecompositionResult::Coeffs_t;
            using Graph_t = LoopsLib::DS::EmbeddedGraph;
            using DecompositionResult_t = LoopsLib::Models::DecompositionResult;
            using NT = LoopsLib::NT;
        protected:
            // The types to use for the basis and decomposition coefficients
            int m_numberOfIterations = 1;
            // The current iteration
            int m_currentIteration = 0;

            double m_pruneValue = 0.00001;

            // Container for all problem input and output data
            DecompositionResult_t* m_decompObj = nullptr;

			// Name of the algorithm
			std::string m_name;

            // Logging directory
            std::string m_logDir;

            // Log file prefix
            std::string m_logFilePrefix;
            // Log file prefix
            std::string m_logPrefix;

            /**
			 * \brief To be implemented in subclass. Actually apply the decomposition algorithm here
			 * \param field The field to decompose
			 * \param basis The output basis
			 * \param decompositionVal The output values for each path
			 * \param decompSize The size of the decomposition: number of positive value paths.
			 */
			virtual void findNewBasisElements() = 0;

            std::ostream& log();

            Basis_t& basis();

            std::size_t numberOfRepresentatives() const;

            const LoopsLib::Models::ProblemInstance::Trajectory& representative(std::size_t index) const;

            DecompositionCoeffs_t& coefficients();

            void extendBasis(const Basis_t& extra);

            /**
             * \brief Throw out all basis elements that have an associated value lower than the given value
             * \param minValue The value below which elements will be removed from the basis
             */
            void pruneBasis(double minValue);

            /**
             * \brief Solve the Non-negative Least Squares problem for the current basis. Stores the objective function
             * and the coefficients in the referenced decomposition result object.
             */
            void solveNNLS();

            DecompositionCoeffs_t& field();

            LoopsLib::DS::EmbeddedGraph* graph();

            virtual void init();
            
		public:
            IFlowDecomposition(const std::string& name, DecompositionResult_t* decompObj);
            IFlowDecomposition(const std::string& name) : m_name(name){}


            /**
             * \brief Resets the internals. Does not reset the internal DecompositionResult!
             */
            virtual void reset();


            /**
             * \brief Sets the decomposition object on the decompositor. Resets all internals
             * \param decompObj 
             */
            void setDecompositionObject(DecompositionResult_t* decompObj);

            DecompositionResult_t* decompositionObject() const;

            LOOPS_ALG_PARAM(type=setter,name=numberOfIterations,dtype=int) 
		    void setNumberOfIterations(int numberOfIts);

            LOOPS_ALG_PARAM(type=getter, name = numberOfIterations, dtype = int)
            int numberOfIterations() const;

            LOOPS_ALG_PARAM(type = setter, name = pruneValue, dtype = double)
            void setPruneValue(double value);

            LOOPS_ALG_PARAM(type = getter, name = pruneValue, dtype = double)
            double pruneValue() const;

            std::string logDir() const;

            void setLogDir(std::string logDir);

            std::string logPrefix() const;

            void setLogPrefix(std::string logDir);

            std::string logFilePrefix() const;

            void setLogFilePrefix(std::string logDir);

            /**
             * \brief Return meta data to reconstruct internal parameters
             * \return Map of key-value meta data
             */
            virtual std::map<std::string, std::string> metaData() const = 0;

            virtual void setFromMetaData(const std::map<std::string, std::string>& metData) = 0;

            /**
             * \brief Returns a string representation of the used parameters
             * \return String of used parameters
             */
            virtual std::string paramDescription() const = 0;

            /**
             * \brief Computes the next iteration. Finds basis elements and extends the basis.
             * Does NOT compute the NNLS and prunes the basis!
             * Apply manually on DecompositionResult object
             */
            virtual void nextStep();

            virtual bool isDone();

            void setName(const std::string& value);

			/**
			 * \brief Returns the name of the algorithm
			 * \return The name of the algorithm
			 */
            std::string name() const;

            virtual ~IFlowDecomposition();

            /**
             * \brief Applies the full decomposition algorithm.
             * Equivalent to repeatedly calling nextStep() and checking isDone().
             */
            virtual void decompose();
		};
	}
}
#endif
