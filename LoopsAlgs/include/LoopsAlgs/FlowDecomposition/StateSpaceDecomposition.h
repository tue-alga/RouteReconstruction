#ifndef LOOPS_STATE_SPACE_DECOMPOSITION_H
#define LOOPS_STATE_SPACE_DECOMPOSITION_H
#include <vector>
#include <tuple>
#include <LoopsLib/Algs/ShortestPath/BellmanFord.h>
#include <queue>
#include <unordered_set>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Math/Vector.h>
#include <eigen3/Eigen/Eigen>
#include <ilcplex/ilocplexi.h>
#include <LoopsAlgs/IFlowDecomposition.h>
#include <LoopsLib/Algs/NNLS.h>
#include <LoopsLib/Helpers/Logger.h>

namespace LoopsAlgs::FlowDecomposition
{
	/**
	 * \brief Algorithm for a simple approach to path decomposition.
	 * The algorithm decomposes the flow in cycles and simple paths.
	 * If the given field is not an actual flow,
	 * \tparam Graph
	 * \tparam FlowField
	 */
	template<typename LongestPathOracle>
	class StateSpaceDecomposition : public IFlowDecomposition
	{
	public:
		using Scalar = double;
		using node_t = int;
		using FlowMap = Eigen::VectorXd;
		using Path = std::vector<node_t>;
	private:
		using NT = LoopsLib::NT;
        LoopsLib::Helpers::LogFactory<LongestPathOracle> m_logger;

		std::vector<LoopsLib::NT> m_searchDir;
        using Id_t = Graph_t::Id_t;

        static bool isSameElement(const std::vector<Id_t>& p0, const std::vector<Id_t>& p1)
        {
            if (p0.size() != p1.size()) return false;
            for(std::size_t i = 0 ; i < p0.size(); ++i)
            {
                if (p0[i] != p1[i]) return false;
            }
            return true;
        }
	    static bool isSameElement(const std::vector<Id_t>& p0, const std::vector<Graph_t::Edge*>& p1)
        {
            if (p0.size() != p1.size()) return false;
            for (std::size_t i = 0; i < p0.size(); ++i)
            {
                if (p0[i] != p1[i]->id()) return false;
            }
            return true;
        }

		int m_maxRetries = 5;
		int m_noImprovementRetries = 10;

		// When the objective value decreases less than this, stop.
		double m_stopThreshold = 1;

		void preprocess()
		{
			// Collapse node of degree 2, determine what the optimal flow
			// value would be there. (mean?)
		}
		BasisElement_t longestPath(const std::vector<LoopsLib::NT>& field, Id_t src, Id_t sink, bool retry = false)
		{
            m_oracle.setGraph(graph());
            if (retry)
                return m_oracle.retry(field);
            return m_oracle.compute(field,src,sink);
		}

		bool findPath(const std::vector<NT>& searchDir, int rowIndex, double currentObjectiveVal, std::vector<Graph_t::Id_t>& path, NT& decompValue)
		{
			using namespace Eigen;
			
            auto offset = LoopsLib::Math::VectorWrapper<NT>::Constant(searchDir.size(), 0);
            auto searchDirWrapper = LoopsLib::Math::wrapVec(searchDir);

            //TODO
            Graph_t::Id_t src = 0, sink = 0;

            std::vector<Graph_t::Id_t> ePath;

			bool success = false;
			// Check for degeneracies= same paths in basis, try to fix it by randomizing the search direction a bit
			for(int i = 0; i < m_noImprovementRetries; i++)
			{
				for(int j = 0; j < m_maxRetries; j++)
				{
					bool degenerate = false;
                    ePath = std::move(longestPath((LoopsLib::Math::wrapVec(searchDir) + offset).toStd(),src,sink));
					// Check if the path is already present in the basis.
					for(int k = 0; k < rowIndex; k++)
					{
                        if(isSameElement(basis()[j], ePath))
                        {
							degenerate = true;
							break;
						}
					}
					if (!degenerate) break;
					m_logger.warn("Degenerate path found");
					offset = LoopsLib::Math::wrapVec(LoopsLib::Math::VectorWrapper<NT>::Random(searchDir.size(),-1.0,1.0)) * searchDirWrapper.maxCoeff();
				}

				// Apply NNLS, see what objective function value it returns
                NT objectiveVal = m_decompObj->m_objectValueNNLS;
                m_decompObj->applyNNLSForCoefficients();

				// If the new objective is better, return success.
				if (!isApprox(objectiveVal, m_decompObj->m_objectValueNNLS))
				{
                    m_logger.info( "Decrease val by:" ,std::abs(objectiveVal - m_decompObj->m_objectValueNNLS));
					success = true;
					break;
				}
			}
            path = ePath;
            //std::transform(ePath.begin(), ePath.end(), std::back_inserter(path), [](auto* e) {return e->id(); });

			return success;
		}

		template<typename T>
		static bool isApprox(T t0, T t1, T epsilon = 0.00001){
			if(t0 > t1) return t0 - t1  <epsilon;
			return t1-t0 < epsilon;
		}

		/**
		 * Setups the initial decomposition
		 */
		void initializePathDecomposition()
		{
			const int edgeCount = graph()->number_of_edges();

            LoopsLib::Math::VectorWrapper<NT> searchDirection;
		    searchDirection.copyFrom(field());

			std::cout << "Initial field norm: " << searchDirection.norm() << ", max coeff: " << searchDirection.maxCoeff() << std::endl;

			// Initialize decomposition.
            basis().clear();// = MatrixXd::Zero(edgeCount, edgeCount); // Rows will contain basis, in normalized state.
            coefficients().clear();// assign(edgeCount, 0);

			double objectiveVal = std::numeric_limits<double>::max();

			// Try to construct an initial basis.
			for(int i =0; i < edgeCount; i++)
			{
                BasisElement_t path;
				NT newObjVal;
				if(!findPath(searchDirection.toStd(), i, objectiveVal, path, newObjVal))
				{
					// We will stop expanding the basis now.
                    m_logger.info("Stopping basis expansion, can only find degenerate point" );
					m_logger.info("Flow remainder norm: " , searchDirection.norm() );
                    m_logger.info("Flow remainder max: " , searchDirection.maxCoeff());
                    m_logger.info("Flow remainder min: ", searchDirection.minCoeff());
					break;
				}
				// Note: objective value is square of differences...
				double diff = std::sqrt(std::abs(newObjVal - objectiveVal));
				objectiveVal = newObjVal;
				// Save the path in the decomposition
				//basis().row(i) = path;

				m_logger.deep("###### Finished " ,(i + 1), "/" ,edgeCount);
                m_logger.deep("New objective value: " ,std::sqrt(newObjVal) );
                m_logger.deep("Diff : ", diff);

				if( diff < m_stopThreshold)
				{
					break;
				}
				
                searchDirection = m_decompObj->residual();

				// Check if initial field is very close to zero. If so, we already found a decomposition.
				if(isApprox(searchDirection.norm(),(NT)0.0))
				{
					std::cout << "Found a decomposition" << std::endl;
					break;
				}
			}
		}

		/**
		 * Refines the decomposition by trying to find a simplex that is closer
		 * to the given flow in state space.
		 */
		void refineDecomposition()
		{
			if (basis().size() != graph()->number_of_edges())
			{
				log() << "Cannot refine" << std::endl;
				return;
			}
			int currSize = basis().size();

            auto decompField = m_decompObj->decompositionField();
			auto searchDirection = LoopsLib::Math::VectorWrapper<NT>(&field()) - LoopsLib::Math::VectorWrapper<NT>(&decompField);
			std::cout << "[StateSpaceDecomposition]\tRefining the decomposition" << std::endl;
			double objectiveVal = searchDirection.normSq();
			// Do smart refining here.
			for(int i = basis().size(); i < graph()->number_of_edges(); i++)
			{
                BasisElement_t path;
				NT newObjVal;
				if (!findPath(searchDirection.toStd(), i, objectiveVal, path, newObjVal))
				{
					// We will stop expanding the basis now.
                    m_logger.info("Stopping basis expansion, can only find degenerate point");
					m_logger.info( "Flow remainder norm: ", searchDirection.norm() );
					m_logger.info("Flow remainder max: ", searchDirection.maxCoeff());
                    m_logger.info("Flow remainder min: " , searchDirection.minCoeff());
					break;
				}
				objectiveVal = newObjVal;
				// Save the path in the decomposition
				//basis().row(i) = path;

                m_logger.deep("###### Finished " ,(i + 1) ,"/" ,graph()->number_of_edges());
				std::cout << "New objective value: " << newObjVal << std::endl;

                searchDirection = m_decompObj->residual();

				// Check if initial field is very close to zero. If so, we already found a decomposition.
				if (isApprox(searchDirection.norm(), (NT)0.0))
				{
					std::cout << "Found a decomposition" << std::endl;
					break;
				}
			}
		}

		LongestPathOracle m_oracle;
		
	public:
		StateSpaceDecomposition(LoopsLib::Models::DecompositionResult* decompObj) : IFlowDecomposition("StateSpace", decompObj),
            m_oracle(LongestPathOracle{ m_decompObj->m_relatedInstance->m_graph })
		{}

		void nextStep() override
		{
			
		}
        LongestPathOracle& longestPathOracle()
		{
            return m_oracle;
		}
	protected:

        void init() override
        {
            IFlowDecomposition::init();
            m_searchDir = m_decompObj->field();
            m_oracle = LongestPathOracle(m_decompObj->m_relatedInstance->m_graph);
        }

		void findNewBasisElements() override
		{
            auto& basis = this->basis();
			initializePathDecomposition();
            m_logger.info("Start refining");
			int initial = basis.size();
			refineDecomposition();
            m_logger.info("Refined with ", (basis.size() - initial), " basis elements");

            m_decompObj->applyNNLSForCoefficients();

            auto diff = m_decompObj->residual();
            LoopsLib::Math::VectorWrapper<NT> diffVec(&diff);

			m_logger.info("Original field total weight:" , diffVec.total() );

			m_logger.info("Diff norm:" ,diffVec.norm() );

			m_logger.info("Decomp values: (value > 0.01");
			for (int i = 0; i < coefficients().size(); i++)
			{
				//if (decompVal(i) > 0.01)
				{
					std::cout << i << ":" << coefficients()[i] << std::endl;
				}
			}
		}

    public:
        std::map<std::string, std::string> metaData() const override
        {
            return {};
        }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override{}
        std::string paramDescription() const override
        {
            return "";
        }
	};
}
#endif