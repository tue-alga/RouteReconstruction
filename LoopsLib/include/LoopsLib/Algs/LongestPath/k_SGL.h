#ifndef ALGS_K_SGL_H
#define ALGS_K_SGL_H
#include "ILongestPathAlg.h"
#include <LoopsLib/DS/BaseGraph.h>

namespace LoopsLib::Algs::LongestPath
{
	class k_SGL : public ILongestPath
	{
        using Edge = DS::BaseGraph::Edge;
        using Vertex = DS::BaseGraph::Vertex;

		// Currently occupied vertices. Cannot reuse these
		Eigen::VectorXi m_forbidden;

		// Reachability of vertices from the sink. Unreachable vertices cannot be used.
		Eigen::VectorXi m_reachable;

        FieldType m_weights;

		std::vector<Edge*> m_best;
		std::vector<int> m_pathNodeParentInds;
		// Find with best weight
		double m_bestWeight = 0.0;

        // Was a path found.
		bool m_found = false;
		// Lookahead number
		int m_k = 0;

    public:
        k_SGL(DS::BaseGraph* graph);

    private:
		/**
		 * \brief Determine if the vertex can be in the path. Requires
		 * that the vertex has not been used yet and can reach the sink
		 * \param vertex The vertex
		 * \return 
		 */
		inline bool isVertexUsable(int vertex)
		{
			return m_forbidden(vertex) == 0 && m_reachable(vertex) == 1;
		}

        /**
         * \brief Find the best path of size k from the given source, taking into account forbidden vertices. Returns a shorter
         * path if the global sink was found.
         * \param source Local source to start at
         * \param maxWeight Maximum weight allowed. Path weights should be strictly less than this.
         */
        bool bruteForceBestKPath(int source, std::vector<Edge*>& bestPath,
                                 double maxWeight = std::numeric_limits<double>::max());


        /**
         * \brief Apply the algorithm
         */
        void applyKSGL(double maxWeight);

        /**
         * \brief Struct holding a segment of length K.
         */
        struct KSegment
        {
            // Start vertex of the segment
            Vertex* start;
            // List of selected edges
            std::vector<Edge*> edges;
            // Pointer to the weights on the graph
            FieldType* m_weights;
            // The maximum allowed weight
            double maxWeight = std::numeric_limits<double>::max();

            /**
             * \brief 
             * \param weights Pointer to global weights to optimize for.
             */
            KSegment(FieldType* weights = nullptr);

            /**
             * \brief Compute the total weight of the segment
             * \return The weight
             */
            double weight() const;
        };
        std::vector<KSegment> m_path;

	public:
        /**
         * \brief Sets the lookahead ("k") value 
         * \param k The value.
         */
        void setK(int k);

		/**
		 * \brief Computes the longest path approximately. 
		 * \param graph Directed graph to use
		 * \param field Weights for the graph edges
		 * \param source Index of the source vertex
		 * \param sink Index of the sink vertex
		 * \return Vector representation of approximate longest path. (for each edge, 1  if it is in the path, 0 otherwise).
		 */
    protected:
        BasisElement computeLongestPath(const FieldType& field, NT maxWeight) override;
    public:
        BasisElement retry(const FieldType& field) override;
	};
}

#endif