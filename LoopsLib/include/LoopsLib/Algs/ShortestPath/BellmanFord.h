#ifndef BELLMAN_FORD_H
#define BELLMAN_FORD_H
#include <vector>

/**
 * \brief O(V\cdot E) algorithm for finding shortest paths. Supports negative edge weights. 
 * \tparam Graph 
 * \tparam Scalar 
 * \tparam EdgeWeight 
 */
template<typename Graph, typename Scalar, typename EdgeWeight>
class BellmanFord
{
public:
	using Index = typename Graph::IndexType;
private:
	Graph* m_target;
	std::vector<Scalar>* m_distances;
	std::vector<Index>* m_predecessors;
	Index m_source = -1;

	void constructShortestPath()
	{
		
	}

public:
	/**
	 * \brief Sets up the algorithm
	 * \param target The graph to use
	 * \param source The index of the source vertex
	 * \param distances Output: The distance for each vertex to the source
	 * \param predecessors Output: The index of the predecessor for each vertex
	 */
	BellmanFord(Graph* target, Index source, std::vector<Scalar>* distances, std::vector<Index>* predecessors): 
	m_target(target),
	m_source(source),
	m_distances(distances),
	m_predecessors(predecessors)
	{
		
	}

	/**
	 * \brief Returns the current source vertex index
	 * \return The source vertex index
	 */
	Index source() const
	{
		return m_source;
	}

	void setSource(Index source)
	{
		m_source = source;
	}

	void apply()
	{
		std::vector<Scalar>& distances = *m_distances;
		std::vector<Index>& predecessors = *m_predecessors;
		// Initialize distances to infinity
		(*m_distances).resize(m_target->vertexCount(), std::numeric_limits<Scalar>::max());
		// Initialize predecessors to negative values
		(*m_predecessors).resize(m_target->vertexCount(), -1);

		Index vertCount = m_target->vertexCount();
		// Initialize
		distances[m_source] = 0;

		// Repeat 
		for(Index i = 0; i < vertCount-1; i++)
		{
			for(const auto& edge: m_target->edges())
			{
				auto weight = m_target->edgeWeight(edge);
				if(distances[edge.start()] + weight < distances[edge.end()])
				{
					distances[edge.end()] = distances[edge.start()] + weight;
					predecessors[edge.end()] = edge.start();
				}
			}
		}
		for(const auto& edge: m_target->edges())
		{
			if(distances[edge.start()] + m_target->edgeWeight(edge) < distances[edge.end()])
			{
				throw std::runtime_error("Found negative-weight cycle in graph");
			}
		}
	}
};

#endif