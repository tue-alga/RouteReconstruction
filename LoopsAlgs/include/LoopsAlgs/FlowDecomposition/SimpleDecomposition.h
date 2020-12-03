#ifndef SIMPLE_DECOMPOSITION_H
#define SIMPLE_DECOMPOSITION_H
#include <vector>
#include <tuple>
#include <LoopsLib/Algs/ShortestPath/BellmanFord.h>
#include <queue>
#include <unordered_set>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <LoopsLib/Algs/Types.h>

namespace LoopsAlgs::FlowDecomposition
{
		///**
		// * \brief Algorithm for a simple approach to path decomposition.
		// * The algorithm decomposes the flow in shortest paths initially.
		// * All remaining paths are constructed via 
		// * \tparam Graph 
		// * \tparam FlowField 
		// */
		//template<typename LongestPathOracle>
		//class SimpleDecomposition
		//{
		//public:
		//	using Scalar = double;
		//	using VertexDescriptor = boost::graph_traits<GraphType>::vertex_descriptor;

		//	using FlowMap = boost::property_map<GraphType, boost::edge_weight_t>;
		//	using Path = std::vector<VertexDescriptor>;
		//private:
		//	GraphType* m_target;
		//	std::vector<std::tuple<Scalar, Path>>* m_decomposition;
		//	// Used flow field. Will be modified by the algorithm.
		//	FlowMap m_field; // Should be something like boost::property_map<GraphType, boost::edge_weight_t>
		//	VertexDescriptor m_source, m_sink;

		//	static Path longestPathWithWeights(const GraphType& g, const FlowMap& flowMap)
		//	{
		//		
		//	}

		//	Scalar findPathHardConstraint(GraphType& g, FlowMap& flowMap)
		//	{

		//	}

		//	std::vector<std::tuple<Scalar,Path>>& decomposition()
		//	{
		//		return *m_decomposition;
		//	}

		//	void initializePathDecomposition()
		//	{
		//		std::vector<int> edgeIndices;
		//		boost::property_map<GraphType, boost::edge_index_t> indices = boost::get(*m_target, boost::edge_index_t{});
		//		// Iterate over edges
		//		// for(const auto& edge: edges){
		//		// Get associated edge vector
		//		// Find direction to search in (towards, difference with last, sth)
		//		// Maybe reject paths when needed (expanded graph case).
		//		//}
		//	}

		//	/**
		//	 * \brief Finds a flow path through the given edge.
		//	 * \param edgeIndex The edge index.
		//	 * \param outputPath The found path
		//	 * \return The amount of flow along the path.
		//	 */
		//	Scalar findPath(Index edgeIndex, Path& outputPath)
		//	{
		//		boost::property_map<GraphType, boost::edge_weight_t> flowMap;
		//		auto edges = boost::edges(*m_target);
		//		// Assign the flow values in the flowMap
		//		for(auto eIt = edges.first; eIt < edges.second; ++eIt)
		//		{
		//			boost::put(flowMap, *eIt, m_field[*eIt]);
		//		}
		//		// Find path from source to start and find path from end to sink.

		//		std::vector<VertexDescriptor> predec(boost::num_vertices(*m_target));
		//		// Perform a shortest path search.
		//		boost::dijkstra_shortest_paths(*m_target, source,
		//			boost::predecessor_map(boost::make_iterator_property_map(predec.begin(), boost::get(boost::vertex_index, *m_target)))
		//			.weight_map(flowMap)
		//		);

		//	}
		//	LongestPathOracle m_oracle;
		//public:
		//	SimpleDecomposition(GraphType* target, std::vector<std::tuple<Scalar, Path>>* decomposition, FlowMap* inField, VertexDescriptor source, VertexDescriptor sink, LongestPathOracle oracle) :
		//		m_field(*inField),
		//		m_target(target),
		//		m_decomposition(decomposition),
		//		m_source(source),
		//		m_sink(sink),
		//		m_oracle(oracle)
		//	{}

		//	void apply()
		//	{
		//		struct EdgeComp
		//		{
		//			FlowField* m_field;
		//			EdgeComp(FlowField* field) :m_field(field) {}
		//			bool operator()(const EdgeType& e1, const EdgeType& e2)
		//			{
		//				return (*m_field)[e1] > (*m_field)[e2];
		//			}
		//		};
		//		// Priority queue on edges, sorted from max to min flow value
		//		std::unordered_set<Index> edgeIndices;
		//		for (Index ind : m_target->edgeIndices())
		//		{
		//			edgeIndices.insert(ind);
		//		}

		//		while (edgeIndices.size() > 0)
		//		{
		//			// Pick some element in there
		//			Index i = *edgeIndices.begin();

		//		}

		//	}
		//};
}


#endif
