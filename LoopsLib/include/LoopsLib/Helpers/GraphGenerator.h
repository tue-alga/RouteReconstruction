#ifndef SRC_HELPERS_GRAPHGENERATOR_H
#define SRC_HELPERS_GRAPHGENERATOR_H
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/Algs/Types.h>
#include <unordered_set>

namespace src{
	namespace Helpers{
		//class GraphGenerator
		//{
		//	template <class T1, class T2>
		//	struct PairHash
		//	{
		//		std::size_t operator() (const std::pair<T1, T2> &pair) const
		//		{
		//			return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
		//		}
		//	};
  //          // Number of vertices to use
		//	int m_vertNum = 100;
  //          // Probability of flipping an edge
		//	double m_flipProbability = 0.5;
  //          // Whether or not to flip edges
		//	bool m_flipEdges = true;

		//	void randomFlipEdges(DS::BaseGraph& graph);

		//	void postProcess(std::unordered_set<std::pair<int,int>, PairHash<int,int>>& edges, int source, int sink, DS::BaseGraph& graph, std::unordered_set<int>& removedVerts);
		//public:
		//	GraphGenerator(int vertexNum) : m_vertNum(vertexNum){}

  //          /**
		//	 * \brief Returns the probability of flipping an edge in the generated graph
		//	 * \return The flip probability
		//	 */
		//	double flipProbability() const
		//	{
		//		return m_flipProbability;
		//	}

		//	void setFlipProbability(double prob)
		//	{
		//		m_flipProbability = prob;
		//	}

		//	/**
		//	 * \brief Sets the number of vertices in the resulting graph
		//	 * \param vertexNum The number of vertices
		//	 */
		//	void setNumberOfVertices(int vertexNum);

		//	/**
		//	 * \brief 
		//	 * \param graphDensity 
		//	 * \param outSource 
		//	 * \param outSink 
		//	 * \param outGraph 
		//	 * \param point 
		//	 */
		//	void generateGraphFromTriangulation(double graphDensity, int& outSource, int& outSink,
		//	                                    DS::BaseGraph& outGraph, std::vector<LoopsLib::Kernel::Point_2>& point);

  //          void twoWayPlanar(DS::BaseGraph& outGraph, std::vector<LoopsLib::MovetkGeometryKernel::MovetkPoint>& points);
  //          /**
		//	 * \brief Generates an (undirected) planar graph
		//	 * \param source Output source
		//	 * \param sink Output sink
		//	 * \param outGraph Output graph
		//	 * \param points Output points
		//	 */
		//	void twoWayPlanar(DS::BaseGraph& outGraph, std::vector<LoopsLib::Kernel::Point_2>& points);

		//	/**
		//	 * \brief 
		//	 * \param graphDensity 
		//	 * \param maxNeigh 
		//	 * \param outSource 
		//	 * \param outSink 
		//	 * \param outGraph 
		//	 * \param point 
		//	 */
		//	void generateGraphFromTriangulationWithNeighbours(double graphDensity, int maxNeigh, int& outSource, int& outSink,
		//		DS::BaseGraph& outGraph, std::vector<LoopsLib::Kernel::Point_2>& point);

		//	/**
		//	 * \brief Generates a random directed graph with no embedding info.
		//	 * Makes sure that the source and sink node are connected by atleast a single 
		//	 * directed path.
		//	 * \param targetGraphDensity Target density (percentage of number of edges relative to complete graph)
		//	 * \param source The source node
		//	 * \param sink The sink node
		//	 * \param outputGraph Output graph
		//	 */
		//	void generateRandomGraph(double targetGraphDensity, int& source, int& sink, DS::BaseGraph& outputGraph);
		//};
	}
}
#endif
