#ifndef DS_GRAPH_H
#define DS_GRAPH_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <ctime>
#include <stack>
#include <algorithm>
#include <set>
#include <Eigen/Eigen>

namespace DS{
    // Adapted from https://github.com/mraggi/LongestSimplePath
	//   /**
 //* Strongly connected component info struct
 //*/
	//struct SccInfo
	//{

	//	/**
	//	 * \brief Structure containing info for strongly connected component
	//	 * \param s Start vertex of the component
	//	 * \param e End vertex of the component
	//	 * \param c The assigned color for the component
	//	 */
	//	SccInfo(int s, int e, int c) : start(s), end(e), color(c) {}
	//	int start;
	//	int end;
	//	int color;
	//};

	//// Forward declare the pseudo topological order class.
	//class PseudoTopoOrder;

	// Parameters?
	//using param_t = std::array<double, 8>;


	using node_t = int;
	using weight_t = double;
	const node_t INVALID_NODE = -1;


	struct NeighborNode
	{
		explicit NeighborNode() : node(INVALID_NODE), weight(0),id(-1) {}

		explicit NeighborNode(node_t v, weight_t w, int id = -1) : node(v), weight(w),id(id){}
		
		inline operator node_t() const
		{
			return node;
		}

		weight_t Weight() const
		{
			return weight;
			// 		return 1;
		}
		int id;
		node_t node;
		weight_t weight{ 1 }; //comment 
	};

	/**
	 * \brief Representation of the directed graph.
	 */
	class DiGraph
	{
	public:
		using VertexIdentifier = node_t;
		explicit DiGraph(node_t numNodes);

		// Empty graph
		DiGraph(){}

		//		Graph modification functions
		int add_edge(node_t from, node_t to);

		int add_edge(node_t from, node_t to, int id);

		void flipEdge(node_t from, node_t to)
		{
			// Find and delete in from node
			int eId = -1;
			{
				auto it = m_outgraph[from].begin();
				for (; it != m_outgraph[from].end(); ++it)
				{
					if (*it == to)
					{
						eId = it->id;
						break;
					}
				}
				m_outgraph[from].erase(it);
			}
			{
				auto it = m_ingraph[to].begin();
				for (; it != m_ingraph[to].end(); ++it)
				{
					if (*it == from)
					{
						break;
					}
				}
				m_ingraph[to].erase(it);
			}
			// Add reverse edge
			m_outgraph[to].emplace_back(from, 1.0, eId);
			m_ingraph[from].emplace_back(to, 1.0, eId);
			// Update degrees
			m_outDegrees[to]++;
			m_outDegrees[from]--;
			m_inDegrees[from]++;
			m_inDegrees[to]--;
		}

        bool is_double_edge(node_t from, node_t to, NeighborNode& doubleEdge)
		{
            // Find the back edge. Does not explicitly check existence of (from,to) edge!
            auto loc = std::find(m_outgraph[to].begin(), m_outgraph[to].end(), from);
            if(loc != m_outgraph[to].end())
            {
                doubleEdge = *loc;
                return true;
            }
            return false;
		}

		// Get Graph Info
		node_t get_size() const { return m_n; }

		/**
		 * \brief Returns the number of vertices
		 * \return Number of vertices
		 */
		node_t num_vertices() const { return m_n; }

		/**
		 * \brief Returns the number of edges
		 * \return The number of edges
		 */
		size_t num_edges() const;

		/**
		 * \brief Get the edge ID for the edge between the start and end node
		 * \param start The start node
		 * \param end The end node
		 * \return The edge ID
		 */
		node_t edgeID(node_t start, node_t end);

		/**
		 * \brief Sets the weights of all edges in the graph
		 * \param weights The weight vector, mapping edge IDs to weights
		 */
		void setWeights(const Eigen::VectorXd& weights);

		/**
		 * \brief Returns the out degree of the given node
		 * \param node The node
		 * \return The outdegree of the node.
		 */
		//size_t outdegree(node_t node) const { return m_outgraph[node].size(); }
		size_t outdegree(node_t node) const { return m_outDegrees[node]; }

		/**
		 * \brief Returns the in degree of the given node
		 * \param node The node
		 * \return The in degree of the node
		 */
		//size_t indegree(node_t node) const { return m_ingraph[node].size(); }
		size_t indegree(node_t node) const { return m_inDegrees[node]; }

		/**
		 * \brief Returns whether the nodes are neighbours
		 * \param a First node
		 * \param b Second node 
		 * \return Are they neighbours?
		 */
		bool are_neighbors(node_t a, node_t b) const;

		inline const std::vector<NeighborNode>& outneighbors(node_t n) const { return m_outgraph[n]; }

		inline const std::vector<NeighborNode>& inneighbors(node_t n) const { return m_ingraph[n]; }

		// 	inline const weight_t edge_value(node_t from, node_t to) const { return m_edge_values(from,to); }

		static DiGraph CreateRandomDiGraph(int n, double p);
		//static DiGraph CreateRandomWeightedDiGraph(int n, double p, weight_t minweight, weight_t maxweight);

		template<typename Pred>
		void sort_outneighbours(Pred p)
		{
			for(int i = 0; i < m_outgraph.size(); i++)
			{
				auto& part = m_outgraph[i];
				std::sort(part.begin(), part.begin() + m_outDegrees[i], p);
			}
		}
		template<typename Pred>
		void sort_inneighbours(Pred p)
		{
			for (int i = 0; i < m_ingraph.size(); i++)
			{
				auto& part = m_ingraph[i];
				std::sort(part.begin(), part.begin() + m_inDegrees[i], p);
			}
		}

		/**
		 * \brief Interpret a separated node as deleted
		 * \param node The node
		 * \return Whether or not the node is deleted.
		 */
		inline bool isDelete(node_t node) const
		{
			return m_inDegrees[node] == 0 && m_outDegrees[node] == 0;
		}

		/**
		 * \brief ''Removes'' the given nodes from the graph: all edges will be deleted.
		 * \param toRemove The nodes to remove
		 */
		void remove_nodes(std::vector<node_t>& toRemove);

		void clear()
		{
			// Number of edges
			m_edgeCount = 0;
			// Number of vertices
			m_n = 0;

			// Outgoing edges per vertex
			m_outgraph.clear();

			// Incoming edges per vertex
			m_ingraph.clear();

			// Explicitly define out degrees. All edges outside the degree range are ignored
			m_outDegrees.clear();
			m_inDegrees.clear();
		}
		void setupNew(int numVertices)
		{
			if (num_vertices() != 0 || num_edges() != 0) clear();
			m_n = numVertices;
			m_outgraph = std::vector<std::vector<NeighborNode>>(numVertices);
			m_ingraph = std::vector<std::vector<NeighborNode>>(numVertices);
			m_outDegrees = std::vector<int>(numVertices, 0);
			m_inDegrees = std::vector<int>(numVertices, 0);
		}
		void writeToFile(std::ofstream& fileStream)
		{
			fileStream << m_n << ' ' << m_edgeCount << std::endl;
			for(int i = 0; i < m_n; ++i)
			{
				const auto& outNeigh = m_outgraph[i];
				for(auto n : outNeigh)
				{
					fileStream << i << ' ' << n.node << ' ' << n.id << std::endl;
				}
			}
		}
		void readFromFile(std::ifstream& fileStream)
		{
			int edgeCount = 0;
			fileStream >> std::skipws;
			fileStream >> m_n >> edgeCount;
			setupNew(m_n);
			for(int i = 0; i < edgeCount; i++)
			{
				int from, to, id;
				fileStream >> from >> to >> id;
				add_edge(from, to, id);
			}
		}
	private:
		// Utils for creating the graph
		

		/**
		 * \brief Returns a copy of the graph with the specified nodes removed.
		 * \param toRemove The nodes to remove
		 * \return The new graph.
		 */
		DiGraph with_nodes_removed(std::vector<node_t>& toRemove) const;

		/**
		 * \brief Disables an edge for the given node
		 * \param node The target node, connected to the edge
		 * \param edgeId The edge id
		 * \param isIn Whether the edge is incoming our outgoing relative to the node.
		 */
		void disableEdge(node_t node, node_t edgeId, bool isIn);

	protected:
		// Number of edges
		int m_edgeCount = 0;
		// Number of vertices
		node_t m_n = 0;

		// Outgoing edges per vertex
		std::vector<std::vector<NeighborNode>> m_outgraph;
		
		// Incoming edges per vertex
		std::vector<std::vector<NeighborNode>> m_ingraph;

		// Explicitly define out degrees. All edges outside the degree range are ignored
		std::vector<int> m_outDegrees;
		std::vector<int> m_inDegrees;
	};

	std::ostream& operator<<(std::ostream& os, const DiGraph& M);
	//std::ostream& operator<<(std::ostream& os, const param_t& a);
}
#endif