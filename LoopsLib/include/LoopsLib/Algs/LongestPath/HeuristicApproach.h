#ifndef LOINGEST_PATH_HEURISTIC_APPROACH_H
#define LOINGEST_PATH_HEURISTIC_APPROACH_H
#include <Eigen/Eigen>
#include <iostream>
#include <LoopsLib/DS/Graph.h>
#include <LoopsLib/Parts/Path.h>
#include <LoopsLib/Parts/PseudoTopoOrder.h>
#include <LoopsLib/Algs/Processing/ConnectedComponents.h>
#include <array>

namespace Algs::LongestPath{
	// Parameters for searching
	using param_t = std::array<double, 8>;
	using sumweight_t = double;
	using weight_t = double;

	template<typename InputIterator, typename TargetType, typename Func>
	TargetType reduce(InputIterator start, InputIterator end, Func reducer, TargetType initial = {})
	{
		TargetType output = initial;
		int curr = 0;
		while (start != end) {
			// Process the output type by reference.
			reducer(output, curr, *start);
		}
		return output;
	}
	
	class HeuristicApproach {

		using node_t = DS::node_t;

		/**
		 * Strongly connected component info struct
		 */
		struct SccInfo
		{

			/**
			 * \brief Structure containing info for strongly connected component
			 * \param s Start vertex of the component
			 * \param e End vertex of the component
			 * \param c The assigned color for the component
			 */
			SccInfo(int s, int e, int c) : start(s), end(e), color(c) {}
			int start;
			int end;
			int color;
		};

		// The target graph
		DS::DiGraph* m_graph;

		// Number of vertices in graph
		int m_vertNum;

		// Best weight value for path.
		mutable sumweight_t m_globalBest = -1;

		// Is the longest path constrained to fixed source and sink?
		bool m_isConstrained = true;

		//Heuristics
		std::vector<double> m_heuristic_out;
		std::vector<double> m_heuristic_in;

		Algs::Processing::ConnectedComponentsFinder m_ccs;

		std::vector<node_t> m_basic_topological_ordering;
		std::vector<node_t> m_basic_topological_ordering_in;
		std::vector<node_t> m_basic_topological_ordering_inverse;
		std::vector<node_t> m_basic_topological_ordering_inverse_in;

		std::vector<SccInfo> m_scc_big_components;

		// Connected Components
		std::vector<node_t> m_scc_rank_out; // This is the ex rank of the connected components
		std::vector<node_t> m_scc_rank_in; // This is the in rank of the connected components

		// The weights of the edges.
		Eigen::VectorXd m_weights;

		// Was the graph preprocessed?
		bool m_processed = false;
		// Source and sink vertex
		node_t m_source, m_sink;

		param_t m_params = {};

#pragma region  Methods

		inline void assertSourceSinkPath(const Path& p) const
		{
			assert(p.front().node == m_source);
			assert(p.back().node == m_sink);
		}
		inline bool isSourceSinkPath(const Path& p) const
		{
			return p.front().node == m_source && p.back().node == m_sink;
		}

		static param_t GetParams(int i);

		/**
		 * \brief Builds the ranks from the calculated connected components in m_ccs.
		 */
		void buildRanks();

		/**
		 * \brief Expands the end of the path by performing a DFS on the neighbours that weren't explored
		 * yet. 
		 * \param P The path to expand
		 */
		void ExpandGreedyBack(Path& P) const;

		/**
		 * \brief Expands the start of the path by performing a DFS on the neighbours that weren't explored
		 * yet.
		 * \param P The path to expand
		 */
		void ExpandGreedyFront(Path& P) const;
		
		/**
		 * \brief This is the order in which the outneighbors are sorted
		 * \param a The first node
		 * \param b The second node
		 * \return Returns whether the first node comes before the second or not.
		 */
		inline bool ex_compare(node_t a, node_t b) const { return m_basic_topological_ordering_inverse[a] < m_basic_topological_ordering_inverse[b]; }

		// This is the order in which the outneighbors are sorted
		inline bool in_compare(node_t a, node_t b) const { return m_basic_topological_ordering_inverse_in[a] < m_basic_topological_ordering_inverse_in[b]; }

		
		/**
		 * DFS searching
		 */
		Path dfs_search();

		/**
		 * \brief Perform the full DFS in the forward direction of the graph.
		 * \return The found path
		 */
		Path dfs_forward_full() const;

		/**
		 * \brief Perform the full DFS in the backward direction of the graph.
		 * \return The found path
		 */
		Path dfs_backward_full();

		/**
		 * \brief Search forward, given start node
		 * \param start The start node
		 * \param maxNumSeconds Maximum number of seconds to spend refining the path
		 * \return The found forward path
		 */
		Path dfs_search_path_forward(node_t start, double maxNumSeconds) const;

		/**
		 * \brief Search backward, given start node
		 * \param start The start node
		 * \param maxNumSeconds Maximum number of seconds to spend refining the path
		 * \return The found forward path
		 */
		Path dfs_search_path_backward(node_t start, double maxnumseconds) const;

		void dfs_search_path_forward(Path& P, double maxnumseconds) const;

		void dfs_search_path_backward(Path& P, double maxnumseconds) const;

		double get_heuristic_out(node_t node);

		double get_heuristic_in(node_t node);

		/**
		 * \brief Compute heuristics.
		 */
		void heuristic_processing();

		/**
		 * \brief Sets parameters for the heuristics.
		 * \param new_params The heuristic parameters.
		 */
		void set_parameters(const param_t& new_params);

		/**
		 * \brief Removes bad nodes from the graph. Essentially removes all but the largest weakly
		 * connected components.
		 */
		void remove_bad_nodes();

		/**
		 * \brief Selects next neighbour to explore, or backtracks until
		 * unexplored neighbours are present for an ancestor
		 * \tparam Compare Comparison alg for searching explored node
		 * \param P The path (''stack'' of the DFS)
		 * \param comp The comparison operator for nodes
		 * \return Whether a new node was found. False means DFS is done.
		 */
		template <class Compare>
		bool dfs_outnext(Path& P, Compare comp) const
		{
			auto lastNode = P.back();

			auto Neighs = &m_graph->outneighbors(lastNode);

			auto t = P.first_not_explored((*Neighs).begin(), (*Neighs).end());

			while (t == DS::INVALID_NODE && P.size() > 1) //this means all nodes in Neigh have been explored
			{
				lastNode = P.back();
				P.pop_back();
				int father = P.back();
				Neighs = &m_graph->outneighbors(father);
				t = P.first_not_explored_binary((*Neighs).begin(), (*Neighs).end(), lastNode, comp);
			}

			if (t == DS::INVALID_NODE)
				return false; // this means we have finished DFS!!

			P.push_back(t);
			ExpandGreedyBack(P);
			return true;
		}

		/**
		 * \brief Similar to dfs_outnext, only traversing the graph in the reverse direction
		 * \tparam Compare Comparison alg for searching explored node
		 * \param P The path (''stack'' of the DFS)
		 * \param comp The comparison operator for nodes
		 * \return Whether a new node was found. False means DFS is done.
		 */
		template <class Compare>
		bool dfs_innext(Path& P, Compare comp) const
		{
			auto firstNode = P.front();

			auto Neighs = &(this->m_graph->inneighbors(firstNode));

			auto t = P.first_not_explored((*Neighs).begin(), (*Neighs).begin() + m_graph->indegree(firstNode));

			while (t == DS::INVALID_NODE && P.size() > 1) //this means all nodes in Neigh have been explored
			{
				firstNode = P.front();
				P.pop_front();
				const int parent = P.front();
				Neighs = &(m_graph->inneighbors(parent));
				t = P.first_not_explored_binary((*Neighs).begin(), (*Neighs).end(), firstNode, comp);
			}

			if (t == DS::INVALID_NODE)
				return false; // this means we have finished DFS!!

			P.push_front(t);
			ExpandGreedyFront(P);
			return true;
		}

		/**
		 * \brief Performs a search using the pseudo topological order
		 * \param A The path to assign to.
		 */
		void pto_search(Path& A) const;
#pragma endregion
    public:

		HeuristicApproach(DS::DiGraph* graph, const Eigen::VectorXd& weights, node_t source, node_t sink):
		m_graph(graph),
		m_weights(weights),
		m_vertNum(graph->num_vertices()),
		m_ccs(graph),
		m_source(source),
		m_sink(sink)
		{
			m_graph->setWeights(weights);
		}

		int rank_out(node_t node) const { return m_scc_rank_out[m_ccs.sccColoring()[node]]; }

		int rank_in(node_t node) const { return m_scc_rank_in[m_ccs.sccColoring()[node]]; }

		/**
		 * \brief Retrieve the weight for the given edge
		 * \param edgeId The ID of the edge
		 * \return The weight.
		 */
		double edgeWeight(int edgeId) const
		{
			if (edgeId < 0) return 0;

			return m_weights(edgeId);
		}

		/**
		 * \brief Options for the search heuristics
		 */
		struct SearchOptions
		{
			// DFS part
			int dfs_num_parameter_restarts{ 3 };
			double dfs_time_woimprovement{ 0.05 };
			int dfs_forward_num_starting_nodes{ 3 };
			int dfs_backward_num_starting_nodes{ 1 };

			int dfs_how_many_to_erase_from_opposite_side{ 20 };
			// 		int num_saved_for_pto {1};

			// PTO part
			double pto_time_without_improvement{ 2.0 };
			int pto_num_times_restart{ 1 };
			int pto_num_heuristic_sort{ 10000 };
			int pto_scc_size_max_pointless{ 4 };
		};

		inline const std::vector<SccInfo>& big_scc() const { return m_scc_big_components; }
		inline const std::vector<std::vector<node_t>>& strongly_connected_components() const { return m_ccs.sccComponents(); }

		/**
		 * \brief The used graph
		 * \return The graph
		 */
		DS::DiGraph* graph() const
		{
			return m_graph;
		}

		SearchOptions Options{};
		
		/**
		 * \brief Preprocesses the graph. This is independent of the edge weights.
		 */
		void process();

		/**
		 * \brief Finds the longest path in the specified graph using the given heuristics.
		 * \return 
		 */
		Path findLongestPath();

		
		/**
		 * \brief Retrieves a random topological ordering of the strongly connected components
		 * \return The random topological ordering.
		 */
		Algs::LongestPath::PseudoTopoOrder get_random_pseudotopological_order() const;
	};
	class LongestPathHeuristic
	{
	public:
		Eigen::VectorXd getLongestPath(DS::DiGraph* graph, const Eigen::VectorXd& field, int source, int sink)
		{
			HeuristicApproach ha(graph, field, source, sink);
			Path p = ha.findLongestPath();
			// Convert back to vector
			Eigen::VectorXd outVec;
			outVec.setConstant(graph->num_edges(), 0);
			for(int i = 0; i < p.size()-1; i++)
			{
				const int eId = graph->edgeID(p[i].node, p[i + 1].node);
				outVec(eId) = 1;
			}
			return outVec;
		}
	};
}

inline std::ostream& operator<<(std::ostream& os, const Algs::LongestPath::param_t& a)
{
	int i = 0;
	for (auto x : a)
	{
		os << x << ' ';
		if (i == 4)
			os << '|';
		++i;
	}
	return os;
}
#endif