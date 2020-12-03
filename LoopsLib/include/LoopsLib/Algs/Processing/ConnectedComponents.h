#ifndef ALHS_PROCESSING_CONNECTED_COMPONENTS_H
#define ALHS_PROCESSING_CONNECTED_COMPONENTS_H
#include <vector>
#include <LoopsLib/DS/Graph.h>

namespace LoopsLib::Algs::Processing{
    class ConnectedComponentsFinder{
		using node_t = int;
		std::vector<std::vector<node_t>> m_strongly_connected_components;
		std::vector<node_t> m_scc_coloring;

		std::vector<short> m_weak_coloring;
		std::vector<std::vector<node_t>> m_weakly_connected_components;

		DS::DiGraph* m_graph = nullptr;

	public:
		explicit ConnectedComponentsFinder(DS::DiGraph* graph)
			: m_graph(graph)
		{
		}

	private:
		void DFSUtilWeak(node_t node, int color)
		{
			m_weak_coloring[node] = color;
			m_weakly_connected_components.back().push_back(node);

			// 	m_weakly_connected_components_values.back() += vertex_values[node]-1;
			for (auto x : m_graph->outneighbors(node))
			{
				if (m_weak_coloring[x] != -1)
				{
					continue;
				}

				DFSUtilWeak(x, color);
			}

			for (auto x : m_graph->inneighbors(node))
			{
				if (m_weak_coloring[x] != -1)
				{
					continue;
				}

				DFSUtilWeak(x, color);
			}
		}

		void findWeaklyConnected()
		{
			const int vCount = m_graph->num_vertices();
			m_weak_coloring.clear();
			m_weak_coloring.resize(vCount, -1);
			m_weakly_connected_components.clear();

			// 	m_weakly_connected_components_values.clear();
			int minvalidcoloring = 0;

			for (int start = 0; start < vCount; ++start)
			{
				if (m_weak_coloring[start] != -1)
				{
					continue;
				}

				m_weakly_connected_components.emplace_back();
				// 		m_weakly_connected_components_values.push_back(0);
				DFSUtilWeak(start, minvalidcoloring);

				++minvalidcoloring;
			}
		}

		void topo_fill_order(node_t v, std::vector< char >& visited, std::stack< node_t >& Stack)
		{
			// Mark the current node as visited and print it
			visited[v] = 1;

			// Recur for all the vertices outgraphacent to this vertex
			for (auto i : m_graph->outneighbors(v))
			{
				if (visited[i] == 0)
				{
					topo_fill_order(i, visited, Stack);
				}
			}

			// All vertices reachable from v are processed by now, push v
			Stack.push(v);
		}

		void DFSUtilReversed(node_t v, std::vector< char >& visited, int current)
		{
			visited[v] = 1;
			// 	std::cout << " v = " << v << " and current = " << current << " and scc.size() = " << strongly_connected_components.size() << std::endl;
			m_strongly_connected_components[current].push_back(v);
			m_scc_coloring[v] = current;

			// 	std::cout << "good" << std::endl;
			for (auto i : m_graph->inneighbors(v))
			{
				if (visited[i] == 0)
				{
					DFSUtilReversed(i, visited, current);
				}
			}
		}

		void findStronglyConnectedComponents()
		{
			const int vCount = m_graph->num_vertices();
			// Prepare for the strongly connected components
			m_strongly_connected_components.clear();
			m_strongly_connected_components.reserve(vCount / 4);

			// Resize the coloring.
			m_scc_coloring.resize(vCount);

			std::stack<node_t> Stack;

			std::vector<char> visited(vCount, 0);

			for (int i = 0; i < vCount; ++i)
			{
				if (!static_cast<bool>(visited[i]))
				{
					topo_fill_order(i, visited, Stack);
				}
			}

			// Reset to zero.
			std::fill_n(visited.begin(), vCount, 0);

			int current = 0;

			// 	std::cout << "before starting, scc.size() = " << m_strongly_connected_components.size() << std::endl;
			while (!Stack.empty())
			{
				int v = Stack.top();
				Stack.pop();

				m_strongly_connected_components.emplace_back();

				if (!static_cast<bool>(visited[v]))
				{
					DFSUtilReversed(v, visited, current);
					// 			std::cout << std::endl;
					++current;
					m_strongly_connected_components.emplace_back();
				}
			}

			//     std::cout << "Finished first loop" << std::endl;

			// Remove empty components
			while (m_strongly_connected_components.back().empty())
			{
				m_strongly_connected_components.pop_back();
			}
		}
    public:

		void process()
		{
			findWeaklyConnected();
			findStronglyConnectedComponents();
		}

		/**
		 * \brief Returns the strongly connected components
		 * \return The strongly connected components.
		 */
		const std::vector<std::vector<node_t>>& sccComponents() const
		{
			return m_strongly_connected_components;
		}
		int sccCount() const
		{
			return m_strongly_connected_components.size();
		}
		const std::vector<node_t>& sccColoring() const
		{
			return m_scc_coloring;
		}

		const std::vector<short>& wccColoring() const
		{
			return m_weak_coloring;
		}
		const std::vector<std::vector<node_t>>& wccComponents() const
		{
			return m_weakly_connected_components;
		}
		int wccCount() const
		{
			return m_weakly_connected_components.size();
		}

    };
}
#endif