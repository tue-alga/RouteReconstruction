#ifndef ALGS_PROCESSING_REACHABILITYLABELING_H
#define ALGS_PROCESSING_REACHABILITYLABELING_H
#include <LoopsLib/DS/OwnGraph.h>
#include <queue>
#include <iostream>

namespace LoopsLib::Algs{
	namespace Processing{
		class ReachabilityLabeling
		{
			DS::BaseGraph* m_graph;
		public:
			ReachabilityLabeling(DS::BaseGraph* graph): m_graph(graph){}

			void apply(int sink, std::vector<int>& output)
			{
				// Initialize output with very large number
				output.resize(m_graph->number_of_vertices(), std::numeric_limits<int>::max()-3);
				// Set sink distance to self
				output[sink] = 0;

				// Setup visited array.
				std::vector<bool> visited;
				visited.resize(m_graph->number_of_vertices(), false); // Initialize to none visited
				visited[sink] = true; // Sink is visited

				// Add predecessor to processing queue.
				auto currentNode = sink;
				std::queue<int> toProcess;
				for (auto n : m_graph->vertex(sink)->m_inEdges) toProcess.push(n->m_source->id());

				int cnt = 0;
				while(toProcess.size() > 0)
				{
					// Grab next predecessor.
					auto curr = toProcess.front();
					toProcess.pop();
                    auto* currVert = m_graph->vertex(curr);
					// Find out smallest distance to sink
					int min = std::numeric_limits<int>::max() - 3;
					for(auto n : currVert->m_outEdges)
					{
						min = std::min(output[n->m_sink->id()] + 1, min);
					}
					std::cout << min << std::endl;
					if(min <= m_graph->number_of_vertices())
					{
						cnt++;
					}
					// Save the value
					output[curr] = min;
					// Recurse on predecessors.
					for(const auto& n : currVert->m_inEdges)
					{
                        auto nId = n->m_source->id();
						if (visited[nId]) continue;
						visited[nId] = true;
						toProcess.push(nId);
					}
				}
				std::cout << "Total reachable vertices:" << cnt << std::endl;
			}
		};
	}
}
#endif
