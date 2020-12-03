#ifndef ALGS_IS_REACHABLE_H
#define ALGS_IS_REACHABLE_H
#include <LoopsLib/DS/OwnGraph.h>
#include "BFS.h"
#include <functional>
// Describe something
#define _D(x) 
namespace LoopsLib::Algs::Processing
{
	class IsReachable
	{
		// For BFS
		struct GraphWrapper
		{
			DS::BaseGraph* graph;
			GraphWrapper(DS::BaseGraph* g):graph(g){}
			using VertexIdentifier = DS::BaseGraph::Id_t;

            VertexIdentifier vertexCount() const
			{
				return graph->number_of_vertices();
			}
			std::vector<DS::BaseGraph::Id_t> adjacent(VertexIdentifier node) const
			{
                std::vector<VertexIdentifier> adjs;
                for(auto e : graph->vertex(node)->m_outEdges)
                {
                    adjs.push_back(e->m_sink->id());
                }
                return adjs;
			}
		};
	public:
        using Id_t = DS::BaseGraph::Id_t;
		bool apply(DS::BaseGraph* graph, Id_t source, Id_t destination)
		{
			std::function<bool(int)> goalPred;
			std::function<void(int _D(CurrentVertex), int _D(NextVertex))> discoveryAction;
			bool found = false;
			goalPred = [destination](int node)->bool
			{
				return node == destination;
			};
			discoveryAction = [&found,destination](int node,int nextNode)
			{
				if (nextNode == destination) found = true;
			};
			BFS<GraphWrapper, decltype(goalPred), decltype(discoveryAction)> bfs(goalPred, discoveryAction);
			GraphWrapper wrap(graph);
			bfs.apply(wrap, source);
			return found;
		}
	};
}
#endif