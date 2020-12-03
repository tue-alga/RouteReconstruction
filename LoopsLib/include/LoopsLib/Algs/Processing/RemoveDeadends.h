#ifndef REMOVE_DEADENDS_H
#define REMOVE_DEADENDS_H
#include <LoopsLib/DS/OwnGraph.h>

namespace LoopsLib::Algs
{
	namespace Processing
	{
		/**
		 * Cleans up the graph by removing any potential source or sink nodes
		 * that are not the specified source or sink nodes.
		 */
		class RemoveDeadends
		{
		public:
			void apply(const DS::BaseGraph& targetGraph, int sourceNode, int sinkNode, DS::BaseGraph& output, int& newSource, int& newSink, std::set<int>& removedNodes)
			{
				using namespace std;
				std::set<int> removeNodes;
				std::vector<int> remap(targetGraph.number_of_vertices(), -1);
				int targetMap = 0;
				for(int i = 0; i < targetGraph.number_of_vertices(); ++i)
				{
					if(targetGraph.vertex(i)->m_inEdges.size() == 0 && i != sinkNode)
					{
						removeNodes.insert(i);

					}
					else if (targetGraph.vertex(i)->m_outEdges.size() == 0 && i != sourceNode)
					{
						removeNodes.insert(i);
					}
					else
					{
						remap[i] = targetMap;
						++targetMap;
					}
				}
				cout << "[DeadendsRemoval]\tFound " << removeNodes.size() << " deadends to remove " << endl;

				output = DS::BaseGraph(targetGraph.number_of_vertices() - removeNodes.size());
				for (int i = 0; i < targetGraph.number_of_vertices(); ++i)
				{
					if(removeNodes.find(i) == removeNodes.end())
					{
						for(auto el : targetGraph.vertex(i)->m_outEdges)
						{
							if(removeNodes.find(el->m_sink->id()) == removeNodes.end())
							{
								output.addEdge(remap[i], remap[el->m_sink->id()]);
							}
						}
					}
				}
				newSource = remap[sourceNode];
				newSink = remap[sinkNode];
				
			}
		};
	}
}
#endif