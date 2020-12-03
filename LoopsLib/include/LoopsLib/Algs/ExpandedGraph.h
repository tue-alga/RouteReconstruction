#ifndef ALGS_EXPANDED_GRAPH_H
#define ALGS_EXPANDED_GRAPH_H
#include <vector>
#include <LoopsLib/DS/BaseGraph.h>
#include <LoopsLib/Algs/Types.h>

namespace LoopsLib::Algs
{
    //class ExpandedGraph : public 

	class ExpandedGraph
	{
        // Base network graph
        DS::BaseGraph* m_baseGraph;
	public:
		ExpandedGraph(DS::BaseGraph* baseGraph):m_baseGraph(baseGraph){}


        /**
		 * \brief 
		 * Actually works only for the ''one-sided'' weak Frechet: we can walk back and forth on the trajectory, 
		 * but not on the constructed path.
		 * TODO: also add vertices for levels
		 * \param trajectory 
		 * \param vertexLocations 
		 * \param epsilon A
		 * \param output 
		 * \param labeling 
		 */
		void apply(const std::vector<Point2>& trajectory, const std::vector<Point2>& vertexLocations, double epsilon, DS::BaseGraph& output, std::vector<int>& labeling)
		{
            // Build the freespace ''gridgraph'' structure on top of the network.

            // The grid graph has a vertex for each freespace cell, and an edge between vertices if the 
            // freespace cells share a boundary and are connected by free space along that boundary.
            // Factually, we only need to check endpoint-linesegment distances.
            // We are going to pretend that our graph is undirected though.
            double epsSquared = epsilon * epsilon; // Subject to roundoff error.

            // Let's dumbly construct all vertices for now...

            const int vertsPerEdge = trajectory.size() - 1;
            // Last 2 vertices are super source/sink
            output.allocateVertices( vertsPerEdge * m_baseGraph->number_of_edges() + 2);
            auto* superSource = output.vertex(output.number_of_vertices() - 2);
            auto* superSink = output.vertex(output.number_of_vertices() - 1);

            std::unordered_set<DS::BaseGraph::Id_t> sourceVertices;
            std::unordered_set<DS::BaseGraph::Id_t> sinkVertices;
            auto levelVertex = [&output, vertsPerEdge](DS::BaseGraph::Edge* e, int level)
            {
                return output.vertex(e->id() * vertsPerEdge + level);
            };

            // Setup segments for the trajectory
            std::vector<CGAL::Segment_2<Kernel>> trajectoryEdges;
            for(int i = 0; i < trajectory.size()-1; i++)
            {
                trajectoryEdges.emplace_back(trajectory[i], trajectory[i + 1]);
            }

            for(auto* e: m_baseGraph->edges())
            {
                CGAL::Segment_2<Kernel> seg(vertexLocations[e->m_source->id()], vertexLocations[e->m_sink->id()]);
                for(auto i = 1; i < trajectory.size(); i++)
                {
                    // Connections between levels
                    if(i != trajectory.size() -1 && CGAL::squared_distance(seg, trajectory[i]) < epsSquared)
                    {
                        // both ways are valid.
                        output.addEdge(levelVertex(e,i - 1), levelVertex(e,i));
                        output.addEdge(levelVertex(e, i), levelVertex(e, i-1));
                    }

                    // Connections to other edges from this level.
                    // Incoming edges
                    if(CGAL::squared_distance(trajectoryEdges[i-1], seg.source()) < epsSquared)
                    {
                        for(auto* pred : e->m_source->m_inEdges)
                        {
                            // Only add once
                            if (pred->id() > e->id()) continue;
                            output.addEdge(levelVertex(pred, i - 1), levelVertex(e, i - 1));
                            // TODO also reverse?
                        }
                    }

                    // Connections to outgoing edges
                    if (CGAL::squared_distance(trajectoryEdges[i - 1], seg.source()) < epsSquared)
                    {
                        for (auto* pred : e->m_source->m_inEdges)
                        {
                            // Only add once
                            if (pred->id() > e->id()) continue;
                            output.addEdge(levelVertex(pred, i - 1), levelVertex(e, i - 1));
                        }
                    }
                }

                // Possibly connect to source or sink
                if(CGAL::squared_distance(vertexLocations[e->m_source->id()], trajectory[0]) < epsSquared)
                {
                    output.addEdge(superSource, levelVertex(e, 0));
                }
                else if (CGAL::squared_distance(vertexLocations[e->m_sink->id()], trajectory.back()) < epsSquared)
                {
                    output.addEdge(levelVertex(e, trajectory.size()-1), superSink);
                }
            }
		}    
	};
}
#endif