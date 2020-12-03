#include <LoopsAlgs/Flow/UncapturableFlow.h>

LoopsAlgs::Flow::UncapturableFlow::NT LoopsAlgs::Flow::UncapturableFlow::compute(
    const LoopsLib::DS::EmbeddedGraph& graph, const std::vector<NT>& field,
    const std::vector<Trajectory>& representatives, NT epsilon)
{
    using Graph = LoopsLib::DS::EmbeddedGraph;
    LoopsLib::DS::GraphView view(&graph);
    std::set<Graph::VertexIndex> available;
    std::set<Graph::VertexIndex> occupied;
    for (auto i = 0; i < graph.number_of_vertices(); ++i)
    {
        available.insert(i);
    }

    for (const auto& representative : representatives)
    {
        graph.getIndex().containedInBuffer(representative, epsilon, occupied);
    }
    // Take difference
    std::set_difference(available.begin(), available.end(), occupied.begin(), occupied.end(),
                        std::inserter(view.availableVertices(),view.availableVertices().begin()));
    // Update 
    view.updateAvailableVerts();
    NT total = 0;
    // Sum all remaining that is definitely not possible to capture.
    for (auto vId : view.availableVertices())
    {
        for (auto e : view.outEdges(vId))
        {
            total += field[e->id()];
        }
    }
    return total;
}
