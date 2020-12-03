#include <LoopsLib/DS/GraphHelpers/GraphModificationGuard.h>
using namespace LoopsLib;
using namespace LoopsLib::DS;

void GraphHelpers::GraphModificationGuard::removeNewEdges()
{
    for (auto e : m_addedEdges)
    {
        m_target->deleteEdge(e);
    }
    m_addedEdges.clear();
}

void GraphHelpers::GraphModificationGuard::removeNewVertices()
{
    for (auto v : m_addedVertices)
    {
        m_target->deleteVertex(v);
    }
}

void GraphHelpers::GraphModificationGuard::addOldVertices()
{
    for (auto removed : m_removedVertices)
    {
        // We only need to reconnect all edges, since we never actually removed the vertex
        for (auto e : removed.outEdges)
        {
            e->reconnect();
        }
        for (auto e : removed.inEdges)
        {
            e->reconnect();
        }
    }
}

void GraphHelpers::GraphModificationGuard::addOldEdges()
{
    // Only need to reconnect, we didn't actually delete
    // the edges
    for (auto removed : m_removedEdges)
    {
        removed.edge->reconnect();
    }
}

GraphHelpers::GraphModificationGuard::GraphModificationGuard(BaseGraph* target):
    m_target(target)
{
}

GraphHelpers::GraphModificationGuard::~GraphModificationGuard()
{
    // Revert all changes
    revertModifications();
}

void GraphHelpers::GraphModificationGuard::revertModifications()
{
    if (m_wasReverted) return;
    m_wasReverted = true;
    removeNewEdges();
    removeNewVertices();
    addOldVertices();
    addOldEdges();
}

BaseGraph::Edge* GraphHelpers::GraphModificationGuard::addEdge(DS::BaseGraph::Vertex* source,
                                                                       BaseGraph::Vertex* sink)
{
    auto* e = m_target->addEdge(source, sink);
    m_addedEdges.push_back(e);
    return e;
}

std::vector<BaseGraph::Edge*> GraphHelpers::GraphModificationGuard::addEdges(DS::BaseGraph::Vertex* source,
                                                                             const std::vector<long long>& vertexIds)
{
    std::vector<BaseGraph::Edge*> out;
    for (const auto& vId : vertexIds)
    {
        out.emplace_back(addEdge(source, m_target->vertex(vId)));
    }
    return out;
}

std::vector<BaseGraph::Edge*> GraphHelpers::GraphModificationGuard::addEdges(const std::vector<long long>& vertexIds,
    DS::BaseGraph::Vertex* sink)
{
    std::vector<BaseGraph::Edge*> out;
    for (const auto& vId : vertexIds)
    {
        out.emplace_back(addEdge(m_target->vertex(vId), sink));
    }
    return out;
}

BaseGraph::Vertex* GraphHelpers::GraphModificationGuard::addVertex()
{
    auto* v = m_target->addVertex();
    m_addedVertices.push_back(v);
    return v;
}

void GraphHelpers::GraphModificationGuard::removeEdge(BaseGraph::Edge* e)
{
    m_removedEdges.push_back({e, *e->m_source, *e->m_sink});
    // Don't actually remove it, but detach it
    e->detach(false);
}

void GraphHelpers::GraphModificationGuard::removeVertex(BaseGraph::Vertex* v)
{
    RemovedVertex removed;
    removed.vert = v;
    // Collect in and out edges, and detach them in the graph

    for (auto e : v->m_outEdges)
    {
        removed.outEdges.push_back(e);
    }
    for (auto e : v->m_inEdges)
    {
        removed.inEdges.push_back(e);
    }
    for (auto e : removed.outEdges)
    {
        e->detach(false);
    }
    for (auto e : removed.inEdges)
    {
        e->detach(false);
    }
    m_removedVertices.push_back(removed);
}
