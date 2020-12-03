#include <LoopsLib/DS/GraphView.h>
using namespace LoopsLib::DS;

bool GraphView::isAvailable(Id_t vertId) const
{
    return m_availableVertices.find(vertId) != m_availableVertices.end();
}

GraphView::GraphView() : m_targetGraph(nullptr)
{
}

GraphView::GraphView(const BaseGraph* targetGraph) : m_targetGraph(targetGraph)
{
}

void GraphView::setGraph(const BaseGraph* graph)
{
    m_targetGraph = graph;
}

const BaseGraph* GraphView::rawGraph() const
{
    return m_targetGraph;
}

std::size_t GraphView::indexForVertex(Id_t vertId) const
{
    if (m_availableVertices.find(vertId) == m_availableVertices.end()) throw std::out_of_range(std::to_string(vertId) + " not present in graph view");
    return std::distance(m_availableVertices.begin(), m_availableVertices.find(vertId));
}

std::size_t GraphView::indexForVertex(VertexHandle vert) const
{
    return indexForVertex(vert->id());
}

std::set<GraphView::Id_t>& GraphView::availableVertices()
{
    return m_availableVertices;
}

const std::set<GraphView::Id_t>& GraphView::availableVertices() const
{
    return m_availableVertices;
}

std::size_t GraphView::numberOfEdges() const
{
    return m_edgeCount;
}

void GraphView::updateAvailableVerts()
{
    m_availalbleVerticesList.clear();
    m_availalbleVerticesList.assign(m_availableVertices.begin(), m_availableVertices.end());
    m_edgeCount = 0;
    for (auto v : m_availableVertices)
    {
        for (auto e : outEdges(v))
        {
            m_edgeCount++;
        }
    }
}



std::size_t GraphView::number_of_vertices() const
{
    return m_availableVertices.size();
}

GraphView::EdgeHandle GraphView::edge(Id_t edgeId) const
{
    auto* e = m_targetGraph->edge(edgeId);
    if (!isAvailable(e->m_source->id()) || !isAvailable(e->m_sink->id())) return nullptr;
    return e;
}

GraphView::VertexHandle GraphView::vertexByIndex(std::size_t index) const
{
    return m_targetGraph->vertex(m_availalbleVerticesList[index]);
}

GraphView::VertexHandle GraphView::vertex(Id_t vertexId) const
{
    if (!isAvailable(vertexId)) return nullptr;
    return m_targetGraph->vertex(vertexId);
}

GraphView::EdgeHandle GraphView::findConnection(VertexHandle v0, Id_t targetVertex)
{
    if (!isAvailable(targetVertex)) return nullptr;
    return v0->findOutEdge(targetVertex);
}
