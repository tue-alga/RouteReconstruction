#include <LoopsLib/GraphAlgs/DFS.h>

using namespace LoopsLib;

bool GraphAlgs::DFSIterator::seen(DS::BaseGraph::Vertex* vert) const
{
    return m_seen.find(vert->id()) != m_seen.end();
}

void GraphAlgs::DFSIterator::setToEnd()
{
    while (!m_queue.empty())m_queue.pop();
    // Set iterators to arbitrary but fixed value
    m_edgeIt = m_end = m_graph->vertex(0)->m_outEdges.begin();
    m_current = nullptr;
}

GraphAlgs::DFSIterator::DFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Vertex* start,
    const std::unordered_set<DS::BaseGraph::Id_t>& validEdges):
    m_graph(graph),
    m_current(start),
    m_edgeIt(start->m_outEdges.begin()),
    m_end(start->m_outEdges.end()),
    m_validEdges(validEdges)
{
}

GraphAlgs::DFSIterator::DFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Vertex* start):
    m_graph(graph),
    m_current(start),
    m_edgeIt(start->m_outEdges.begin()),
    m_end(start->m_outEdges.end())
{
}

GraphAlgs::DFSIterator::DFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Id_t start): DFSIterator(
    graph, graph->vertex(start))
{
}

GraphAlgs::DFSIterator::DFSIterator(DS::BaseGraph* graph): m_graph(graph)
{
    setToEnd();
}

void GraphAlgs::DFSIterator::stopIterations()
{
    setToEnd();
}

std::vector<DS::BaseGraph::Vertex*> GraphAlgs::DFSIterator::currentStack()
{
    std::vector<DS::BaseGraph::Vertex*> output;
    while (!m_queue.empty())
    {
        auto el = m_queue.top();
        output.push_back(el);
    }
    for (auto it = output.rbegin(); it != output.rend(); ++it)
    {
        m_queue.push(*it);
    }
    std::reverse(output.begin(), output.end());
    return output;
}

GraphAlgs::DFSResult::DFSResult(DFSIterator& parent): m_parent(parent)
{
}

DS::BaseGraph::Edge* GraphAlgs::DFSResult::edge() const
{
    return *(m_parent.m_edgeIt);
}

bool GraphAlgs::DFSResult::sinkSeen() const
{
    return m_parent.seen(sink());
}

DS::BaseGraph::Vertex* GraphAlgs::DFSResult::source() const
{
    return (*m_parent.m_edgeIt)->m_source;
}

DS::BaseGraph::Vertex* GraphAlgs::DFSResult::sink() const
{
    return (*m_parent.m_edgeIt)->m_sink;
}

void GraphAlgs::DFSResult::stopIterations() const
{
    m_parent.stopIterations();
}

GraphAlgs::DFSIterator& GraphAlgs::DFSIterator::operator++()
{
    if (!seen((*m_edgeIt)->m_sink))
    {
        m_seen.insert((*m_edgeIt)->m_sink->id());
        m_queue.push((*m_edgeIt)->m_sink);
    }

    ++m_edgeIt;
    
    if (m_edgeIt == m_end)
    {
        // Done
        if (m_queue.empty())
        {
            setToEnd();
            return *this;
        }

        auto* vert = m_queue.top();
        m_queue.pop();
        m_edgeIt = vert->m_outEdges.begin();
        m_end = vert->m_outEdges.end();
        m_current = vert;
    }
    return *this;
}

GraphAlgs::DFSResult GraphAlgs::DFSIterator::operator*()
{
    return DFSResult(*this);
}

bool GraphAlgs::DFSIterator::operator!=(const DFSIterator& other) const
{
    return m_queue.size() != other.m_queue.size() ||
        m_current != other.m_current ||
        m_edgeIt != other.m_edgeIt || m_end != other.m_end;
}

GraphAlgs::DFSIterable::DFSIterable(DS::BaseGraph* graph, DS::BaseGraph::Id_t start): m_graph(graph),
                                                                                      m_start(start)
{
}

GraphAlgs::DFSIterator GraphAlgs::DFSIterable::begin()
{
    return DFSIterator(m_graph, m_start);
}

GraphAlgs::DFSIterator GraphAlgs::DFSIterable::end()
{
    return DFSIterator(m_graph);
}
