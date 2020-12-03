#include <LoopsLib/DS/GraphHelpers/DagView.h>

using namespace LoopsLib::DS;

GraphHelpers::DagView::DagView(BaseGraph* graph, const std::vector<int>& dagMapping):
    m_dagMapping(dagMapping),
    m_graph(graph)
{
}

bool GraphHelpers::DagView::DagEdgeIterator::isAbsent(BaseGraph::Edge* edge)
{
    return m_parent->m_dagMapping[edge->m_source->id()] > m_parent->m_dagMapping[edge->m_sink->id()];
}

GraphHelpers::DagView::DagEdgeIterator GraphHelpers::DagView::DagEdgeIterator::end(DagView* parent,
                                                                                           BaseGraph::Vertex* vertex,
                                                                                           bool isOut)
{
    DagEdgeIterator it(parent, vertex, isOut);
    it.it = it.m_end;
    return it;
}

GraphHelpers::DagView::DagEdgeIterator::DagEdgeIterator(DagView* parent, BaseGraph::Vertex* vertex, bool isOut):
    m_parent(parent),
    m_vertex(vertex)
{
    m_end = isOut ? vertex->m_outEdges.end() : vertex->m_inEdges.end();
    it = isOut ? vertex->m_outEdges.begin() : vertex->m_inEdges.begin();
    for (; it != m_end; ++it)
    {
        if (!isAbsent(*it)) break;
    }
}

BaseGraph::Edge* GraphHelpers::DagView::DagEdgeIterator::operator*() const
{
    return *it;
}

BaseGraph::Edge* GraphHelpers::DagView::DagEdgeIterator::operator->() const
{
    return *it;
}

bool GraphHelpers::DagView::DagEdgeIterator::operator!=(const DagEdgeIterator& other) const
{
    return it != other.it;
}

bool GraphHelpers::DagView::DagEdgeIterator::operator==(const DagEdgeIterator& other) const
{
    return it == other.it;
}

GraphHelpers::DagView::DagEdgeIterator& GraphHelpers::DagView::DagEdgeIterator::operator++()
{
    ++it;
    for (; it != m_end; ++it)
    {
        if (!isAbsent(*it)) break;
    }
    return *this;
}

GraphHelpers::DagView::DagEdges::DagEdges(DagView* parent, BaseGraph::Vertex* vert, bool isOut):
    m_parent(parent),
    m_isOut(isOut),
    m_vert(vert)
{
}

GraphHelpers::DagView::DagEdgeIterator GraphHelpers::DagView::DagEdges::begin()
{
    return DagEdgeIterator(m_parent, m_vert, m_isOut);
}

GraphHelpers::DagView::DagEdgeIterator GraphHelpers::DagView::DagEdges::end()
{
    return DagEdgeIterator::end(m_parent, m_vert, m_isOut);
}

GraphHelpers::DagView::DagEdgeIterator GraphHelpers::DagView::outEdgesBegin(BaseGraph::Vertex* vert)
{
    return DagEdgeIterator(this, vert, true);
}

GraphHelpers::DagView::DagEdgeIterator GraphHelpers::DagView::outEdgesEnd(BaseGraph::Vertex* vert)
{
    return DagEdgeIterator::end(this, vert, true);
}

GraphHelpers::DagView::DagEdgeIterator GraphHelpers::DagView::inEdgesBegin(BaseGraph::Vertex* vert)
{
    return DagEdgeIterator(this, vert, false);
}

GraphHelpers::DagView::DagEdgeIterator GraphHelpers::DagView::inEdgesEnd(BaseGraph::Vertex* vert)
{
    return DagEdgeIterator::end(this, vert, false);
}

GraphHelpers::DagView::DagEdges GraphHelpers::DagView::dagEdgesOut(BaseGraph::Vertex* vert)
{
    return DagEdges(this, vert, true);
}

GraphHelpers::DagView::DagEdges GraphHelpers::DagView::dagEdgesIn(BaseGraph::Vertex* vert)
{
    return DagEdges(this, vert, false);
}
