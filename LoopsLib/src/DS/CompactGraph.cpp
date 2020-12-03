#include <LoopsLib/DS/CompactGraph.h>

using namespace LoopsLib::DS;

LoopsLib::DS::CompactGraph::IdToEdgeHandle::IdToEdgeHandle(CompactGraph& graph): m_graph(graph)
{
}

LoopsLib::DS::CompactGraph::EdgeHandle LoopsLib::DS::CompactGraph::IdToEdgeHandle::operator()(Idx_t eId) const
{
    return EdgeHandle(m_graph, eId);
}

LoopsLib::DS::CompactGraph::VertexHandle::VertexHandle(CompactGraph& parent, Idx_t id): m_parent(parent), m_vertexId(id)
{
}

LoopsLib::DS::CompactGraph::Idx_t LoopsLib::DS::CompactGraph::VertexHandle::id() const
{
    return m_vertexId;
}

bool LoopsLib::DS::CompactGraph::VertexHandle::operator==(const VertexHandle& other) const
{
    return m_vertexId == other.m_vertexId;
}

LoopsLib::DS::CompactGraph::VertexHandle* LoopsLib::DS::CompactGraph::VertexHandle::operator->()
{
    return this;
}

LoopsLib::DS::CompactGraph::EdgeHandle LoopsLib::DS::CompactGraph::VertexHandle::findOutEdge(Idx_t targetVertex)
{
    for (const EdgeHandle& handle : outEdges())
    {
        if (handle.sinkVertex().id() == targetVertex) return handle;
    }
    return EdgeHandle(m_parent, INVALID_IDX);
}

LoopsLib::DS::CompactGraph::EdgeHandle LoopsLib::DS::CompactGraph::VertexHandle::findInEdge(Idx_t targetVertex)
{
    for (const EdgeHandle& handle : inEdges())
    {
        if (handle.sinkVertex().id() == targetVertex) return handle;
    }
    return EdgeHandle(m_parent, INVALID_IDX);
}

LoopsLib::Helpers::Iterators::Iterable<LoopsLib::DS::CompactGraph::VertexHandle::ToEHIt> LoopsLib::DS::CompactGraph::VertexHandle
::outEdges() const
{
    IdToEdgeHandle toEh(m_parent);
    auto it = ToEHIt(m_parent.m_vertices[m_vertexId].m_outEdges.begin(), toEh);
    return Helpers::Iterators::Iterable(it, it.associatedEnd());
}

LoopsLib::Helpers::Iterators::Iterable<LoopsLib::DS::CompactGraph::VertexHandle::ToEHIt> LoopsLib::DS::CompactGraph::VertexHandle
::inEdges() const
{
    IdToEdgeHandle toEh(m_parent);
    auto it = ToEHIt(m_parent.m_vertices[m_vertexId].m_inEdges.begin(), toEh);
    return Helpers::Iterators::Iterable(it, it.associatedEnd());
}

LoopsLib::DS::CompactGraph::EdgeHandle::EdgeHandle(CompactGraph& parent, Idx_t id): m_parent(parent), m_edgeId(id)
{
}

LoopsLib::DS::CompactGraph::Idx_t LoopsLib::DS::CompactGraph::EdgeHandle::id() const
{
    return m_edgeId;
}

bool LoopsLib::DS::CompactGraph::EdgeHandle::operator==(const EdgeHandle& other) const
{
    return m_edgeId == other.m_edgeId;
}

LoopsLib::DS::CompactGraph::EdgeHandle* LoopsLib::DS::CompactGraph::EdgeHandle::operator->()
{
    return this;
}

LoopsLib::DS::CompactGraph::VertexHandle LoopsLib::DS::CompactGraph::EdgeHandle::sourceVertex() const
{
    return VertexHandle(m_parent, m_parent.m_edges[m_edgeId].srcVertex);
}

LoopsLib::DS::CompactGraph::VertexHandle LoopsLib::DS::CompactGraph::EdgeHandle::sinkVertex() const
{
    return VertexHandle(m_parent, m_parent.m_edges[m_edgeId].sinkVertex);
}

LoopsLib::DS::CompactGraph::CompactGraph()
{
}

LoopsLib::DS::CompactGraph::CompactGraph(Idx_t numberOfVertices)
{
}

bool LoopsLib::DS::CompactGraph::isEdgeValid(Idx_t eId) const
{
    return m_edges[eId].srcVertex < m_edges.size();
}

void LoopsLib::DS::CompactGraph::allocateVertices(Idx_t numberOfVertices)
{
    m_vertices.resize(numberOfVertices, {});
}

void LoopsLib::DS::CompactGraph::allocateEdges(Idx_t numberOfEdges)
{
    m_edges.resize(numberOfEdges, {});
}

void LoopsLib::DS::CompactGraph::setEdgeConnection(Idx_t edge, Idx_t src, Idx_t sink)
{
    m_edges[edge].srcVertex = src;
    m_edges[edge].sinkVertex = sink;
    m_vertices[src].m_outEdges.push_back(edge);
    m_vertices[sink].m_inEdges.push_back(edge);
}

LoopsLib::DS::CompactGraph::VertexHandle LoopsLib::DS::CompactGraph::vertex(Idx_t vId)
{
    return VertexHandle(*this, vId);
}

LoopsLib::DS::CompactGraph::EdgeHandle LoopsLib::DS::CompactGraph::edge(Idx_t eId)
{
    return EdgeHandle(*this, eId);
}

std::size_t LoopsLib::DS::CompactGraph::numberOfEdges() const
{
    return m_edges.size();
}

std::size_t LoopsLib::DS::CompactGraph::numberOfVertices() const
{
    return m_vertices.size();
}
