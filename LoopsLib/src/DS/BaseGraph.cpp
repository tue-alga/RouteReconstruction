#include <LoopsLib/DS/BaseGraph.h>
#include <LoopsLib/DS/GraphConcepts.h>
#include <unordered_set>
using namespace LoopsLib::DS;

long long BaseGraph::Vertex::id() const
{
    return m_id;
}

BaseGraph::Vertex::Vertex(long long id) : m_id(id)
{
}

BaseGraph::Vertex::operator long long() const
{
    return m_id;
}

BaseGraph::Edge* BaseGraph::Vertex::findOutEdge(Vertex* sink) const
{
    return findOutEdge(sink->id());
}

BaseGraph::Edge* BaseGraph::Vertex::findInEdge(Vertex* source) const
{
    return findInEdge(source->id());
}

BaseGraph::Edge* BaseGraph::Vertex::findOutEdge(BaseGraph::Id_t sink) const
{
    for (auto* e : m_outEdges)
    {
        if (e->m_sink->id() == sink) return e;
    }
    return nullptr;
}

BaseGraph::Edge* BaseGraph::Vertex::findInEdge(BaseGraph::Id_t source) const
{
    for (auto* e : m_inEdges)
    {
        if (e->m_source->id() == source) return e;
    }
    return nullptr;
}

bool BaseGraph::Vertex::connectedTo(long long otherVertex) const
{
    for (auto e : m_outEdges)
    {
        if (e->m_sink->id() == otherVertex)return true;
    }
    for (auto e : m_inEdges)
    {
        if (e->m_source->id() == otherVertex)return true;
    }
    return false;
}

long long BaseGraph::Edge::id() const
{
    return m_id;
}

BaseGraph::Edge::Edge(long long id) : m_id(id)
{
}

BaseGraph::Edge::operator long long() const
{
    return m_id;
}

bool BaseGraph::Edge::hasSink(long long id) const
{
    return m_sink->id() == id;
}

bool BaseGraph::Edge::hasSource(long long id) const
{
    return m_source->id() == id;
}

std::pair<BaseGraph::Vertex*, BaseGraph::Vertex*> BaseGraph::Edge::verts() const
{
    return std::make_pair(m_source, m_sink);
}

void BaseGraph::Edge::detach(bool nullSourceAndSink)
{
    assert(m_sink->m_inEdges.count(this) == 1);
    assert(m_source->m_outEdges.count(this) == 1);
    auto sinkEraseCount = m_sink->m_inEdges.erase(this);
    auto sourceEraseCount = m_source->m_outEdges.erase(this);
    if (nullSourceAndSink)
    {
        m_source = nullptr;
        m_sink = nullptr;
    }
}

void BaseGraph::Edge::reconnect()
{
    m_sink->m_inEdges.insert(this);
    m_source->m_outEdges.insert(this);
}

void BaseGraph::Edge::flip()
{
    m_source->m_outEdges.erase(this);
    m_source->m_inEdges.insert(this);
    m_sink->m_inEdges.erase(this);
    m_sink->m_outEdges.insert(this);
    std::swap(m_source, m_sink);
}

bool BaseGraph::EdgeSortSink::operator()(Edge* e0, Edge* e1) const
{
    return (e0->m_sink->id() == e1->m_sink->id() && e0->id() < e1->id()) || e0->m_sink->id() < e1->m_sink->id();
}

bool BaseGraph::EdgeSortSource::operator()(Edge* e0, Edge* e1) const
{
    return (e0->m_source->id() == e1->m_source->id() && e0->id() < e1->id()) || e0->m_source->id() < e1->m_source->id();
}

BaseGraph::~BaseGraph()
{
    for (auto el : m_vertices)
    {
        delete el;
    }
    for (auto el : m_edges)
    {
        delete el;
    }
}

BaseGraph::BaseGraph()
{
}

void BaseGraph::clear()
{
    for (auto e : m_edges)
    {
        delete e;
    }
    for (auto v : m_vertices)
    {
        delete v;
    }
    m_edges.resize(0);
    m_vertices.resize(0);
}

void BaseGraph::allocateVertices(long long numberOfVertices)
{
    const auto vertNum = number_of_vertices();
    for (auto i = vertNum; i < vertNum + numberOfVertices; ++i)
    {
        m_vertices.push_back(new Vertex(i));
    }
}

void BaseGraph::allocateEdges(long long numberOfEdges)
{
    const auto edgeNum = number_of_edges();
    for (auto i = edgeNum; i < edgeNum + numberOfEdges; ++i)
    {
        m_edges.push_back(new Edge(i));
    }
}

BaseGraph::BaseGraph(long long numberOfVertices) : BaseGraph()
{
    // TODO placement new for more aligned data
    for (auto i = 0; i < numberOfVertices; ++i)
    {
        m_vertices.push_back(new Vertex(i));
    }
}

BaseGraph::BaseGraph(long long numberOfVertices, long long numberOfEdges) : BaseGraph()
{
    // TODO placement new for more aligned data
    m_vertices.reserve(numberOfVertices);
    for (auto i = 0; i < numberOfVertices; ++i)
    {
        m_vertices.push_back(new Vertex(i));
    }
    m_edges.reserve(numberOfEdges);
    for (auto i = 0; i < numberOfEdges; ++i)
    {
        m_edges.push_back(new Edge(i));
    }
}

const std::vector<BaseGraph::Vertex*>& BaseGraph::vertices() const
{
    return m_vertices;
}

const std::vector<BaseGraph::Edge*>& BaseGraph::edges() const
{
    return m_edges;
}

std::size_t BaseGraph::number_of_vertices() const
{
    return m_vertices.size();
}

BaseGraph::Idx_t BaseGraph::numberOfVertices() const
{
    return number_of_vertices();
}

std::size_t BaseGraph::number_of_edges() const
{
    return m_edges.size();
}

BaseGraph::Idx_t BaseGraph::numberOfEdges() const
{
    return number_of_edges();
}

bool BaseGraph::isSimplePath(const std::vector<Id_t>& vertices) const
{
    std::unordered_set<Id_t> visited;
    for (int i = 0; i < vertices.size(); i++)
    {
        auto vId = vertices[i];
        if (visited.find(vId) != visited.end()) return false;
        visited.insert(vId);
        auto* vert = vertex(vId);
        if (i == vertices.size() - 1) break;
        bool nextFound = false;
        for (auto* e : vert->m_outEdges)
        {
            if (e->m_sink->id() == vertices[i + 1])
            {
                nextFound = true;
                break;
            }
        }
        if (!nextFound) return false;
    }
    return true;
}

bool BaseGraph::isSimpleEdgePath(const Eigen::VectorXd& edges) const
{
    int count = 0;
    int startIndex = -1;
    for (int i = 0; i < edges.size(); i++)
    {
        if (edges(i) > 0)
        {
            ++count;
            startIndex = i;
        }
    }
    std::unordered_set<Id_t> visited;
    // Forward
    Id_t currentIndex = startIndex;
    while (true)
    {
        auto* e = edge(currentIndex);
        if (visited.find(e->m_sink->id()) != visited.end()) return false;
        visited.insert(e->m_sink->id());

        for (auto* eNext : e->m_sink->m_outEdges)
        {
            if (edges(eNext->id()) > 0)
            {
                currentIndex = eNext->id();
                break;
            }
        }
        // No next element found
        if (currentIndex == e->id()) break;
    }
    // Backward
    currentIndex = startIndex;
    while (true)
    {
        auto* e = edge(currentIndex);
        if (visited.find(e->m_source->id()) != visited.end()) return false;
        visited.insert(e->m_source->id());

        for (auto* eNext : e->m_source->m_inEdges)
        {
            if (edges(eNext->id()) > 0)
            {
                currentIndex = eNext->id();
                break;
            }
        }
        // No next element found
        if (currentIndex == e->id()) break;
    }
    // Total visited vertices should be one more than number of edges.
    return visited.size() == count + 1;
}

bool BaseGraph::isSimpleVertPath(const Eigen::VectorXd& verts) const
{
    int count = 0;
    int startIndex = -1;
    for (int i = 0; i < verts.size(); i++)
    {
        if (verts(i) > 0)
        {
            ++count;
            startIndex = i;
        }
    }
    std::unordered_set<Id_t> visited;
    // Forward
    Id_t currentIndex = startIndex;
    while (true)
    {
        auto* v = vertex(currentIndex);
        if (visited.find(v->id()) != visited.end()) return false;
        visited.insert(v->id());

        for (auto* eNext : v->m_outEdges)
        {
            if (verts(eNext->m_sink->id()) > 0)
            {
                currentIndex = eNext->m_sink->id();
                break;
            }
        }
        // No next element found
        if (currentIndex == v->id()) break;
    }
    // Backward
    visited.erase(startIndex); //We are going to see the start vert twice
    currentIndex = startIndex;
    while (true)
    {
        auto* v = vertex(currentIndex);
        if (visited.find(v->id()) != visited.end()) return false;
        visited.insert(v->id());

        for (auto* eNext : v->m_inEdges)
        {
            if (verts(eNext->m_source->id()) > 0)
            {
                currentIndex = eNext->m_source->id();
                break;
            }
        }
        // No next element found
        if (currentIndex == v->id()) break;
    }
    // Total visited vertices should be one more than number of edges.
    return visited.size() == count;
}

bool BaseGraph::isSimplePath(const std::vector<Edge*>& edges) const
{
    std::unordered_set<Id_t> visited;
    visited.insert(edges[0]->m_source->id());
    for (int i = 0; i < edges.size(); i++)
    {
        auto* e = edges[i];
        if (visited.find(e->m_sink->id()) != visited.end()) return false;
        visited.insert(e->m_sink->id());

        if (i == edges.size() - 1) break;

        bool nextFound = false;
        for (auto* eNext : e->m_sink->m_outEdges)
        {
            if (eNext->id() == edges[i + 1]->id())
            {
                nextFound = true;
                break;
            }
        }
        if (!nextFound) return false;
    }
    return true;
}

bool BaseGraph::isSimplePath(const std::vector<Vertex*>& vertices) const
{
    std::unordered_set<Id_t> visited;
    for (int i = 0; i < vertices.size(); i++)
    {
        auto* vert = vertices[i];
        auto vId = vert->id();
        if (visited.find(vId) != visited.end()) return false;
        visited.insert(vId);
        if (i == vertices.size() - 1) break;
        bool nextFound = false;
        for (auto* e : vert->m_outEdges)
        {
            if (e->m_sink->id() == vertices[i + 1]->id())
            {
                nextFound = true;
                break;
            }
        }
        if (!nextFound) return false;
    }
    return true;
}

BaseGraph::Edge* BaseGraph::edge(long long id)
{
    return m_edges[id];
}

BaseGraph::Vertex* BaseGraph::vertex(long long id)
{
    return m_vertices[id];
}

const BaseGraph::Edge* BaseGraph::edge(long long id) const
{
    return m_edges[id];
}

const BaseGraph::Vertex* BaseGraph::vertex(long long id) const
{
    return m_vertices[id];
}

BaseGraph::Edge* BaseGraph::addEdge(long long source, long long sink)
{
    return addEdge(vertex(source), vertex(sink));
}

BaseGraph::Edge* BaseGraph::addEdge(Vertex* source, Vertex* sink)
{
    assert(source != sink);
    auto* edge = new Edge(m_edges.size());
    m_edges.push_back(edge);
    edge->m_source = source;
    edge->m_sink = sink;
    source->m_outEdges.insert(edge);
    sink->m_inEdges.insert(edge);
    return edge;
}

BaseGraph::Vertex* BaseGraph::addVertex()
{
    auto* vert = new Vertex(m_vertices.size());
    m_vertices.push_back(vert);
    return vert;
}

void BaseGraph::deleteEdge(Edge* e)
{
    e->detach();
    const auto loc = e->id();
    delete e;
    if (loc + 1 != m_edges.size())
    {
        m_edges[loc] = m_edges.back();
        m_edges[loc]->m_id = loc;
    }
    m_edges.resize(m_edges.size() - 1);
}

void BaseGraph::deleteVertex(Vertex* v)
{
    auto inEdges = v->m_inEdges;
    for (auto e : inEdges)
    {
        this->deleteEdge(e);
    }
    auto outEdges = v->m_outEdges;
    for (auto e : outEdges)
    {
        this->deleteEdge(e);
    }
    auto vid = v->id();
    delete v;
    m_vertices[vid] = m_vertices.back();
    m_vertices[vid]->m_id = vid;
    m_vertices.resize(m_vertices.size() - 1);
}

std::vector<BaseGraph::Edge*> BaseGraph::addEdges(const std::vector<std::pair<Id_t, Id_t>>& connections)
{
    std::vector<Edge*> out;
    for (auto conn : connections)
    {
        out.push_back(addEdge(conn.first, conn.second));
    }
    return out;
}