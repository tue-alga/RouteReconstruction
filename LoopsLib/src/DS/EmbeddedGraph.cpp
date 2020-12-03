#include <LoopsLib/DS/EmbeddedGraph.h>
using namespace LoopsLib;
using namespace LoopsLib::DS;

OGRSpatialReference& EmbeddedGraph::spatialRef()
{
    return m_ref;
}

const OGRSpatialReference* EmbeddedGraph::spatialRefPntr() const
{
    return &m_ref;
}

BiMap<EmbeddedGraph::Id_t, EmbeddedGraph::Id_t>& EmbeddedGraph::vertexIdMap()
{
    return m_vertexIdMap;
}

const BiMap<EmbeddedGraph::Id_t, EmbeddedGraph::Id_t>& EmbeddedGraph::vertexIdMap() const
{
    return m_vertexIdMap;
}

BiMap<EmbeddedGraph::Id_t, EmbeddedGraph::Id_t>& EmbeddedGraph::edgeIdMap()
{
    return m_edgeIdMap;
}

const BiMap<EmbeddedGraph::Id_t, EmbeddedGraph::Id_t>& EmbeddedGraph::edgeIdMap() const
{
    return m_edgeIdMap;
}

void EmbeddedGraph::clearIndirectData()
{
    m_vertexIdMap = {};
    m_edgeIdMap = {};
    m_layers = {};
}

const OGRSpatialReference& EmbeddedGraph::spatialRef() const
{
    return m_ref;
}

void EmbeddedGraph::reserveLayers(std::size_t layers)
{
    m_layers.resize(layers, {});
}

LoopsLib::Geometry::Index::PointIndex& EmbeddedGraph::getIndex()
{
    return m_index;
}

const Geometry::Index::PointIndex& EmbeddedGraph::getIndex() const
{
    return m_index;
}

void EmbeddedGraph::deleteVertexRange(std::size_t start)
{
    const auto target = number_of_vertices();
    for (std::size_t i = start; i < target; ++i)
    {
        auto* v = vertex(i);
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
    }
    m_vertices.resize(start);
}

void EmbeddedGraph::swapVertices(DS::BaseGraph::Id_t v0, DS::BaseGraph::Id_t v1)
{
    if (v0 == v1) return;

    if (m_vertexIdMap.size() > 0)
    {
        auto v0Id = m_vertexIdMap.backward(v0);
        auto v1Id = m_vertexIdMap.backward(v1);
        m_vertexIdMap.eraseViaForward(v0Id);
        m_vertexIdMap.eraseViaForward(v1Id);
        m_vertexIdMap.insert(v0Id, v1);
        m_vertexIdMap.insert(v1Id, v0);
    }
    std::swap(m_vertices[v0]->m_id, m_vertices[v1]->m_id);
    {
        auto* temp = m_vertices[v0];
        m_vertices[v0] = m_vertices[v1];
        m_vertices[v1] = temp;
    }
    std::swap(m_locations[v0], m_locations[v1]);
    for(auto* e : m_vertices[v0]->m_inEdges)
    {
        assert(e);
        assert(m_vertices[v0]->m_inEdges.count(e) == 1);
    }
    for (auto* e : m_vertices[v0]->m_outEdges)
    {
        assert(e);
        assert(m_vertices[v0]->m_outEdges.count(e) == 1);
    }
    for (auto* e : m_vertices[v1]->m_inEdges)
    {
        assert(e);
        assert(m_vertices[v1]->m_inEdges.count(e) == 1);
    }
    for (auto* e : m_vertices[v1]->m_outEdges)
    {
        assert(e);
        assert(m_vertices[v1]->m_outEdges.count(e) == 1);
    }
}

std::size_t EmbeddedGraph::layerCount() const
{
    return m_layers.size();
}

const EmbeddedGraph::Point& EmbeddedGraph::vertexLocation(Id_t vId) const
{
    return m_locations[vId];
}

const EmbeddedGraph::Point& EmbeddedGraph::vertexLocation(LoopsLib::DS::BaseGraph::Vertex* v) const
{
    return m_locations[v->id()];
}


const std::set<BaseGraph::Id_t>& EmbeddedGraph::layer(std::size_t index) const
{
    return m_layers[index];
}

void EmbeddedGraph::addToLayer(DS::BaseGraph::Id_t edgeIndex, std::size_t layer)
{
    m_layers[layer].insert(edgeIndex);
}

LoopsLib::NT EmbeddedGraph::edgeLength(Id_t edgeId)
{
    const auto& v0 = m_locations.at(edge(edgeId)->m_source->id());
    const auto& v1 = m_locations.at(edge(edgeId)->m_sink->id());
    return std::hypot(v0.get().x()- v1.get().x(), v0.get().y() - v1.get().y());
}

std::pair<EmbeddedGraph::EdgeIterator, EmbeddedGraph::EdgeIterator> EmbeddedGraph::out_edges(
    const Id_t& vertex)
{
    return std::make_pair(this->vertex(vertex)->m_outEdges.begin(), this->vertex(vertex)->m_outEdges.end());
}

const std::vector<EmbeddedGraph::Point>& EmbeddedGraph::locations() const
{
    return m_locations;
}

void EmbeddedGraph::setLocations(const std::vector<Point>& locations, bool computeIndex)
{
    m_locations = locations;
    if (computeIndex)
    {
        m_index.clear();
        m_index.construct(locations);
    }
}

std::vector<EmbeddedGraph::Point>& EmbeddedGraph::locations()
{
    return m_locations;
}

void DS::edgePathToLocations(const DS::EmbeddedGraph& graph, const std::vector<long long>& edgePath,
                             std::vector<EmbeddedGraph::Point>& locations)
{
    if (edgePath.empty()) return;

    if (graph.numberOfEdges() == 0)throw std::runtime_error("Empty graph given to edgePathToLocation()");

    locations.reserve(edgePath.size() + 1);
    locations.push_back(graph.vertexLocation(graph.edge(edgePath[0])->m_source));

    for (int i = 0; i < edgePath.size(); ++i)
    {
        locations.push_back(graph.vertexLocation(graph.edge(edgePath[i])->m_sink));
    }
}

void DS::edgePathToLocations(const DS::EmbeddedGraph& graph, const std::vector<DS::EmbeddedGraph::EdgeIndex>& edgePath,
                             std::vector<EmbeddedGraph::Point>& locations)
{
    if (edgePath.empty()) return;

    locations.reserve(edgePath.size() + 1);
    locations.push_back(graph.vertexLocation(graph.edge(edgePath[0])->m_source));

    for (int i = 0; i < edgePath.size(); ++i)
    {
        locations.push_back(graph.vertexLocation(graph.edge(edgePath[i])->m_sink));
    }
}

void DS::edgePathToLocations(const DS::EmbeddedGraph& graph, const std::vector<DS::EmbeddedGraph::EdgeHandle>& edgePath,
                             std::vector<EmbeddedGraph::Point>& locations)
{
    if (edgePath.empty()) return;

    locations.reserve(edgePath.size() + 1);
    locations.push_back(graph.vertexLocation(edgePath[0]->m_source));

    for (int i = 0; i < edgePath.size(); ++i)
    {
        locations.push_back(graph.vertexLocation(edgePath[i]->m_sink));
    }
}

void DS::edgePathToLocations(const DS::EmbeddedGraph& graph,
                             const std::vector<const DS::EmbeddedGraph::Edge*>& edgePath,
                             std::vector<EmbeddedGraph::Point>& locations)
{
    if (edgePath.empty()) return;

    locations.reserve(edgePath.size() + 1);
    locations.push_back(graph.vertexLocation(edgePath[0]->m_source));

    for (int i = 0; i < edgePath.size(); ++i)
    {
        locations.push_back(graph.vertexLocation(edgePath[i]->m_sink));
    }
}
