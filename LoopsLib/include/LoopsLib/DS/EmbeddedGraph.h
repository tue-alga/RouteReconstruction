#ifndef DS_EMBEDDEDGRAPH_H
#define DS_EMBEDDEDGRAPH_H
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Geometry/Index/PointIndex.h>
#include <ogr_spatialref.h>
#include <LoopsLib/DS/BiMap.h>

namespace LoopsLib::DS
{
    class EmbeddedGraph : public DS::BaseGraph
    {
    public:
        using Point = MovetkGeometryKernel::MovetkPoint;
    private:
        // Locations of the vertices
        std::vector<Point> m_locations;

        // Layers for LOD 
        std::vector<std::set<DS::BaseGraph::Id_t>> m_layers;

        // RTree index on points for location lookup
        LoopsLib::Geometry::Index::PointIndex m_index;

        // Spatial reference
        OGRSpatialReference m_ref;

        // ID mapping
        BiMap<Id_t, Id_t> m_vertexIdMap; //Original to internal
        BiMap<Id_t, Id_t> m_edgeIdMap;//Original to internal
    public:
        OGRSpatialReference& spatialRef();
        const OGRSpatialReference* spatialRefPntr() const;

        BiMap<Id_t, Id_t>& vertexIdMap();

        const BiMap<Id_t, Id_t>& vertexIdMap() const;

        BiMap<Id_t, Id_t>& edgeIdMap();

        const BiMap<Id_t, Id_t>& edgeIdMap() const;

        void clearIndirectData();

        const OGRSpatialReference& spatialRef() const;
        // Allow usage of BaseGraph constructors
        using DS::BaseGraph::BaseGraph;

        using NodeIndex = std::size_t;
        using EdgeIndex = std::size_t;
        using EdgeIterator = decltype(m_vertices[0]->m_outEdges.begin());

        void reserveLayers(std::size_t layers);

        Geometry::Index::PointIndex& getIndex();

        const Geometry::Index::PointIndex& getIndex() const;

        void deleteVertexRange(std::size_t start);

        void swapVertices(DS::BaseGraph::Id_t v0, DS::BaseGraph::Id_t v1);

        std::size_t layerCount() const;

        const Point& vertexLocation(Id_t vId) const;

        const Point& vertexLocation(LoopsLib::DS::BaseGraph::Vertex* v) const;

        const std::set<Id_t>& layer(std::size_t index) const;

        void addToLayer(DS::BaseGraph::Id_t edgeIndex, std::size_t layer);

        NT edgeLength(Id_t edgeId);

        std::pair<EdgeIterator, EdgeIterator> out_edges(const Id_t& vertex);

        const std::vector<Point>& locations() const;

        void setLocations(const std::vector<Point>& locations, bool computeIndex = true);

        std::vector<Point>& locations();
    };

    void edgePathToLocations(const DS::EmbeddedGraph& graph, const std::vector<long long>& edgePath,
                                    std::vector<EmbeddedGraph::Point>& locations);

    void edgePathToLocations(const DS::EmbeddedGraph& graph,
                                    const std::vector<DS::EmbeddedGraph::EdgeIndex>& edgePath,
                                    std::vector<EmbeddedGraph::Point>& locations);

    void edgePathToLocations(const DS::EmbeddedGraph& graph,
                                    const std::vector<DS::EmbeddedGraph::EdgeHandle>& edgePath,
                                    std::vector<EmbeddedGraph::Point>& locations);

    void edgePathToLocations(const DS::EmbeddedGraph& graph,
                                    const std::vector<const DS::EmbeddedGraph::Edge*>& edgePath,
                                    std::vector<EmbeddedGraph::Point>& locations);
}
#endif
