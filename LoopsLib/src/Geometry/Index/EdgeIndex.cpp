#include <LoopsLib/Geometry/Index/EdgeIndex.h>
#include <boost/geometry/index/predicates.hpp>
#include <LoopsLib/Helpers/Iterators.h>
using namespace LoopsLib;
using namespace LoopsLib::Geometry;

void Index::EdgeIndex::clear()
{
    m_rtree.clear();
}

void Index::EdgeIndex::construct(DS::EmbeddedGraph* graph)
{
    m_graph = graph;
    auto mkPoint = movetk_core::MakePoint<MovetkGeometryKernel>();
    for (auto* e : graph->edges())
    {
        // Create box for edge segment
        NT min_x, min_y, max_x, max_y;
        auto v0 = graph->locations()[*e->m_source];
        auto v1 = graph->locations()[*e->m_sink];
        std::tie(min_x, max_x) = std::minmax(v0.get().x(), v1.get().x());
        std::tie(min_y, max_y) = std::minmax(v0.get().y(), v1.get().y());
        boost_box box(mkPoint({min_x, min_y}), mkPoint({max_x, max_y}));
        m_rtree.insert(std::make_pair(box, e));
    }
}

void Index::EdgeIndex::construct(DS::EmbeddedGraph* graph, const std::set<DS::BaseGraph::Id_t>& edges)
{
    m_graph = graph;
    auto mkPoint = movetk_core::MakePoint<MovetkGeometryKernel>();
    for (const auto& eId: edges)
    {
        auto* e = graph->edge(eId);
        if(!e->m_source || !e->m_sink)
        {
            continue;
        }
        // Create box for edge segment
        NT min_x, min_y, max_x, max_y;
        auto v0 = graph->locations()[*e->m_source];
        auto v1 = graph->locations()[*e->m_sink];
        std::tie(min_x, max_x) = std::minmax(v0.get().x(), v1.get().x());
        std::tie(min_y, max_y) = std::minmax(v0.get().y(), v1.get().y());
        boost_box box(mkPoint({ min_x, min_y }), mkPoint({ max_x, max_y }));
        m_rtree.insert(std::make_pair(box, e));
    }
}

std::vector<LoopsLib::DS::EmbeddedGraph::EdgeIndex> Index::EdgeIndex::lookUp(NT minX, NT maxX, NT minY, NT maxY) const
{
    auto mkPoint = movetk_core::MakePoint<MovetkGeometryKernel>();
    boost_box b(mkPoint({minX, minY}),
                mkPoint({maxX, maxY}));
    std::vector<DS::EmbeddedGraph::EdgeIndex> temp;
    Helpers::Iterators::transformed_back_inserter_iterator inserter(temp, [](const Item& i)
    {
        return i.second->id();
    });
    // Rtree can only detect intersect with a the bounding box of
    // the geometry stored.
    m_rtree.query(boost::geometry::index::intersects(b),
                  inserter);
    return temp;
}

std::vector<DS::EmbeddedGraph::EdgeIndex> Index::EdgeIndex::kNearest(NT cX, NT cY, int k, NT searchRadius) const
{
    auto result = lookUp(cX - searchRadius, cX + searchRadius, cY - searchRadius, cY + searchRadius);
    auto center = LoopsLib::Math::Vec2<NT>(cX, cY);

    std::unordered_map<DS::EmbeddedGraph::EdgeIndex, NT> dist;

    auto closestDist = [this,&dist](const auto& e0, const LoopsLib::Math::Vec2<NT>& vert) -> NT
    {
        if (dist.find(e0) != dist.end()) return dist.at(e0);

        auto p0 = this->m_graph->vertexLocation(m_graph->edge(e0)->m_source);
        auto p1 = m_graph->vertexLocation(m_graph->edge(e0)->m_sink);
        auto diff = p1 - p0;
        NT segLength;
        auto projected = vert.projectOn(p0, p1, segLength);
        NT res = 0;
        if (diff.dot(projected-p0) < 0) res = (vert - p0).length();
        else if (diff.dot(projected - p1) > 0) res = (vert - p1).length();
        else res = (projected - vert).length();

        dist[e0] = res;

        return res;
    };

    if(result.size() <= k)
    {
        return result;
    }

    // Sort by closest distance
    std::sort(result.begin(), result.end(), [closestDist, &center](const auto& e0, const auto& e1)
    {
        return closestDist(e0, center) < closestDist(e1, center);
    });
    // Sublist of the original
    return std::vector<DS::EmbeddedGraph::EdgeIndex>(result.begin(), result.begin() + k);
}
