#ifndef GEOMETRY_INDEX_EDGERTREE_H
#define GEOMETRY_INDEX_EDGERTREE_H
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/DS/EmbeddedGraph.h>
#include <LoopsLib/Geometry/BoostInterface.h>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometry.hpp>
#include <movetk/geom/GeometryInterface.h>
#include <LoopsLib/Helpers/Iterators.h>

namespace LoopsLib::Geometry::Index
{
    class EdgeIndex
    {
        // Boost rtree
        std::size_t m_numVertices = 0;
        DS::EmbeddedGraph* m_graph = nullptr;
    public:

        /**
         * Box of a edge
         */
        typedef boost::geometry::model::box<DS::EmbeddedGraph::Point> boost_box;
        typedef boost::geometry::model::segment<DS::EmbeddedGraph::Point> boost_segment;
        /**
         * Item stored in a node of Rtree
         */
        typedef std::pair<boost_box, DS::BaseGraph::Edge*> Item;
        /**
         * Rtree of road edges
         */
        typedef boost::geometry::index::rtree<
            Item, boost::geometry::index::quadratic<16> > Rtree;
        Rtree m_rtree;

        void clear();

        void construct(DS::EmbeddedGraph* graph);
        void construct(DS::EmbeddedGraph* graph, const std::set<DS::BaseGraph::Id_t>& edges);

        std::vector<DS::EmbeddedGraph::EdgeIndex> lookUp(NT minX, NT maxX, NT minY, NT maxY) const;

        std::vector<DS::EmbeddedGraph::EdgeIndex> kNearest(NT cX, NT cY, int k, NT searchRadius) const;
    };
}
#endif