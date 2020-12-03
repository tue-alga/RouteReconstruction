#ifndef LOOPSIO_SERIALIZERS_GRAPHSERIALIZER_H
#define LOOPSIO_SERIALIZERS_GRAPHSERIALIZER_H
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_free.hpp>
#include <LoopsLib/DS/EmbeddedGraph.h>
#include <movetk/geom/GeometryInterface.h>


namespace boost::serialization
{
    template<typename Archive>
    inline void save(Archive& ar, const LoopsLib::DS::EmbeddedGraph& res, unsigned int version)
    {
        auto nvp = [](const char* name, auto& t)
        {
            return make_nvp(name, t);
        };
        auto vCount = res.numberOfVertices();
        ar << make_nvp("vertexCount", vCount);
        auto eCount = res.numberOfEdges();
        ar << make_nvp("edgeCount", eCount);

        for(const auto& v : res.vertices())
        {
            ar & v->outEdges().size();
            for(const auto& e : v->outEdges())
            {
                ar & e->id();
                ar & e->m_sink->id();
            }
        }
        // Locations
        for(const auto& v: res.vertices())
        {
            ar & res.locations()[v->id()].get().x();
            ar & res.locations()[v->id()].get().y();
        }
    }
    template<typename Archive>
    inline void load(Archive& ar, LoopsLib::DS::EmbeddedGraph& res, unsigned int version)
    {
        auto nvp = [](const char* name, auto& t)
        {
            return make_nvp(name, t);
        };
        LoopsLib::DS::BaseGraph::Id_t vCount, eCount;
        ar & make_nvp("vertexCount", vCount);
        ar & make_nvp("edgeCount", eCount);
        res.allocateVertices(vCount);
        res.allocateVertices(eCount);
        for (const auto& v : res.vertices())
        {
            // Number of out edges of v.
            LoopsLib::DS::BaseGraph::Id_t vECount;
            ar & vECount;
            for (LoopsLib::DS::BaseGraph::Id_t i = 0; i < vECount; ++i)
            {
                LoopsLib::DS::BaseGraph::Id_t eId,sink;
                ar & eId;
                ar & sink;
                res.setEdgeConnection(eId, v->id(), sink);
            }
        }
        res.locations().resize(vCount);
        auto mkPoint = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();
        for (const auto& v : res.vertices())
        {
            LoopsLib::NT x, y;
            ar & x;
            ar & y;
            res.locations().push_back(mkPoint({ x,y }));
        }
    }
}
BOOST_SERIALIZATION_SPLIT_FREE(LoopsLib::DS::EmbeddedGraph)
#endif