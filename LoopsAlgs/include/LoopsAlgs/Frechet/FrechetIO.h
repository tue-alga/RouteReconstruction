#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_free.hpp>
#include "FrechetHelpers.h"


namespace boost::serialization
{
    template<typename Archive>
    inline void save(Archive& ar, const LoopsLib::DS::GraphView& res, unsigned int version)
    {
        
    }
    template<typename Archive>
    inline void load(Archive& ar, LoopsLib::DS::GraphView& res, unsigned int version)
    {

    }
    template<typename Archive>
    inline void serialize(Archive& ar, LoopsLib::Geometry::Interval& interval, unsigned int version)
    {
        ar & interval.min;
        ar & interval.max;
    }

    template<typename Archive>
    inline void save(Archive& ar, const LoopsAlgs::Frechet::StrongFrechetGraphData& res, unsigned int version)
    {
        auto nvp = [](const char* name, auto& t)
        {
            return make_nvp(name, t);
        };
        ar << res.polyline.size();
        for(std::size_t i = 0; i < res.polyline.size(); ++i)
        {
            ar << res.polyline[i].get().source().x();
            ar << res.polyline[i].get().source().y();
            ar << res.polyline[i].get().target().x();
            ar << res.polyline[i].get().target().y();
        }
        // Serialize FDiWhiteIntervals
        ar << res.FDiWhiteIntervals.size();
        for(const auto& el : res.FDiWhiteIntervals)
        {
            ar << el.size();
            for(const auto &i : el)
            {
                ar << i;
            }
        }
        // Serialize WhiteIntervals
        ar << res.WhiteIntervals.size();
        for (const auto& el : res.WhiteIntervals)
        {
            ar << el.size();
            for (const auto &i : el)
            {
                ar << i;
            }
        }

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