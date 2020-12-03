#ifndef GEOMETRY_BOOSTINTERFACE_H
#define GEOMETRY_BOOSTINTERFACE_H
#include <LoopsLib/Algs/Types.h>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

namespace LoopsLib::specialize
{
    using MtkPoint = typename MovetkGeometryKernel::MovetkPoint;
    using NT = typename LoopsLib::NT;
}


namespace boost::geometry::traits
{
    BOOST_GEOMETRY_DETAIL_SPECIALIZE_POINT_TRAITS(LoopsLib::specialize::MtkPoint, 2, LoopsLib::specialize::NT, cs::cartesian)

        template<> struct access<LoopsLib::specialize::MtkPoint, 0> {
        static inline LoopsLib::specialize::NT get(LoopsLib::specialize::MtkPoint const& p) { return p.get().x(); }
        static inline void set(LoopsLib::specialize::MtkPoint& p, LoopsLib::specialize::NT const& value)
        {
            p.m_x = value;
            //LoopsLib::specialize::NT y = p.get().dimension() != 0 ? p.get().y() : 0;
            //p = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>()({ value, y });
        }
    };
    template<> struct access<LoopsLib::specialize::MtkPoint, 1> {
        static inline LoopsLib::specialize::NT get(LoopsLib::specialize::MtkPoint const& p) { return p.get().y(); }
        static inline void set(LoopsLib::specialize::MtkPoint& p, LoopsLib::specialize::NT const& value)
        {
            p.m_y = value;
            //LoopsLib::specialize::NT x = p.get().dimension() != 0 ? p.get().x() : 0;
            //p = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>()({ x, value });
        }
    };
}

#endif // !GEOMETRY_BOOSTINTERFACE_H
