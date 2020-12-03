#ifndef LOOPSIO_SERIALIZERS_BASETYPES_H
#define LOOPSIO_SERIALIZERS_BASETYPES_H
#include <LoopsLib/Algs/Types.h>
#include <boost/serialization/vector.hpp>

namespace boost::serialization
{
    template<typename Archive>
    inline void serialize(Archive& ar, LoopsLib::MovetkGeometryKernel::MovetkPoint& res, unsigned int version)
    {
        ar & make_nvp("x", res.m_x);
        ar & make_nvp("y", res.m_y);
    }
}
#endif