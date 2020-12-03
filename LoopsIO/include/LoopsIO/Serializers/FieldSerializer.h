#ifndef LOOPSIO_SERIALIZERS_FIELDSERIALIZER_H
#define LOOPSIO_SERIALIZERS_FIELDSERIALIZER_H
#include <boost/serialization/serialization.hpp>
#include <LoopsLib/Models/FlowField.h>
#include <boost/serialization/vector.hpp>
namespace boost::serialization
{
    template<typename Archive>
    inline void serialize(Archive& ar, LoopsLib::Models::FlowField& res, unsigned int version)
    {
        auto nvp = [](const char* name, auto& t)
        {
            return make_nvp(name, t);
        };
        ar & make_nvp("paths", res.m_paths);
        ar & make_nvp("pathValues", res.m_pathValues);
    }
}
#endif