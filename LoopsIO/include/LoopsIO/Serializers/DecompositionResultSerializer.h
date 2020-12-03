#ifndef LOOPSIO_SERIALIZERS_DECOMPOSITIONRESULTSERIALIZER_H
#define LOOPSIO_SERIALIZERS_DECOMPOSITIONRESULTSERIALIZER_H
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <LoopsLib/Models/DecompositionResult.h>
namespace boost::serialization
{
    template<typename Archive>
    inline void serialize(Archive& ar, LoopsLib::Models::DecompositionResult& res, unsigned int version)
    {
        auto nvp = [](const char* name, auto& t)
        {
            return make_nvp(name, t);
        };
        ar & make_nvp("instancePath", res.m_relatedInstance->m_savePaths.instancePath);
        std::cout << "Decomp inst:"<<res.m_relatedInstance->m_savePaths.instancePath << std::endl;
        ar & make_nvp("basis",res.m_basis);
        ar & make_nvp("decompCoeffs",res.m_decompositionCoeffs);
        ar & make_nvp("nnlsResult", res.m_objectValueNNLS);
    }
}
#endif