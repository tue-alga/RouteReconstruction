#ifndef LOOPSIO_SERIALIZERS_PROBLEMINSTANCESERIALIZER_H
#define LOOPSIO_SERIALIZERS_PROBLEMINSTANCESERIALIZER_H
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <LoopsLib/Models/ProblemInstance.h>
#include "BaseTypes.h"
namespace boost::serialization
{
    template<typename Archive>
    inline void load(Archive& ar, LoopsLib::Models::ProblemInstance::TrajectorySource& res, unsigned int version)
    {
        ar >> make_nvp("pathSource", res.m_pathSource);
        ar >> make_nvp("trajectoryId", res.m_trajectoryId);
    }
    template<typename Archive>
    inline void save(Archive& ar, const LoopsLib::Models::ProblemInstance::TrajectorySource& res, unsigned int version)
    {
        ar << make_nvp("pathSource", res.m_pathSource);
        ar << make_nvp("trajectoryId", res.m_trajectoryId);
    }

    template<typename Archive>
    inline void load(Archive& ar, LoopsLib::Models::ProblemInstance& res, unsigned int version)
    {
        ar >> make_nvp("graphFile", res.m_savePaths.graphPath);
        ar >> make_nvp("fieldFile", res.m_savePaths.fieldPath);
        std::size_t representativesCount = 0;
        ar >> make_nvp("availablePathCount", representativesCount);
        res.m_representatives.reserve(representativesCount);
        res.m_representativeSources.reserve(representativesCount);
        ar >> make_nvp("epsilon", res.m_epsilon);
        ar >> make_nvp("representativeSources", res.m_representativeSources);
        ar >> make_nvp("representatives", res.m_representatives);
    }
    template<typename Archive>
    inline void save(Archive& ar, const LoopsLib::Models::ProblemInstance& res, unsigned int version)
    {
        ar << make_nvp("graphFile", res.m_savePaths.graphPath);
        ar << make_nvp("fieldFile", res.m_savePaths.fieldPath);
        auto representativeCount = res.m_representatives.size();
        ar << make_nvp("availablePathCount", representativeCount);
        ar << make_nvp("epsilon", res.m_epsilon);
        ar << make_nvp("representativeSources", res.m_representativeSources);
        ar << make_nvp("representatives", res.m_representatives);
    }
}
BOOST_SERIALIZATION_SPLIT_FREE(LoopsLib::Models::ProblemInstance)
BOOST_SERIALIZATION_SPLIT_FREE(LoopsLib::Models::ProblemInstance::TrajectorySource)
#endif