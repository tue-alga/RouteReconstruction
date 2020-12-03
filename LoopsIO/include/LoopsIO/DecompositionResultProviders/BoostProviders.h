#ifndef LOOPSIO_DECOMPOSITIONRESULTPROVIDERS_BOOSTPROVIDERS_H
#define LOOPSIO_DECOMPOSITIONRESULTPROVIDERS_BOOSTPROVIDERS_H
#include "LoopsIO/ISerializer.h"
#include <LoopsLib/Models/DecompositionResult.h>
#include <LoopsIO/Serializers.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
namespace LoopsIO::DecompositionResultProviders
{
    class BoostTxtProvider : public TypedSerializer<LoopsLib::Models::DecompositionResult>
    {
    public:
        BoostTxtProvider()
            : TypedSerializer<LoopsLib::Models::DecompositionResult>("boostTxt", "BoostTXT(*.boosttxt)", "boosttxt")
        {
        }

        void read(const std::string& fileName, LoopsLib::Models::DecompositionResult& out) override
        {
            std::ifstream stream(fileName);
            if(!stream.is_open())
            {
                throw std::runtime_error("Could not open " + fileName);
            }
            boost::archive::text_iarchive arch(stream);
            arch & out;
            std::cout << "Read result with instance " << out.m_relatedInstance->m_savePaths.instancePath << std::endl;
        }
        void write(const std::string& fileName, const LoopsLib::Models::DecompositionResult& out) override
        {
            std::ofstream stream(fileName);
            if (!stream.is_open())
            {
                throw std::runtime_error("Could not open " + fileName);
            }
            boost::archive::text_oarchive arch(stream);
            arch & out;
        }
    };
    class BoostBinProvider : public TypedSerializer<LoopsLib::Models::DecompositionResult>
    {
    public:
        BoostBinProvider()
            : TypedSerializer<LoopsLib::Models::DecompositionResult>("boostBin", "BoostBIN(*.boostbin)", "boostbin")
        {
        }

        void read(const std::string& fileName, LoopsLib::Models::DecompositionResult& out) override
        {
            std::ifstream stream(fileName);
            if (!stream.is_open())
            {
                throw std::runtime_error("Could not open " + fileName);
            }
            boost::archive::binary_iarchive arch(stream);
            arch & out;
        }
        void write(const std::string& fileName, const LoopsLib::Models::DecompositionResult& out) override
        {
            std::ofstream stream(fileName);
            if (!stream.is_open())
            {
                throw std::runtime_error("Could not open " + fileName);
            }
            boost::archive::binary_oarchive arch(stream);
            arch & out;
        }
    };
}
#endif