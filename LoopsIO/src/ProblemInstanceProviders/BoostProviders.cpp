#include <LoopsIO/ProblemInstanceProviders/BoostProviders.h>

#include <LoopsIO/Serializers.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

LoopsIO::ProblemInstanceProviders::BoostTxtProvider::BoostTxtProvider(): TypedSerializer<LoopsLib::Models::
    ProblemInstance>("boostTxt", "BoostTXT(*.boosttxt)", "boosttxt")
{
}

void LoopsIO::ProblemInstanceProviders::BoostTxtProvider::read(const std::string& fileName,
                                                               LoopsLib::Models::ProblemInstance& out)
{
    std::ifstream stream(fileName);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open " + fileName);
    }
    boost::archive::text_iarchive arch(stream);
    arch & out;
}

void LoopsIO::ProblemInstanceProviders::BoostTxtProvider::write(const std::string& fileName,
                                                                const LoopsLib::Models::ProblemInstance& out)
{
    std::ofstream stream(fileName);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open " + fileName);
    }
    boost::archive::text_oarchive arch(stream);
    arch & out;
}

LoopsIO::ProblemInstanceProviders::BoostBinProvider::BoostBinProvider(): TypedSerializer<LoopsLib::Models::
    ProblemInstance>("boostBin", "BoostBIN(*.boostbin)", "boostbin")
{
}

void LoopsIO::ProblemInstanceProviders::BoostBinProvider::read(const std::string& fileName,
                                                               LoopsLib::Models::ProblemInstance& out)
{
    std::ifstream stream(fileName);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open " + fileName);
    }
    boost::archive::binary_iarchive arch(stream);
    arch & out;
}

void LoopsIO::ProblemInstanceProviders::BoostBinProvider::write(const std::string& fileName,
                                                                const LoopsLib::Models::ProblemInstance& out)
{
    std::ofstream stream(fileName);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open " + fileName);
    }
    boost::archive::binary_oarchive arch(stream);
    arch & out;
}
