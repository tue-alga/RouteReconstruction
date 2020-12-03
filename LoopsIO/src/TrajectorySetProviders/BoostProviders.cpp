#include <LoopsIO/TrajectorySetProviders/BoostProviders.h>
#include <LoopsIO/Serializers/BaseTypes.h>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

LoopsIO::TrajectorySetProviders::BoostBinGraphTrajectorySetProvider::BoostBinGraphTrajectorySetProvider():
    TypedSerializer<LoopsLib::MovetkGeometryKernel::detail::TrajectorySet<std::vector<long long>>>(
        "BoostBin graph trajectory set", "Boost graph trajectory set (*.boostbin)", "boostbin")
{
}

void LoopsIO::TrajectorySetProviders::BoostBinGraphTrajectorySetProvider::read(const std::string& fileName,
    LoopsLib::MovetkGeometryKernel::GraphTrajectorySet& out)
{
    std::ifstream stream(fileName, std::ios::binary | std::ios::in);
    boost::archive::binary_iarchive iArch(stream);
    iArch >> out.ids;
    iArch >> out.trajectories;
    std::string data;
    iArch >> data;
    out.m_ref.importFromWkt(data.data());
}

void LoopsIO::TrajectorySetProviders::BoostBinGraphTrajectorySetProvider::write(const std::string& fileName,
                                                                                const LoopsLib::MovetkGeometryKernel::
                                                                                GraphTrajectorySet& out)
{
    std::ofstream stream(fileName, std::ios::binary | std::ios::out);
    boost::archive::binary_oarchive oArch(stream);
    oArch << out.ids;
    oArch << out.trajectories;

    char* data;
    out.m_ref.exportToWkt(&data);
    oArch << std::string(data);

    // Yuck
    CPLFree(data);
}

LoopsIO::TrajectorySetProviders::BoostTxtGraphTrajectorySetProvider::BoostTxtGraphTrajectorySetProvider() :
    TypedSerializer<LoopsLib::MovetkGeometryKernel::detail::TrajectorySet<std::vector<long long>>>(
        "BoostTxt graph trajectory set", "Boost graph trajectory set (*.boosttxt)", "boosttxt")
{
}

void LoopsIO::TrajectorySetProviders::BoostTxtGraphTrajectorySetProvider::read(const std::string& fileName,
    LoopsLib::MovetkGeometryKernel::GraphTrajectorySet& out)
{
    std::ifstream stream(fileName, std::ios::in);
    boost::archive::text_iarchive iArch(stream);
    iArch >> out.ids;
    iArch >> out.trajectories;
    std::string data;
    iArch >> data;
    out.m_ref.importFromWkt(data.data());
}

void LoopsIO::TrajectorySetProviders::BoostTxtGraphTrajectorySetProvider::write(const std::string& fileName,
    const LoopsLib::MovetkGeometryKernel::GraphTrajectorySet& out)
{
    std::ofstream stream(fileName, std::ios::out);
    boost::archive::text_oarchive oArch(stream);
    oArch << out.ids;
    oArch << out.trajectories;

    char* data;
    out.m_ref.exportToWkt(&data);
    oArch << std::string(data);

    // Yuck
    CPLFree(data);
}
