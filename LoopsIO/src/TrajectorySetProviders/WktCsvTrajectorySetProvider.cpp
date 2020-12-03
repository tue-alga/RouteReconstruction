#include <LoopsIO/TrajectorySetProviders/WktCsvTrajectorySetProvider.h>
#include "OGRInterface.h"
#include <fstream>
#include <iostream>

using NT = LoopsLib::NT;

struct WktReader
{
    OGRErr operator()(const std::string& data, OGRGeometry** output)
    {
        // TODO result of createFromWkb
        return OGRGeometryFactory::createFromWkt(data.data(), nullptr, output);
    }
};

void LoopsIO::TrajectorySetProviders::WktCsvTrajectorySetProvider::read(const std::string& fileName,
    LoopsLib::MovetkGeometryKernel::TrajectorySet& out)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(fileName, path, args);
    const int trajectoryCol = std::stoi(args.at("trajectoryCol"));
    const int idCol = std::stoi(args.at("idCol"));

    CsvLikeOgrTrajectoryReader reader(';', trajectoryCol, idCol);
    OgrTrajectoryVisitor visit(out);
    reader.read(path, visit, WktReader(), true);
    // Default to WGS84
    out.m_ref.SetWellKnownGeogCS("WGS84");
}

void LoopsIO::TrajectorySetProviders::WktCsvTrajectorySetProvider::write(const std::string& fileName,
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& in)
{
    std::ofstream out(fileName);
    if(!out.is_open())
    {
        std::cout << "Could not open file for writing: " << fileName;
        return;
    }
    out << "ID;TRAJECTORY\n";
    for(auto i = 0; i < in.trajectories.size(); ++i)
    {
        if (i != 0) out << '\n';
        out << in.ids[i] << ';';
        // Encode trajectory
        OGRLineString traj;
        for(const auto& el: in.trajectories[i])
        {
            traj.addPoint(el.m_x, el.m_y);
        }
        char* data;
        traj.exportToWkt(&data);
        out << data;
        CPLFree(data);
    }
}

std::vector<std::string> LoopsIO::TrajectorySetProviders::WktCsvTrajectorySetProvider::requiredArguments() const
{
    return { "idCol","trajectoryCol" };
}

void LoopsIO::TrajectorySetProviders::WktCsvTimestampedTrajectorySetProvider::read(const std::string& fileName,
    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& out)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(fileName, path, args);
    const int trajectoryCol = std::stoi(args.at("trajectoryCol"));
    const int idCol = std::stoi(args.at("idCol"));

    CsvLikeOgrTrajectoryReader reader(';', trajectoryCol, idCol);
    OgrTimestampedTrajectoryVisitor visit(out);
    reader.read(path, visit, WktReader(), true);
    // Default to WGS84
    out.m_ref.SetWellKnownGeogCS("WGS84");
}

void LoopsIO::TrajectorySetProviders::WktCsvTimestampedTrajectorySetProvider::write(const std::string& fileName,
    const LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& in)
{
    std::ofstream out(fileName);
    if (!out.is_open())
    {
        std::cout << "Could not open file for writing: " << fileName;
        return;
    }
    out << "ID;TRAJECTORY\n";
    for (auto i = 0; i < in.trajectories.size(); ++i)
    {
        if (i != 0) out << '\n';
        out << in.ids[i] << ';';
        // Encode trajectory
        OGRLineString traj;
        traj.setMeasured(true);
        for (const auto& el : in.trajectories[i])
        {
            traj.addPointM(el.first.m_x, el.first.m_y, el.second);
        }
        char* data;
        traj.exportToWkt(&data, OGRwkbVariant::wkbVariantIso);
        out << data;
        CPLFree(data);
    }
}

std::vector<std::string> LoopsIO::TrajectorySetProviders::WktCsvTimestampedTrajectorySetProvider::
requiredArguments() const
{
    return { "idCol","trajectoryCol" };
}
