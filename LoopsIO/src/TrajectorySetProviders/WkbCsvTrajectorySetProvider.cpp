#include <LoopsIO/TrajectorySetProviders/WkbCsvTrajectorySetProvider.h>
#include "OGRInterface.h"
#include <fstream>
#include <iostream>

using NT = LoopsLib::NT;

struct WkbReader
{
    OGRErr operator()(const std::string& data, OGRGeometry** output)
    {
        int byteCount;
        auto del = [](auto* pntr) { CPLFree(pntr); };
        auto bytes = std::unique_ptr<GByte, decltype(del)>(CPLHexToBinary(data.data(), &byteCount), del);
        // TODO result of createFromWkb
        return OGRGeometryFactory::createFromWkb(bytes.get(), nullptr, output, byteCount);
    }
};


void LoopsIO::TrajectorySetProviders::WkbCsvTrajectorySetProvider::read(const std::string& fileName,
    LoopsLib::MovetkGeometryKernel::TrajectorySet& out)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(fileName, path, args);
    const int trajectoryCol = std::stoi(args.at("trajectoryCol"));
    const int idCol = std::stoi(args.at("idCol"));

    CsvLikeOgrTrajectoryReader reader(';', trajectoryCol, idCol);
    OgrTrajectoryVisitor visit(out);
    reader.read(path, visit, WkbReader(), true);
    // Default to WGS84
    out.m_ref.SetWellKnownGeogCS("WGS84");
}

void LoopsIO::TrajectorySetProviders::WkbCsvTrajectorySetProvider::write(const std::string& fileName,
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& in)
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
        for (const auto& el : in.trajectories[i])
        {
            traj.addPoint(el.m_x, el.m_y);
        }
        auto* data = new unsigned char[traj.WkbSize()];
        traj.exportToWkb(OGRwkbByteOrder::wkbNDR, data);
        auto* hexEncoding = CPLBinaryToHex(traj.WkbSize(), data);
        out << hexEncoding;
        CPLFree(hexEncoding);

        delete[] data;
    }
}

std::vector<std::string> LoopsIO::TrajectorySetProviders::WkbCsvTrajectorySetProvider::requiredArguments() const
{
    return { "idCol","trajectoryCol" };
}

void LoopsIO::TrajectorySetProviders::WkbCsvTimestampedTrajectorySetProvider::read(const std::string& fileName,
    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& out)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(fileName, path, args);
    const int trajectoryCol = std::stoi(args.at("trajectoryCol"));
    const int idCol = std::stoi(args.at("idCol"));

    CsvLikeOgrTrajectoryReader reader(';', trajectoryCol, idCol);
    OgrTimestampedTrajectoryVisitor visit(out);
    reader.read(path, visit, WkbReader(), true);
    // Default to WGS84
    out.m_ref.SetWellKnownGeogCS("WGS84");
}

void LoopsIO::TrajectorySetProviders::WkbCsvTimestampedTrajectorySetProvider::write(const std::string& fileName,
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
        unsigned char* data = new unsigned char[traj.WkbSize()];
        traj.exportToWkb(OGRwkbByteOrder::wkbNDR, data, OGRwkbVariant::wkbVariantIso);
        auto* hexEncoding = CPLBinaryToHex(traj.WkbSize(), data);
        out << hexEncoding;
        CPLFree(hexEncoding);

        delete[] data;
    }
}

std::vector<std::string> LoopsIO::TrajectorySetProviders::WkbCsvTimestampedTrajectorySetProvider::
requiredArguments() const
{
    return { "idCol","trajectoryCol" };
}
