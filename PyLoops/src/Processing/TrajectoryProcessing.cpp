#include <PyLoops/Processing/TrajectoryProcessing.h>
#include <LoopsLib/Algs/Types.h>
#include "LoopsIO/WkbCsvTrajectoryReader.h"
#include <movetk/algo/Segmentation.h>
#include <movetk/algo/SegmentationTraits.h>
#include <movetk/algo/SegmentationPredicates.h>
#include "LoopsIO/TrajectorySetSerializer.h"
#include <pybind11/stl.h>
#include "LoopsAlgs/Trajectories/Coverage.h"
#include <movetk/algo/Similarity.h>

// Ugh
struct Norm
{
    using Point = LoopsLib::MovetkGeometryKernel::MovetkPoint;
    using NT = LoopsLib::MovetkGeometryKernel::NT;
    NT operator()(const Point& p0, const Point& p1) const
    {
        return (p0 - p1).length();
    }
};

void PyLoops::processing::TrajectoryProcessing::readTrajectories(const std::string& sourceFile, const TrajectoryProcessing::CsvConfig& config, TrajectoryProcessing::FileType fileType,
    std::vector<fmm::FmmMMTrajectoryVisitor::Trajectory_t>& output)
{
    
    fmm::FmmMMTrajectoryVisitor visitor(output, 60);

    
    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&ref, &m_graph->spatialRef());
    visitor.setTransform(transform);

    if(fileType == FileType::WktCsv)
    {
        // Read all csv trajectory from well known text
        LoopsIO::WktCsvTrajectoryReader reader(config.delim, config.trajectoryColumn, config.trajectoryIdColumn);
        reader.setInterruptor(LoopsIO::WktCsvTrajectoryReader::InterruptHooks{
            []()
            {
                return PyErr_CheckSignals() != 0;
            }, 10
            });
        reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);
    }
    else
    {
        // Read all csv trajectory from well known text
        LoopsIO::WkbCsvTrajectoryReader reader(config.delim, config.trajectoryColumn, config.trajectoryIdColumn);
        reader.setInterruptor(LoopsIO::WkbCsvTrajectoryReader::InterruptHooks{
            []()
            {
                return PyErr_CheckSignals() != 0;
            }, 10
            });
        reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);
    }

    // Destroy the transform
    OGRCoordinateTransformation::DestroyCT(transform);
}

void PyLoops::processing::TrajectoryProcessing::readTimeStampedTrajectories(const std::string& sourceFile,
    const std::string& fileType, LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& trajectories)
{
    std::string inputFile = sourceFile;
    if (fileType == "wkb")
    {
        if (inputFile.rfind("wkbcsv") == std::string::npos)
        {
            auto pos = inputFile.rfind(".csv");
            inputFile.replace(pos, 4, ".wkbcsv");
        }
    }
    else
    {
        if (inputFile.rfind("wktcsv") == std::string::npos)
        {
            auto pos = inputFile.rfind(".csv");
            inputFile.replace(pos, 4, ".wktcsv");
        }
    }

    LoopsIO::TimestampedTrajectorySetSerializer::read(inputFile, trajectories);
}

void PyLoops::processing::TrajectoryProcessing::readTrajectories(const std::string& sourceFile,
    const std::string& fileType, LoopsLib::MovetkGeometryKernel::TrajectorySet& trajectories)
{
    std::string inputFile = sourceFile;
    if (fileType == "wkb")
    {
        if (inputFile.rfind("wkbcsv") == std::string::npos)
        {
            auto pos = inputFile.rfind(".csv");
            inputFile.replace(pos, 4, ".wkbcsv");
        }
    }
    else
    {
        if (inputFile.rfind("wktcsv") == std::string::npos)
        {
            auto pos = inputFile.rfind(".csv");
            inputFile.replace(pos, 4, ".wktcsv");
        }
    }

    LoopsIO::TrajectorySetSerializer::read(inputFile, trajectories);
}

void PyLoops::processing::TrajectoryProcessing::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;
    using TP = PyLoops::processing::TrajectoryProcessing;
    mod.def("splitTrajectoriesOnTime", &TP::splitOnTime)
    .def("filterTrajectoriesOnLength", &TP::filterOnMinLength)
    .def("filterOnBoundingBox",&TP::filterOnBoundingBox)
    .def("filterOnGraphBoundingBox", &TP::filterOnGraphBoundingBox)
    .def("computeFrechet",[](const LoopsLib::MovetkGeometryKernel::Trajectory& t0, const LoopsLib::MovetkGeometryKernel::Trajectory& t1, NT precision)
    {
        using Kernel = LoopsLib::MovetkGeometryKernel;
        movetk_algorithms::StrongFrechetDistance<Kernel, Norm> sfd;
        sfd.setTolerance(precision);
        using Mode = decltype(sfd.mode());
        sfd.setMode(Mode::DoubleAndSearch);
        return sfd(t0, t1);
    })
    ;
}

void PyLoops::processing::TrajectoryProcessing::splitWkbCsvByMeb(const std::string& inputFile,
    const std::string& outputFile)
{
    std::vector<fmm::FmmMMTrajectoryVisitor::Trajectory_t> trajectories;
    CsvConfig c{ ';',0,1 };
    readTrajectories(inputFile, c, FileType::WkbCsv, trajectories);
}

void PyLoops::processing::TrajectoryProcessing::splitOnTime(const std::string& inputFile, double splitDifference,
    int idColumn, int trajectoryColumn, const std::string& outputFile)
{
    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet set;
    IO::IOHelpers::Uri uri(inputFile, {});
    uri.addArg("idCol", idColumn);
    uri.addArg("trajectoryCol", trajectoryColumn);
    // Assuming here that these arguments are enough
    LoopsIO::TimestampedTrajectorySetSerializer::read(inputFile, uri.encoded(), set);

    
    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet splitSet;
    std::size_t index = 0;
    for(auto& traj: set)
    {
        std::size_t prevBegin = 0;
        for(std::size_t i = 1; i < traj.size(); ++i)
        {
            if(traj[i].second - traj[i-1].second > splitDifference)
            {
                if(i-1 - prevBegin > 0)
                {
                    splitSet.ids.push_back(set.ids[index] + "_part_" + std::to_string(prevBegin) + "_" + std::to_string(i - 1));
                    // Add empty trajectory to be filled
                    splitSet.trajectories.emplace_back();
                    // Copy
                    std::copy(traj.begin() + prevBegin, traj.begin() + i, std::back_inserter(splitSet.trajectories.back()));
                }
                prevBegin = i;
            }
        }
        if (traj.size()-1 - prevBegin > 0)
        {
            splitSet.ids.push_back(set.ids[index] + "_part_" + std::to_string(prevBegin) + "_" + std::to_string(traj.size() - 1));
            // Add empty trajectory to be filled
            splitSet.trajectories.emplace_back();
            // Copy
            std::copy(traj.begin() + prevBegin, traj.begin() + traj.size(), std::back_inserter(splitSet.trajectories.back()));
        }
        ++index;
    }
    LoopsIO::TimestampedTrajectorySetSerializer::write(outputFile, splitSet);
}

void PyLoops::processing::TrajectoryProcessing::filterOnMinLength(const std::string& inputFile, int minLength,
    int idColumn, int trajectoryColumn, const std::string& outputFile)
{
    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet set;
    IO::IOHelpers::Uri uri(inputFile, {});
    uri.addArg("idCol", idColumn);
    uri.addArg("trajectoryCol", trajectoryColumn);
    // Assuming here that these arguments are enough
    LoopsIO::TimestampedTrajectorySetSerializer::read(inputFile, uri.encoded(), set);


    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet filteredSet;
    filteredSet.m_ref = set.m_ref;
    std::size_t index = 0;
    for (auto& traj : set)
    {
        if(traj.size() >= minLength)
        {
            filteredSet.trajectories.push_back(traj);
            filteredSet.ids.push_back(set.ids[index]);
        }
        ++index;
    }
    LoopsIO::TimestampedTrajectorySetSerializer::write(outputFile, filteredSet);
}

void PyLoops::processing::TrajectoryProcessing::filterOnBoundingBox(const std::string& inputFile,
    const std::tuple<NT, NT, NT, NT>& bbox, int idColumn, int trajectoryColumn, const std::string& outputFile)
{
    using Kernel = LoopsLib::MovetkGeometryKernel;

    Kernel::TimestampedTrajectorySet set;
    IO::IOHelpers::Uri uri(inputFile, {});
    uri.addArg("idCol", idColumn);
    uri.addArg("trajectoryCol", trajectoryColumn);
    // Assuming here that these arguments are enough
    LoopsIO::TimestampedTrajectorySetSerializer::read(inputFile, uri.encoded(), set);

    
    std::pair<NT, NT> xRange = std::make_pair(std::get<0>(bbox), std::get<1>(bbox));
    std::pair<NT, NT> yRange = std::make_pair(std::get<2>(bbox), std::get<3>(bbox));
    auto inBox= [&xRange,&yRange](const LoopsLib::MovetkGeometryKernel::MovetkPoint &val)
    {
        return (xRange.first <= val.x() && val.x() <= xRange.second) && (yRange.first <= val.y() && val.y() <= yRange.second);
    };

    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet filteredSet;
    filteredSet.m_ref = set.m_ref;
    std::size_t index = 0;
    for (auto& traj : set)
    {
        long long insideIndex = -1;
        for (std::size_t i = 0; i < traj.size(); ++i)
        {
            const auto& pnt = traj[i];
            if (inBox(pnt.first))
            {
                // Record start of inside the box.
                if (insideIndex == -1) insideIndex = i;
            }
            else
            {
                if (insideIndex == -1) continue;
                if (i - 1 - insideIndex > 1)
                {
                    filteredSet.trajectories.push_back(Kernel::TimestampedTrajectory(set.trajectories[index].begin() + insideIndex, set.trajectories[index].begin() + i));
                    filteredSet.ids.push_back(set.ids[index] + "_bboxsub" + std::to_string(insideIndex) + "_" + std::to_string(i));
                }
                insideIndex = -1;
            }
        }
        if (insideIndex != -1 && traj.size() - insideIndex > 1)
        {
            filteredSet.trajectories.push_back(Kernel::TimestampedTrajectory(set.trajectories[index].begin() + insideIndex, set.trajectories[index].begin() + traj.size()));
            filteredSet.ids.push_back(set.ids[index] + "_bboxsub" + std::to_string(insideIndex) + "_" + std::to_string(traj.size()));
        }
        ++index;
    }
    LoopsIO::TimestampedTrajectorySetSerializer::write(outputFile, filteredSet);
}

void PyLoops::processing::TrajectoryProcessing::filterOnGraphBoundingBox(const std::string& inputFile,
    const LoopsLib::DS::EmbeddedGraph& graph, int idColumn, int trajectoryColumn, const std::string& outputFile, LoopsLib::NT bboxScale)
{
    using Kernel = LoopsLib::MovetkGeometryKernel;

    Kernel::TimestampedTrajectorySet set;
    IO::IOHelpers::Uri uri(inputFile, {});
    uri.addArg("idCol", idColumn);
    uri.addArg("trajectoryCol", trajectoryColumn);
    // Assuming here that these arguments are enough
    LoopsIO::TimestampedTrajectorySetSerializer::read(inputFile, uri.encoded(), set);

    // Make a copy 
    Kernel::TimestampedTrajectorySet orig = set;

    if(!set.m_ref.IsSame(graph.spatialRefPntr()))
    {
        for(auto& traj: set)
        {
            LoopsLib::MovetkGeometryKernel().convertCRS(set.m_ref, graph.spatialRef(), traj);
        }
    }
    const auto negInf = std::numeric_limits<NT>::lowest();
    const auto posInf = std::numeric_limits<NT>::max();
    std::pair<NT, NT> xRange = std::make_pair(posInf, negInf);
    std::pair<NT, NT> yRange = std::make_pair(posInf, negInf);
    for(const auto& loc: graph.locations())
    {
        xRange.first = std::min(xRange.first, loc.x());
        xRange.second = std::max(xRange.second, loc.x());
        yRange.first = std::min(yRange.first, loc.y());
        yRange.second = std::max(yRange.second, loc.y());
    }
    // Apply scaling
    {
        auto xAvg = 0.5 * (xRange.first + xRange.second);
        auto xSz = (xRange.second - xRange.first)*bboxScale;
        xRange = std::make_pair(xAvg - 0.5*xSz, xAvg + 0.5*xSz);
        auto yAvg = 0.5 * (yRange.first + yRange.second);
        auto ySz = (yRange.second - yRange.first)*bboxScale;
        yRange = std::make_pair(yAvg - 0.5*ySz, yAvg + 0.5*ySz);
    }

    auto inBox = [&xRange, &yRange](const LoopsLib::MovetkGeometryKernel::MovetkPoint &val)
    {
        return (xRange.first <= val.x() && val.x() <= xRange.second) && (yRange.first <= val.y() && val.y() <= yRange.second);
    };


    LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet filteredSet;
    filteredSet.m_ref = set.m_ref;
    std::size_t index = 0;
    for (auto& traj : set)
    {
        long long insideIndex = -1;
        for (std::size_t i = 0; i < traj.size(); ++i)
        {
            const auto& pnt = traj[i];
            if (inBox(pnt.first))
            {
                // Record start of inside the box.
                if (insideIndex == -1) insideIndex = i;
            }
            else
            {
                if (insideIndex == -1) continue;
                if (i - 1 - insideIndex > 1)
                {
                    filteredSet.trajectories.push_back(Kernel::TimestampedTrajectory(orig.trajectories[index].begin() + insideIndex, orig.trajectories[index].begin() + i));
                    filteredSet.ids.push_back(set.ids[index] + "_bboxsub" + std::to_string(insideIndex) + "_" + std::to_string(i));
                }
                insideIndex = -1;
            }
        }
        if (insideIndex != -1 && traj.size() - insideIndex > 1)
        {
            filteredSet.trajectories.push_back(Kernel::TimestampedTrajectory(orig.trajectories[index].begin() + insideIndex, orig.trajectories[index].begin() + traj.size()));
            filteredSet.ids.push_back(set.ids[index] + "_bboxsub" + std::to_string(insideIndex) + "_" + std::to_string(traj.size()));
        }
        ++index;
    }
    LoopsIO::TimestampedTrajectorySetSerializer::write(outputFile, filteredSet);
}
