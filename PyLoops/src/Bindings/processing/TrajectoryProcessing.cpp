#include <PyLoops/Bindings/processing/TrajectoryProcessing.h>
#include <LoopsLib/Algs/Types.h>
#include "LoopsIO/WkbCsvTrajectoryReader.h"
#include <movetk/algo/Segmentation.h>
#include <movetk/algo/SegmentationTraits.h>
#include <movetk/algo/SegmentationPredicates.h>


void PyLoops::processing::TrajectoryProcessing::readTrajectories(const std::string& sourceFile, const CsvConfig& config, FileType fileType, 
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

void PyLoops::processing::TrajectoryProcessing::splitWkbCsvByMeb(const std::string& inputFile,
    const std::string& outputFile)
{
    std::vector<fmm::FmmMMTrajectoryVisitor::Trajectory_t> trajectories;
    CsvConfig c{ ';',0,1 };
    readTrajectories(inputFile, c, FileType::WkbCsv, trajectories);

    /*typedef movetk_algorithms::SegmentationTraits<LoopsLib::NT,
        LoopsLib::MovetkGeometryKernel, LoopsLib::KernelDef::dimensions> SegmentationTraits;
    typedef LoopsLib::MovetkGeometryKernel::NT NT;
    typedef vector<SegmentationTraits::Point> PolyLine;
    typedef std::vector<PolyLine::const_iterator> SegmentIdx;
    SegmentationTraits::LocationSegmentation segment_by_meb(10);

    for(const auto& trajectory: trajectories)
    {
        segment_by_meb(trajectory.geom.)
    }*/
}