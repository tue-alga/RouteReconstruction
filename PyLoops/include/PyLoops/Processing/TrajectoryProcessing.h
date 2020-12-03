#ifndef PYLOOPS_PROCESSING_TRAJECTORYPROCESSING_H
#define PYLOOPS_PROCESSING_TRAJECTORYPROCESSING_H
#include <PyLoops/PyLoops.inc.h>
#include <pybind11/pybind11.h>
#include "LoopsLib/DS/EmbeddedGraph.h"
#include "IO/FmmMMTrajectoryVisitor.h"

namespace PyLoops::processing
{
    class TrajectoryProcessing
    {
    public:
        struct CsvConfig
        {
            char delim;
            int trajectoryColumn;
            int trajectoryIdColumn;
        };
        enum class FileType
        {
            WkbCsv,
            WktCsv
        };
    private:
        LoopsLib::DS::EmbeddedGraph* m_graph = nullptr;
        void readTrajectories(const std::string& sourceFile, const CsvConfig& config, FileType fileType, std::vector<fmm::FmmMMTrajectoryVisitor::Trajectory_t>& output);

        void readTimeStampedTrajectories(const std::string& sourceFile, const std::string& fileType, LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& trajectories);
        void readTrajectories(const std::string& sourceFile, const std::string& fileType, LoopsLib::MovetkGeometryKernel::TrajectorySet& trajectories);
    public:
        using NT = LoopsLib::NT;
        static void registerPy(pybind11::module& mod);

        void splitWkbCsvByMeb(const std::string& inputFile, const std::string& outputFile);

        static void splitOnTime(const std::string& inputFile, double splitDifference, int idColumn, int trajectoryColumn, const std::string& outputFile);

        static void filterOnMinLength(const std::string& inputFile, int minLength, int idColumn, int trajectoryColumn, const std::string& outputFile);

        static void filterOnBoundingBox(const std::string& inputFile, const std::tuple<NT,NT,NT,NT>& bbox, int idColumn, int trajectoryColumn, const std::string& outputFile);
        static void filterOnGraphBoundingBox(const std::string& inputFile, const LoopsLib::DS::EmbeddedGraph& graph, int idColumn, int trajectoryColumn, const std::string& outputFile, LoopsLib::NT bboxScale);
    };
}

#endif