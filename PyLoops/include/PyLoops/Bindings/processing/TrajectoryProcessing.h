#ifndef PYLOOPS_PROCESSING_TRAJECTORYPROCESSING_H
#define PYLOOPS_PROCESSING_TRAJECTORYPROCESSING_H
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
        LoopsLib::DS::EmbeddedGraph* m_graph;
        void readTrajectories(const std::string& sourceFile, const CsvConfig& config, FileType fileType, std::vector<fmm::FmmMMTrajectoryVisitor::Trajectory_t>& output);
    public:
        static void registerPy(pybind11::module& mod);

        void splitWkbCsvByMeb(const std::string& inputFile, const std::string& outputFile);
    };
}

#endif