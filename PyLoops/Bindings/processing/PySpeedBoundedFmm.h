#ifndef PYLOOPS_PROCESSING_PYSPEEDBOUNDEDFMM_H
#define PYLOOPS_PROCESSING_PYSPEEDBOUNDEDFMM_H
#include <mm/fmm/SpeedBoundedFmm.h>
#include <pybind11/pybind11.h>

namespace PyLoops {
    namespace ds {
        class TrajectoryList;
    }
}

namespace PyLoops::processing {
    class PySpeedBoundedFmm
    {
        // The graph to map match to    
        LoopsLib::DS::EmbeddedGraph* m_graph;
        // The mapmatching algorithm
        std::shared_ptr<FMM::MM::SpeedBoundedFastMapMatch> m_mapMatch;

        struct CsvConfig
        {
            int trajectoryColumn;
            char delim;
            int idColumn;
        };

        void readWkbCsvTrajectories(const std::string& sourceFile, const CsvConfig& config, std::vector<FMM::CORE::Trajectory>& output) const;
        void readWktCsvTrajectories(const std::string& sourceFile, const CsvConfig& config, std::vector<FMM::CORE::Trajectory>& output) const;
    public:
        PySpeedBoundedFmm(LoopsLib::DS::EmbeddedGraph* graph);

        void printFirstWkbCsv(const std::string& sourceFile) const;

        FMM::MM::MatchResult mapMatch(const std::pair<std::vector<std::pair<double, double>>, std::vector<double>>& traj) const;

        void mapMatchWkbCsv(const std::string& sourceFile, const std::string& outFile, const std::string& delim, int trajectoryColumn) const;

        void mapMatchWkbCsvConcurrent(const std::string& sourceFile, const std::string& outFile, const std::string& delim, int trajectoryColumn, std::size_t
                                      threadNum, bool debugMode) const;
        void mapMatchWktCsvConcurrent(const std::string& sourceFile, const std::string& outFile,
                                      const std::string& delim,
                                      int trajectoryColumn, std::size_t threadNum, bool debugMode) const;
        void mapMatchTrajectoryListConcurrent(const ds::TrajectoryList& trajectories, const std::string& outFile, std::size_t threadNum, bool debugMode) const;

        void mapMatchConcurrent(const std::vector<FMM::CORE::Trajectory>& trajectories, const std::string& outFile, std::size_t threadNum, bool debugMode) const;

        static void registerPy(pybind11::module& mod);
    };
}
#endif