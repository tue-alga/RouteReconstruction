#ifndef PYLOOPS_PROCESSING_FMM_H
#define PYLOOPS_PROCESSING_FMM_H
#include <PyLoops/PyLoops.inc.h>
#include <mm/fmm/fmm_algorithm.hpp>
#include <mm/fmm/ubodt.hpp>
#include <odt/compute_ubodt.h>
#include <pybind11/pybind11.h>
#include <GeographicLib/UTMUPS.hpp>

namespace PyLoops::processing {
    template<typename PntrType>
    struct ConstType
    {
        using type = PntrType const;
        static type cast(PntrType val)
        {
            return static_cast<type>(val);
        }
    };
    class PyUbodt
    {
    public:
        static void registerPy(pybind11::module& mod);
    };
    class PyFmm
    {
        // The graph to map match to    
        LoopsLib::DS::EmbeddedGraph* m_graph;
        // The upperbounded origin-destination table (for shortest paths)
        std::shared_ptr<FMM::MM::UBODT_Alternative> m_table;
        // The mapmatching algorithm
        std::shared_ptr<FMM::MM::FastMapMatch> m_mapMatch;
    public:
        PyFmm(LoopsLib::DS::EmbeddedGraph* graph);

        PyFmm(LoopsLib::DS::EmbeddedGraph* graph, const std::string& tableFilePath);

        void setTable(std::shared_ptr<FMM::MM::UBODT_Alternative> pntr);
        void printFirstWkbCsv(const std::string& sourceFile) const;

        FMM::MM::MatchResult mapMatch(const std::pair<std::vector<std::pair<double, double>>, std::vector<double>>& traj) const;
        FMM::MM::MatchResult mapMatchTraj(const FMM::CORE::Trajectory& traj) const;

        void mapMatchWkbCsv(const std::string& sourceFile, const std::string& outFile, const std::string& delim, int trajectoryColumn) const;

        std::vector<FMM::CORE::Trajectory> readWkbCsvTrajectories(const std::string& sourceFile, const std::string& delim, int trajectoryColumn) const;

        void mapMatchWkbCsvConcurrent(const std::string& sourceFile, const std::string& outFile, const std::string& delim, int trajectoryColumn, std::size_t threadNum) const;

        static void registerPy(pybind11::module& mod);
    };
}
#endif