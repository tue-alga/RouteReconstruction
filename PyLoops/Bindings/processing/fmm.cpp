#include "fmm.h"
#include <pybind11/stl.h>
#include "LoopsIO/WkbCsvTrajectoryReader.h"
#include "LoopsIO/OutStreamWrapper.h"
#include "IO/FmmMMTrajectoryVisitor.h"
#include <thread>

#include <chrono>
#include <mutex>

PyLoops::processing::PyFmm::PyFmm(LoopsLib::DS::EmbeddedGraph* graph): m_graph(graph)
{
    fmm::compute_ubodt computer(*m_graph, 40); //Buckets ?
    m_table = computer.computeAlternative(50.0);
    m_mapMatch = std::make_shared<FMM::MM::FastMapMatch>(m_graph, m_table);
}

PyLoops::processing::PyFmm::PyFmm(LoopsLib::DS::EmbeddedGraph* graph, const std::string& tableFilePath): m_graph(graph)
{
    //m_table = FMM::MM::UBODT::read_ubodt_file(tableFilePath);
    m_mapMatch = std::make_shared<FMM::MM::FastMapMatch>(m_graph, m_table);
}

void PyLoops::processing::PyUbodt::registerPy(pybind11::module& mod)
{
    using Ubodt = FMM::MM::UBODT_Alternative;
    pybind11::class_<Ubodt, std::shared_ptr<Ubodt>>(mod, "Ubodt")
        .def("shortestPath", &Ubodt::look_sp_path)
        .def_static("createUbodt", [](LoopsLib::DS::EmbeddedGraph& graph, LoopsLib::NT upperBound, bool directMode)
        {
            if(directMode)
            {
                auto pntr = std::make_shared<Ubodt>(&graph);
                pntr->setDelta(upperBound);
                return pntr;
            }
            fmm::compute_ubodt compute(graph, 10);
            return compute.computeAlternative(upperBound);
        })
    .def("avgConnections",[](Ubodt& table)
    {
            if (table.numberOfVertices() == 0) return 0.0;
            return table.totalSize() / (double)table.numberOfVertices();
    });
}

void PyLoops::processing::PyFmm::setTable(std::shared_ptr<FMM::MM::UBODT_Alternative> pntr)
{
    m_table = pntr;
    m_mapMatch->setODTable(m_table);
}


void PyLoops::processing::PyFmm::printFirstWkbCsv(const std::string& sourceFile) const
{
    LoopsIO::WkbCsvTrajectoryReader reader(';', 2);
    std::vector<FMM::CORE::Trajectory> trajectories;
    fmm::FmmMMTrajectoryVisitor visitor(trajectories, 60);
    reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);

    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&ref, &m_graph->spatialRef());
    for (auto& traj : trajectories)
    {

        for (std::size_t i = 0; i < traj.timestamps.size(); ++i)
        {
            
            double x = traj.geom.get_x(i);
            double y = traj.geom.get_y(i);
            pybind11::print("X-Y:", x, " ", y);
            transform->Transform(1, &x, &y);
            pybind11::print("Transfored X-Y:", x, " ", y);
            traj.geom.set_x(i, x);
            traj.geom.set_y(i, y);
        }
        break;
    }
}

void PyLoops::processing::PyFmm::mapMatchWkbCsv(const std::string& sourceFile, const std::string& outFile, const std::string& delim, int trajectoryColumn) const
{
    LoopsIO::WkbCsvTrajectoryReader reader(delim[0], trajectoryColumn);
    reader.setInterruptor(LoopsIO::WkbCsvTrajectoryReader::InterruptHooks{
        []()
        {
          if (PyErr_CheckSignals() != 0) return true;
          return false;
        }, 10
        });
    std::vector<FMM::CORE::Trajectory> trajectories;
    fmm::FmmMMTrajectoryVisitor visitor(trajectories, 60);
    reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);
    //Open output file and start writing
    std::ofstream stream(outFile);
    if (!stream.is_open()) throw std::runtime_error("Could not open output file");

    LoopsIO::OutStreamWrapper wrapper(stream);

    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&ref, &m_graph->spatialRef());
    for(auto& traj: trajectories)
    {
        
        for(std::size_t i = 0; i < traj.timestamps.size(); ++i)
        {
            double x = traj.geom.get_x(i);
            double y = traj.geom.get_y(i);
            transform->Transform(1, &x, &y);
            traj.geom.set_x(i,x);
            traj.geom.set_y(i, y);
        }
        auto res = m_mapMatch->match_traj(traj);
        //pybind11::print("MM Trajectory of size: ", res.cpath.size());
        if(!res.cpath.empty())
        {
            wrapper.writeJoined(res.cpath, " ").nl();
        }
    }
}

struct MapMatcherJob
{
    // Resides on parent thread, so read only! 
    const std::vector<FMM::CORE::Trajectory>& trajectories;
    const FMM::MM::FastMapMatch& matcher;
    OGRCoordinateTransformation* transform;

    // End flag
    std::atomic_bool& m_interruptFlag;
    std::atomic_bool& m_doneFlag;

    int m_checkEveryNIterations = 10;

    MapMatcherJob(const std::vector<FMM::CORE::Trajectory>& trajectories, const FMM::MM::FastMapMatch& matcher, LoopsLib::DS::EmbeddedGraph* graph, 
        std::atomic_bool& flagEnd,
        std::atomic_bool& doneFlag):
    trajectories(trajectories),
    matcher(matcher),
        m_interruptFlag(flagEnd),
    m_doneFlag(doneFlag)
    {
        OGRSpatialReference ref;
        ref.SetWellKnownGeogCS("WGS84");
        transform = OGRCreateCoordinateTransformation(&ref, &graph->spatialRef());
    }
    ~MapMatcherJob()
    {
        OGRCoordinateTransformation::DestroyCT(transform);
    }

    void operator()(std::size_t startRange, std::size_t endRange, const std::string& outFile) const
    {
        //Open output file and start writing
        std::ofstream stream(outFile);
        if (!stream.is_open()) throw std::runtime_error("Could not open output file");

        LoopsIO::OutStreamWrapper wrapper(stream);
        for(std::size_t i = startRange; i < endRange; ++i)
        {
            if((int)(i-startRange)%m_checkEveryNIterations == 0 && m_interruptFlag)
            {
                wrapper.write("Interrupted at ").write(i).nl();
                break;
            }

            auto traj = trajectories[i];
            for (std::size_t i = 0; i < traj.timestamps.size(); ++i)
            {
                double x = traj.geom.get_x(i);
                double y = traj.geom.get_y(i);
                transform->Transform(1, &x, &y);
                traj.geom.set_x(i, x);
                traj.geom.set_y(i, y);
            }
            auto res = matcher.match_traj(traj);
            //pybind11::print("MM Trajectory of size: ", res.cpath.size());
            if (!res.cpath.empty())
            {
                wrapper.writeJoined(res.cpath, " ").nl();
            }
        }
        // Flag to calling thread that this thread is done
        m_doneFlag = true;
    }
};

struct LockedOutStream
{
    std::ostream& stream;
    std::mutex m_writeLock;
    LockedOutStream(std::ostream& stream):stream(stream){}
    template<typename T>
    LockedOutStream& operator<<(const T& val)
    {
        std::lock_guard<std::mutex> lockGuard(m_writeLock);
        stream << val;
        return *this;
    }
};

void PyLoops::processing::PyFmm::mapMatchWkbCsvConcurrent(const std::string& sourceFile, const std::string& outFile,
    const std::string& delim, int trajectoryColumn, std::size_t threadNum) const
{

    LoopsIO::WkbCsvTrajectoryReader reader(delim[0], trajectoryColumn);
    reader.setInterruptor(LoopsIO::WkbCsvTrajectoryReader::InterruptHooks{
        []()
        {
            return PyErr_CheckSignals() != 0;
        }, 10
        });
    std::vector<FMM::CORE::Trajectory> trajectories;
    fmm::FmmMMTrajectoryVisitor visitor(trajectories, 60);
    reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);
    //Open output file and start writing
    std::ofstream stream(outFile);
    if (!stream.is_open()) throw std::runtime_error("Could not open output file");

    LoopsIO::OutStreamWrapper wrapper(stream);

    // The threads
    std::vector<std::thread> runningThreads;
    // Interrupt flag
    std::atomic_bool flagEnd = false;
    struct AtomBool
    {
        std::atomic_bool m_b;
        AtomBool(bool value = false): m_b(value){}
        AtomBool(const AtomBool& other): m_b((bool)other.m_b){}
        AtomBool& operator=(const AtomBool& other)
        {
            m_b = (bool)other.m_b;
            return *this;
        }
        std::atomic_bool& get() { return m_b; }
    };
    // Initialize done flags
    std::vector<AtomBool> threadDoneFlags;
    for (std::size_t i = 0; i < threadNum; ++i) threadDoneFlags.emplace_back(false);


    std::size_t elsPerThread = (std::size_t)(trajectories.size() / threadNum);
    std::size_t currStart = 0;
    for (std::size_t i = 0; i < threadNum; ++i)
    {
        std::size_t end = std::min(currStart + elsPerThread, trajectories.size());
        if (i == threadNum - 1) end = trajectories.size();
        std::string file = outFile + "_" + std::to_string(i);
        MapMatcherJob job(trajectories, *m_mapMatch, m_graph, flagEnd, threadDoneFlags[i].get());
        runningThreads.emplace_back(std::move(job),currStart, end, file);
        currStart += elsPerThread;
    }
    while(true)
    {
        // Keep in main thread to be sure
        if(PyErr_CheckSignals() != 0)
        {
            // Flag interrupt to threads
            flagEnd = true;
            // Wait for exit
            for (auto& t : runningThreads) t.join();
            break;
        }
        bool allDone = true;
        // Check if all are done
        for(auto& done: threadDoneFlags)
        {
            if (!done.get()) {
                allDone = false; break;
            }
        }
        if (allDone)
        {
            // Wait for exit
            for (auto& t : runningThreads) t.join();
            break;
        }
        using namespace std::chrono_literals;
        // Sleep for a bit to avoid excessive polling
        std::this_thread::sleep_for(5s);
    }
}

FMM::MM::MatchResult PyLoops::processing::PyFmm::mapMatch(const std::pair<std::vector<std::pair<double, double>>, std::vector<double>>& trajIn) const
{
    FMM::CORE::Trajectory traj;
    traj.timestamps = trajIn.second;
    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    {
        OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&ref, &m_graph->spatialRef());
        std::for_each(trajIn.first.begin(), trajIn.first.end(), [&traj, transform](const std::tuple<LoopsLib::NT, LoopsLib::NT>& latLon)
        {
            double x, y;
            std::tie(y, x) = latLon;
            transform->Transform(1, &x, &y);
            traj.geom.add_point(x, y);
        });
        OGRCoordinateTransformation::DestroyCT(transform);
    }

    auto res = m_mapMatch->match_traj(traj);
    // Transform coordinates back
    {
        OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&m_graph->spatialRef(), &ref);
        auto range = LoopsLib::Helpers::Iterators::ValueIterable<std::size_t>(0, res.mgeom.get_num_points());
        std::for_each(range.begin(), range.end(), [&res,transform](std::size_t ind)
        {
            double x, y;
            std::tie(x, y) = std::make_tuple(res.mgeom.get_x(ind), res.mgeom.get_y(ind));
            transform->Transform(1, &x, &y);
            res.mgeom.set_x(ind, x);
            res.mgeom.set_y(ind, y);
        });
    }
    // Transform IDs
    {
    }
    return res;
}

FMM::MM::MatchResult PyLoops::processing::PyFmm::mapMatchTraj(const FMM::CORE::Trajectory& traj) const
{
    auto res = m_mapMatch->match_traj(traj);
    return res;
}

std::vector<FMM::CORE::Trajectory> PyLoops::processing::PyFmm::readWkbCsvTrajectories(const std::string& sourceFile, const std::string& delim, int trajectoryColumn) const
{

    LoopsIO::WkbCsvTrajectoryReader reader(delim[0], trajectoryColumn);
    reader.setInterruptor(LoopsIO::WkbCsvTrajectoryReader::InterruptHooks{
        []()
        {
            return PyErr_CheckSignals() != 0;
        }, 10
        });
    std::vector<FMM::CORE::Trajectory> trajectories;
    fmm::FmmMMTrajectoryVisitor visitor(trajectories, 60);
    reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);

    return trajectories;
}

void PyLoops::processing::PyFmm::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;

    // Add binding for mapmatch result object
    using MMRes = FMM::MM::MatchResult;
    py::class_<FMM::MM::MatchResult>(mod, "MMResult")
        .def_property_readonly("edgeIndices", [](FMM::MM::MatchResult& res)
        {
            return res.cpath;
        }, py::return_value_policy::reference)
        .def_property_readonly("geometry", [](FMM::MM::MatchResult& res)
        {
            std::vector<std::pair<LoopsLib::NT, LoopsLib::NT>> output;
            output.reserve(res.mgeom.get_num_points());
            auto range = LoopsLib::Helpers::Iterators::ValueIterable<std::size_t>(0, res.mgeom.get_num_points());
            std::transform(range.begin(), range.end(), std::back_inserter(output), [&res](std::size_t ind)
            {
                return std::make_pair(res.mgeom.get_x(ind), res.mgeom.get_y(ind));
            });

            return output;
        });

        pybind11::class_<PyFmm>(mod, "FMapMatcher")
            .def(py::init<LoopsLib::DS::EmbeddedGraph*>())
            .def(py::init<LoopsLib::DS::EmbeddedGraph*, const std::string&>())
            .def("setTable", &PyFmm::setTable)
            .def_property("k", [](PyFmm& mm) { return mm.m_mapMatch->k(); },
                [](PyFmm& mm, int k) { mm.m_mapMatch->setK(k); })
            .def_property("gpsError", [](PyFmm& mm) {return mm.m_mapMatch->gpsError(); },
                [](PyFmm& mm, double error) {mm.m_mapMatch->setGpsError(error); }
            )
            .def_property("candidateRadius", [](PyFmm& mm) {return mm.m_mapMatch->radius(); },
                [](PyFmm& mm, double radius) {mm.m_mapMatch->setRadius(radius); }
            )
            .def("mapMatch", &PyFmm::mapMatch)
            .def("mapMatchTraj", &PyFmm::mapMatch)
            .def("printFirstWkbCsv",&PyFmm::printFirstWkbCsv)
            .def("readWkbCsvTrajectories",&PyFmm::readWkbCsvTrajectories)
            .def("mapMatchWkbCsv",&PyFmm::mapMatchWkbCsv);
}
