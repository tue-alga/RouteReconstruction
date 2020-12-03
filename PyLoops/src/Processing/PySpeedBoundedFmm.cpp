#include <PyLoops/Processing/PySpeedBoundedFmm.h>
#include <pybind11/stl.h>
#include "LoopsIO/WkbCsvTrajectoryReader.h"
#include "LoopsIO/Helpers/OutStreamWrapper.h"
#include "IO/FmmMMTrajectoryVisitor.h"
#include <thread>
#include <pybind11/iostream.h>
#include <chrono>
#include <mutex>
#include <utility>
//#include <filesystem>
#include <chrono>
#include <ctime>    
#include <PyLoops/Bindings/ds/PyTrajectory.h>

void PyLoops::processing::PySpeedBoundedFmm::readWkbCsvTrajectories(const std::string& sourceFile,
    const CsvConfig& config, std::vector<FMM::CORE::Trajectory>& output) const
{
    LoopsIO::WkbCsvTrajectoryReader reader(config.delim, config.trajectoryColumn);
    reader.setInterruptor(LoopsIO::WkbCsvTrajectoryReader::InterruptHooks{
        []()
        {
            // Check for signals, in particular keyboard interrupts
          if (PyErr_CheckSignals() != 0) return true;
          return false;
        }, 10
        });
    fmm::FmmMMTrajectoryVisitor visitor(output, 60);

    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&ref, &m_graph->spatialRef());
    visitor.setTransform(transform);

    // Read the trajectories
    reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);

    // Destroy the transform
    OGRCoordinateTransformation::DestroyCT(transform);
}

void PyLoops::processing::PySpeedBoundedFmm::readWktCsvTrajectories(const std::string& sourceFile,
    const CsvConfig& config, std::vector<FMM::CORE::Trajectory>& output) const
{
    LoopsIO::WktCsvTrajectoryReader reader(config.delim, config.trajectoryColumn);
    reader.setInterruptor(LoopsIO::WktCsvTrajectoryReader::InterruptHooks{
        []()
        {
          if (PyErr_CheckSignals() != 0) return true;
          return false;
        }, 10
        });
    fmm::FmmMMTrajectoryVisitor visitor(output, 60);

    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&ref, &m_graph->spatialRef());
    visitor.setTransform(transform);

    // Read the trajectories
    reader.read<fmm::FmmMMTrajectoryVisitor>(sourceFile, visitor, true);

    // Destroy the transform
    OGRCoordinateTransformation::DestroyCT(transform);
}

PyLoops::processing::PySpeedBoundedFmm::PySpeedBoundedFmm(LoopsLib::DS::EmbeddedGraph* graph) : m_graph(graph)
{
    m_mapMatch = std::make_shared<FMM::MM::SpeedBoundedFastMapMatch>(m_graph);
}



void PyLoops::processing::PySpeedBoundedFmm::printFirstWkbCsv(const std::string& sourceFile) const
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

void PyLoops::processing::PySpeedBoundedFmm::mapMatchWkbCsv(const std::string& sourceFile, const std::string& outFile, const std::string& delim, int trajectoryColumn) const
{
    pybind11::scoped_ostream_redirect streamRedir(
        std::cout,                               // std::ostream&
        pybind11::module::import("sys").attr("stdout") // Python output
    );

    CsvConfig config;
    config.delim = delim[0];
    config.trajectoryColumn = trajectoryColumn;
    std::vector<FMM::CORE::Trajectory> trajectories;
    readWkbCsvTrajectories(sourceFile, config, trajectories);
    std::cout << "Read trajectories: " << trajectories.size() << std::endl;
    //Open output file and start writing
    std::ofstream stream(outFile);
    if (!stream.is_open()) throw std::runtime_error("Could not open output file");

    LoopsIO::Helpers::OutStreamWrapper wrapper(stream);

    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&ref, &m_graph->spatialRef());

    std::size_t trajI = 0;
    for (auto& traj : trajectories)
    {

        for (std::size_t i = 0; i < traj.timestamps.size(); ++i)
        {
            double x = traj.geom.get_x(i);
            double y = traj.geom.get_y(i);
            transform->Transform(1, &x, &y);
            traj.geom.set_x(i, x);
            traj.geom.set_y(i, y);
        }
        std::string reason;
        auto res = m_mapMatch->match_traj(traj,reason);
        //pybind11::print("MM Trajectory of size: ", res.cpath.size());
        if (!res.cpath.empty())
        {
            wrapper.writeJoined(res.cpath, " ").nl();
        }
        else
        {
            std::cout << "Traj " << trajI << " failed:" << reason << std::endl;
        }
        ++trajI;
    }
}


struct LockedOutStream
{
    std::ostream& stream;
    std::mutex m_writeLock;
    LockedOutStream(std::ostream& stream) :stream(stream) {}


    template<typename...Args>
    void write(Args... args)
    {
        std::lock_guard<std::mutex> lockGuard(m_writeLock);
        using expand = int[];
        (void) expand{(stream << args, 0)...};
    }
    template<typename...Args>
    void writeLn(Args... args)
    {
        std::lock_guard<std::mutex> lockGuard(m_writeLock);
        using expand = int[];
        (void)expand {
            (stream << args, 0)...
        };
        stream << '\n';
    }
    template<typename...Args>
    void writeEndl(Args... args)
    {
        std::lock_guard<std::mutex> lockGuard(m_writeLock);
        using expand = int[];
        (void)expand {
            (stream << args, 0)...
        };
        stream << '\n';
        stream.flush();
    }
    void flush()
    {
        std::lock_guard<std::mutex> lockGuard(m_writeLock);
        stream.flush();
    }

};

struct FMMJobResult
{
    std::string error;
};

struct FastMapMatcherJob
{
    // Resides on parent thread, so read only! 
    const std::vector<FMM::CORE::Trajectory>& trajectories;
    std::shared_ptr<FMM::MM::SpeedBoundedFastMapMatch> matcher;
    // End flag
    std::atomic_bool* m_interruptFlag;
    std::atomic_bool* m_doneFlag;

    std::atomic_int* m_progress;

    FMMJobResult* m_result;

    int m_checkEveryNIterations = 10;

    bool m_debugMode = false;

    FastMapMatcherJob(const std::vector<FMM::CORE::Trajectory>& trajectories, std::shared_ptr<FMM::MM::SpeedBoundedFastMapMatch> matcher, LoopsLib::DS::EmbeddedGraph* graph,
        std::atomic_bool* flagEnd,
        std::atomic_bool* doneFlag,
        std::atomic_int* progress,
        FMMJobResult* jobResult) :
        trajectories(trajectories),
        matcher(std::make_shared< FMM::MM::SpeedBoundedFastMapMatch>(*matcher.get())),
        m_interruptFlag(flagEnd),
        m_doneFlag(doneFlag),
        m_progress(progress),
        m_result(jobResult)
    {
    }

    void operator()(std::size_t startRange, std::size_t endRange, const std::string& outFile)
    {
        try
        {
            run(startRange, endRange, outFile);
        }
        catch(std::exception& e)
        {
            m_result->error = e.what();
            *m_doneFlag = true;
        }
    }

    bool checkPreviousLog(std::size_t& startRange, const std::string& logFile)
    {
        std::ifstream logStream(logFile);
        if (!logStream.is_open()) return false;
        std::string line;
        const std::string trajString = "trajectory";
        bool foundId = false;
        while(std::getline(logStream, line))
        {
            auto sepPos = line.find(':');
            if (sepPos == std::string::npos) continue;
            auto trajPos = line.find("trajectory", sepPos);
            if (trajPos== std::string::npos) continue;
            auto nextSep = line.find(',', trajPos);
            if (nextSep == std::string::npos) continue;
            std::string id = line.substr(trajPos + trajString.size(), nextSep - trajPos + trajString.size());
            if(line.find("size", nextSep) != std::string::npos)
            {
                foundId = true;
                startRange = std::stoull(id);
            }
        }
        return foundId;
    }

    void run(std::size_t startRange, std::size_t endRange, const std::string& outFile) 
    {
        
        std::ios::openmode outMode = std::ios::out;

        // Previous log exists and gives a starting point: adjust start range
        // and open log in append mode
        std::size_t start = startRange; // May be modified
        bool hasIdsInPreviousLog = checkPreviousLog(start, outFile + ".log");
        if(hasIdsInPreviousLog)
        {
            outMode |= std::ios::app;
        }

        //Open output file and start writing
        // If the log exists, the outfile should also exist!!!!!
        std::ofstream stream(outFile, outMode);
        if (!stream.is_open()) {
            m_result->error = "Could not open output file";
            *m_doneFlag = true;
            return;
        }

        std::ofstream failStream(outFile+".log", outMode);
        if (!failStream.is_open()) {
            m_result->error = "Could not open log file";
            *m_doneFlag = true;
            return;
        }
        std::shared_ptr<std::ofstream> debugStream;
        if(m_debugMode)
        {
            // Make shared ptr to managed scoped deallocation
            debugStream = std::make_shared<std::ofstream>(outFile + ".debug");
            *debugStream << std::this_thread::get_id() <<":Starting debug writing " << std::endl;
            matcher->setLogger(debugStream.get());
        }
        LoopsIO::Helpers::OutStreamWrapper wrapper(stream);
        LoopsIO::Helpers::OutStreamWrapper failWrapper(failStream);
        if(hasIdsInPreviousLog)
        {
            failWrapper.write(std::this_thread::get_id()).write(':').write("Resuming from trajectory num ").write(start).nl();
        }

        {
            failWrapper.write(std::this_thread::get_id()).write(':').write("Starting thread ").write(std::this_thread::get_id()).write(" on range ").write(startRange).write(",").write(endRange).nl();
        }
        for (std::size_t i = start; i < endRange; ++i)
        {
            if(m_debugMode)
            {
                *debugStream << std::this_thread::get_id() << ":Trajectory " << i << "/" << trajectories.size() << std::endl;
            }
            if ((int)(i - startRange) % m_checkEveryNIterations == 0)
            {
                m_progress->store(i - startRange, std::memory_order_relaxed);
                failWrapper.write(std::this_thread::get_id()).write(": progress ").write(i - startRange).write("/").write(endRange - startRange).nl();
                failWrapper.flush();
                if(m_interruptFlag->load(std::memory_order_relaxed))
                {
                    wrapper.write("Interrupted at ").write(i).nl().flush();
                    break;
                }
            }
            failWrapper.write(std::this_thread::get_id()).write(": trajectory ").write(i).write(", size: ").write(trajectories.at(i).timestamps.size()).nl().flush();

            std::string reason;
            auto res = matcher->match_traj(trajectories.at(i),reason);
            //pybind11::print("MM Trajectory of size: ", res.cpath.size());
            if (!res.cpath.empty())
            {
                wrapper.writeJoined(res.cpath, " ").nl().flush();
            }
            else
            {
                failWrapper.write(std::this_thread::get_id()).write(": trajectory ").write(i).write(", failed: ").write(reason).nl().flush();
            }
        }
        // Flag to calling thread that this thread is done
        m_doneFlag->store(true,std::memory_order_relaxed);
    }
};

void PyLoops::processing::PySpeedBoundedFmm::mapMatchWkbCsvConcurrent(const std::string& sourceFile, const std::string& outFile,
    const std::string& delim, int trajectoryColumn, std::size_t threadNum, bool debugMode) const
{
    pybind11::scoped_ostream_redirect streamRedir(
        std::cout,                               // std::ostream&
        pybind11::module::import("sys").attr("stdout") // Python output
    );
    if(!m_graph)
    {
        throw std::runtime_error("No graph set on speebounded mapmatcher");
    }

    std::cout << "Loading trajectories" << std::endl;
    CsvConfig config;
    config.delim = delim[0];
    config.trajectoryColumn = trajectoryColumn;
    std::vector<FMM::CORE::Trajectory> trajectories;
    readWkbCsvTrajectories(sourceFile, config, trajectories);

    mapMatchConcurrent(trajectories, outFile, threadNum, debugMode);
}
void PyLoops::processing::PySpeedBoundedFmm::mapMatchWktCsvConcurrent(const std::string& sourceFile, const std::string& outFile,
    const std::string& delim, int trajectoryColumn, std::size_t threadNum, bool debugMode) const
{
    pybind11::scoped_ostream_redirect streamRedir(
        std::cout,                               // std::ostream&
        pybind11::module::import("sys").attr("stdout") // Python output
    );
    if (!m_graph)
    {
        throw std::runtime_error("No graph set on speebounded mapmatcher");
    }

    std::cout << "Loading trajectories" << std::endl;
    CsvConfig config;
    config.delim = delim[0];
    config.trajectoryColumn = trajectoryColumn;
    std::vector<FMM::CORE::Trajectory> trajectories;
    readWktCsvTrajectories(sourceFile, config, trajectories);

    mapMatchConcurrent(trajectories, outFile, threadNum, debugMode);
}

void PyLoops::processing::PySpeedBoundedFmm::splitWkbCsvTrajectoriesMeb(const std::string& sourceFile,
    const std::string& outFile, const CsvConfig& csvParameters)
{
    std::vector<FMM::CORE::Trajectory> trajectories;
    readWktCsvTrajectories(sourceFile, csvParameters, trajectories);
    //movetk_core::
}

void PyLoops::processing::PySpeedBoundedFmm::mapMatchTrajectoryListConcurrent(const ds::TrajectoryList& trajectories,
    const std::string& outFile, std::size_t threadNum, bool debugMode) const
{
    mapMatchConcurrent(trajectories.trajectories(), outFile, threadNum, debugMode);
}

void PyLoops::processing::PySpeedBoundedFmm::mapMatchConcurrent(const std::vector<FMM::CORE::Trajectory>& trajectories,
    const std::string& outFile, std::size_t threadNum, bool debugMode) const
{
    pybind11::scoped_ostream_redirect streamRedir(
        std::cout,                               // std::ostream&
        pybind11::module::import("sys").attr("stdout") // Python output
    );
    std::cout << "Setting up" << std::endl;

    // The threads
    std::vector<std::thread> runningThreads;
    // Interrupt flag
    std::atomic_bool flagEnd = false;

    // Initialize done flags
    std::atomic_bool* threadDoneFlags = new std::atomic_bool[threadNum];
    for (int i = 0; i < threadNum; ++i) threadDoneFlags[i] = false;

    std::atomic_int* progressPerThread = new std::atomic_int[threadNum];
    for (int i = 0; i < threadNum; ++i) progressPerThread[i] = 0;

    pybind11::print("Map matching with ", threadNum, " threads");

    std::vector<int> elsPerThreadList;
    std::vector<int> lastProgress;

    // Don't touch until thread is done
    std::vector<FMMJobResult> jobResults;
    jobResults.resize(threadNum, {});


    std::size_t elsPerThread = (std::size_t)(trajectories.size() / threadNum);
    std::size_t currStart = 0;
    for (std::size_t i = 0; i < threadNum; ++i)
    {
        std::size_t end = std::min(currStart + elsPerThread, trajectories.size());
        if (i == threadNum - 1) end = trajectories.size();
        std::string file = outFile + "_" + std::to_string(i);
        FastMapMatcherJob job(trajectories, m_mapMatch, m_graph, &flagEnd, &threadDoneFlags[i], &progressPerThread[i], &jobResults[i]);
        job.m_debugMode = debugMode;
        runningThreads.emplace_back(std::move(job), currStart, end, file);
        elsPerThreadList.push_back(end - currStart);
        lastProgress.push_back(0);
        currStart += elsPerThread;
    }
    std::cout << "Jobs started" << std::endl;;
    int progressReportCount = 1;
    int progressReportWrap = 3;

    auto startTime = std::chrono::system_clock::now();
    while (true)
    {
        //std::cout << "Checking signals" << std::endl;;
        // Keep in main thread to be sure
        if (PyErr_CheckSignals() != 0)
        {
            // Flag interrupt to threads
            flagEnd.store(true, std::memory_order_relaxed);
            // Wait for exit
            std::cout << "Interrupt: joining threads" << std::endl;;
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2s); // Sleep so as to flush the above message hopefully.
            for (auto& t : runningThreads) t.join();
            std::cout << "Interrupted done" << std::endl;
            break;
        }
        if (progressReportCount == 0)
        {
            bool allDone = true;

            // Some computation here
            auto curr = std::chrono::system_clock::now();
            auto end_time = std::chrono::system_clock::to_time_t(curr);
            std::chrono::duration<long long> timeSpent = std::chrono::duration_cast<std::chrono::seconds>(
                curr - startTime);
            // Check if all are done
            std::cout << "Checking thread status at " << std::ctime(&end_time) << std::endl;;
            for (int i = 0; i < threadNum; ++i)
            {
                if (!threadDoneFlags[i].load(std::memory_order_relaxed)) {
                    allDone = false; break;
                }
            }
            if (allDone)
            {
                std::cout << "Computations done" << std::endl;;
                // Wait for exit
                for (auto& t : runningThreads) t.join();
                break;
            }
            for (int i = 0; i < threadNum; ++i)
            {
                const int progress = progressPerThread[i].load(std::memory_order_relaxed);
                std::cout << "Thread " << i << ": " << progress << "/" << elsPerThreadList[i] << ": estimated " << elsPerThreadList[i] / (progress / (double)timeSpent.count()) << "s remaining" << std::endl;
                if ((bool)threadDoneFlags[i] && !jobResults[i].error.empty()) std::cout << "--- Error: " << jobResults[i].error << std::endl;
            }
        }
        progressReportCount = (progressReportCount + 1) % progressReportWrap;

        using namespace std::chrono_literals;
        // Sleep for a bit to avoid excessive polling
        //std::cout << "Sleeping" << std::endl;;
        std::this_thread::sleep_for(5s);
    }
    delete[] threadDoneFlags;
    delete[] progressPerThread;
}

FMM::MM::MatchResult PyLoops::processing::PySpeedBoundedFmm::mapMatch(const std::pair<std::vector<std::pair<double, double>>, std::vector<double>>& trajIn) const
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
    std::string reason;
    auto res = m_mapMatch->match_traj(traj, reason);
    // Transform coordinates back
    {
        OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&m_graph->spatialRef(), &ref);
        auto range = LoopsLib::Helpers::Iterators::ValueIterable<std::size_t>(0, res.mgeom.get_num_points());
        std::for_each(range.begin(), range.end(), [&res, transform](std::size_t ind)
        {
            double x, y;
            std::tie(x, y) = std::make_tuple(res.mgeom.get_x(ind), res.mgeom.get_y(ind));
            transform->Transform(1, &x, &y);
            res.mgeom.set_x(ind, x);
            res.mgeom.set_y(ind, y);
        });
    }
    if(!reason.empty())
    {
        std::cout << "Matching failed:" << reason << std::endl;
    }
    // Transform IDs
    {
    }
    return res;
}

void PyLoops::processing::PySpeedBoundedFmm::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;
    
    pybind11::class_<PySpeedBoundedFmm>(mod, "SpeedBoundedFMapMatcher")
        .def(py::init<LoopsLib::DS::EmbeddedGraph*>())
        .def_property("k", [](PySpeedBoundedFmm& mm) { return mm.m_mapMatch->k(); },
            [](PySpeedBoundedFmm& mm, int k) { mm.m_mapMatch->setK(k); })
        .def_property("gpsError", [](PySpeedBoundedFmm& mm) {return mm.m_mapMatch->gpsError(); },
            [](PySpeedBoundedFmm& mm, double error) {mm.m_mapMatch->setGpsError(error); }
        )
        .def_property("candidateRadius",
            [](PySpeedBoundedFmm& mm) {return mm.m_mapMatch->radius(); },
            [](PySpeedBoundedFmm& mm, double radius) {mm.m_mapMatch->setRadius(radius); }
        )
        .def_property("speedBound",
            [](PySpeedBoundedFmm& mm) {return mm.m_mapMatch->speedBound(); },
            [](PySpeedBoundedFmm& mm, double speedBound) {mm.m_mapMatch->setSpeedBound(speedBound); }
        )
        .def("mapMatch", &PySpeedBoundedFmm::mapMatch)
        .def("printFirstWkbCsv", &PySpeedBoundedFmm::printFirstWkbCsv)
        .def("mapMatchWkbCsv", &PySpeedBoundedFmm::mapMatchWkbCsv)
        .def("mapMatchWkbCsvConcurrent", &PySpeedBoundedFmm::mapMatchWkbCsvConcurrent)
        .def("mapMatchWktCsvConcurrent", &PySpeedBoundedFmm::mapMatchWktCsvConcurrent)
        .def("mapMatchTrajectoryListConcurrent", &PySpeedBoundedFmm::mapMatchTrajectoryListConcurrent);
}
