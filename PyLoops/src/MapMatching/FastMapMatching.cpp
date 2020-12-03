#include <PyLoops/MapMatching/FastMapMatching.h>
#include "LoopsAlgs/MapMatching/FastMapMatching.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h>
#include <fstream>
#include <thread>


struct FMMJobResult
{
    std::string error;
};

struct FastMapMatcherJob
{
    // Resides on parent thread, so read only! 
    //const std::vector<FMM::CORE::Trajectory>& trajectories;
    LoopsAlgs::MapMatching::FastMapMatching matcher;
    // End flag
    std::atomic_bool* m_interruptFlag;
    std::atomic_bool* m_doneFlag;

    std::atomic_int* m_progress;

    // Keep a const ref
    const LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& trajectories;

    FMMJobResult* m_result;

    int m_checkEveryNIterations = 10;

    bool m_debugMode = false;

    FastMapMatcherJob(const LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& trajectories, const LoopsAlgs::MapMatching::FastMapMatching& matcher, 
        std::atomic_bool* flagEnd,
        std::atomic_bool* doneFlag,
        std::atomic_int* progress,
        FMMJobResult* jobResult) :
        trajectories(trajectories),
        matcher(matcher),
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
        catch (std::exception& e)
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
        while (std::getline(logStream, line))
        {
            auto sepPos = line.find(':');
            if (sepPos == std::string::npos) continue;
            auto trajPos = line.find("trajectory", sepPos);
            if (trajPos == std::string::npos) continue;
            auto nextSep = line.find(',', trajPos);
            if (nextSep == std::string::npos) continue;
            std::string id = line.substr(trajPos + trajString.size(), nextSep - trajPos + trajString.size());
            if (line.find("size", nextSep) != std::string::npos)
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
        if (hasIdsInPreviousLog)
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

        std::ofstream failStream(outFile + ".log", outMode);
        if (!failStream.is_open()) {
            m_result->error = "Could not open log file";
            *m_doneFlag = true;
            return;
        }
        std::shared_ptr<std::ofstream> debugStream;
        //if (m_debugMode)
        //{
        //    // Make shared ptr to managed scoped deallocation
        //    debugStream = std::make_shared<std::ofstream>(outFile + ".debug");
        //    *debugStream << std::this_thread::get_id() << ":Starting debug writing " << std::endl;
        //    matcher->setLogger(debugStream.get());
        //}
        if (hasIdsInPreviousLog)
        {
            failStream << std::this_thread::get_id() << ':' << "Resuming from trajectory num " << start << "\n";
        }
        {
            failStream << std::this_thread::get_id() << ':' << "Starting thread " << std::this_thread::get_id() << " on range " << startRange << ","
                << endRange << "\n";
        }
        for (std::size_t i = start; i < endRange; ++i)
        {
            if (m_debugMode)
            {
                *debugStream << std::this_thread::get_id() << ":Trajectory " << i << "/" << trajectories.size() << std::endl;
            }
            if ((int)(i - startRange) % m_checkEveryNIterations == 0)
            {
                m_progress->store(i - startRange, std::memory_order_relaxed);
                failStream << std::this_thread::get_id() << ": progress " << i - startRange << "/"<< (endRange - startRange) << std::endl;
                if (m_interruptFlag->load(std::memory_order_relaxed))
                {
                    stream << "Interrupted at " << i << '\n';
                    break;
                }
            }
            failStream << std::this_thread::get_id() << ": trajectory " << i << ", size: " << trajectories.trajectories.at(i).size() << std::endl;

            

            LoopsLib::MovetkGeometryKernel::GraphTrajectory out;
            // Maybe convert
            matcher.mapMatch(trajectories.trajectories.at(i), trajectories.m_ref, out);

            if (!out.empty())
            {
                std::ostream_iterator<decltype(out[0])> writer(stream, " ");
                std::copy(out.begin(), out.end(), writer);
                stream << std::endl;
            }
            else
            {
                failStream << std::this_thread::get_id() << ": trajectory " << i << ", failed: " << std::endl;
            }
        }
        // Flag to calling thread that this thread is done
        m_doneFlag->store(true, std::memory_order_relaxed);
    }
};


void PyLoops::MapMatching::FastMapMatching::multithreadedMapmatchSet(
    const LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& input,
    const LoopsAlgs::MapMatching::FastMapMatching& mapMatcher,
    int threadNum,
    const std::string& outFile)
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


    std::size_t elsPerThread = (std::size_t)(input.size() / threadNum);
    std::size_t currStart = 0;
    for (std::size_t i = 0; i < threadNum; ++i)
    {
        std::size_t end = std::min(currStart + elsPerThread, input.size());
        if (i == threadNum - 1) end = input.size();
        std::string file = outFile + "_" + std::to_string(i);
        FastMapMatcherJob job(input, mapMatcher, &flagEnd, &threadDoneFlags[i], &progressPerThread[i], &jobResults[i]);
        job.m_debugMode = false;
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

void PyLoops::MapMatching::FastMapMatching::registerPy(pybind11::module& mod)
{
    using FMM = LoopsAlgs::MapMatching::FastMapMatching;
    using Kernel = LoopsLib::MovetkGeometryKernel;

    namespace py = pybind11;
    py::class_<FMM>(mod, "FastMapMatching")
        .def(py::init<LoopsLib::DS::EmbeddedGraph*>())
        // Graph, searchRadius, gpsError, k
        .def(py::init<LoopsLib::DS::EmbeddedGraph*, LoopsLib::NT, LoopsLib::NT, int>())
        .def("mapMatch", [](FMM& fmm, const Kernel::TimestampedTrajectory& trajectory )
        {
            pybind11::scoped_ostream_redirect streamRedir(
                std::cout,                               // std::ostream&
                pybind11::module::import("sys").attr("stdout") // Python output
            );
            Kernel::GraphTrajectory out;
            fmm.mapMatch(trajectory, out);
            return out;
        })
        .def("mapMatchSet", [](FMM& fmm, const Kernel::TimestampedTrajectorySet& trajectory, int count)
        {
            pybind11::scoped_ostream_redirect streamRedir(
                std::cout,                               // std::ostream&
                pybind11::module::import("sys").attr("stdout") // Python output
            );
            Kernel::GraphTrajectorySet out;
            fmm.mapMatchSet(trajectory, out, count);
            return out;
        })
        .def("mapMatchFromWGS84", [](FMM& fmm, const Kernel::TimestampedTrajectory& trajectory)
        {
            pybind11::scoped_ostream_redirect streamRedir(
                std::cout,                               // std::ostream&
                pybind11::module::import("sys").attr("stdout") // Python output
            );
            Kernel::GraphTrajectory out;
            int matchOffset, matchCount;
            fmm.mapMatchFromWGS84(trajectory, out, matchOffset,matchCount);
            return std::make_tuple(out, matchOffset, matchCount);
        })
        .def("mapMatchSetWithStats", [](FMM& fmm, const Kernel::TimestampedTrajectorySet& trajectory, int count)
        {
            pybind11::scoped_ostream_redirect streamRedir(
                std::cout,                               // std::ostream&
                pybind11::module::import("sys").attr("stdout") // Python output
            );
            Kernel::GraphTrajectorySet out;
            FMM::SetMatchResultDescription description;
            fmm.mapMatchSet(trajectory, out, description, count);
            return std::make_tuple(out, description.matchedTrajectories,description.matchOffset, description.matchSize, description.inputSize);
        })
        .def("mapMatchSetConcurrent", [](FMM& fmm, const Kernel::TimestampedTrajectorySet& trajectory, int threadNum, const std::string& outputFilePath)
        {
            FastMapMatching fm;
            fm.multithreadedMapmatchSet(trajectory, fmm, threadNum, outputFilePath);
            py::print("Done mapmatching");
        })
        .def_property("k", &FMM::k, &FMM::setK)
        .def_property("candidateRadius", &FMM::radius, &FMM::setRadius)
        .def_property("gpsError", &FMM::gpsError, &FMM::setGpsError)
        .def_property("speedBound", &FMM::speedBound, &FMM::setSpeedBound)
        .def_property("verbose", &FMM::verbose, &FMM::setVerbose)
        ;
    //.def_property("")
}
