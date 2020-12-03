#include <LoopsAlgs/FlowDecomposition/WeightedFrechetPathsDecomposition.h>
#include <LoopsAlgs/Frechet/WeightedStrongFrechet.h>
#include "LoopsLib/Algs/Processing/TrajectorySetFrechet.h"
#include "movetk/metric/Norm.h"
using namespace LoopsLib;

LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::WeightedFrechetPathsDecomposition(
    Models::DecompositionResult* decompObj) : IFlowDecomposition("WeightedFrechetPathDecomposition", decompObj)
{
}

bool LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::noDataCaching() const
{
    return m_noDataCaching;
}

void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::setGreedyAttachEnds(bool value)
{
    m_greedyAttachEnds = value;
}

bool LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::greedyAttachEnds() const
{
    return m_greedyAttachEnds;
}

void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::setMaxSearchTime(double value)
{
    m_maxSearchTimeS = value;
}

double LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::maxSearchTime() const
{
    return m_maxSearchTimeS;
}

void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::setMaximumPerPath(int maxim)
{
    m_maxNum = maxim;
}

int LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::maximumPerPath() const
{
    return m_maxNum;
}

void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::setNumberOfThreads(int maxim)
{
    m_numberOfThreads = maxim;
}

int LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::numberOfThreads() const
{
    return m_numberOfThreads;
}

void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::setNoDataCaching(bool value)
{
    m_noDataCaching = value;
}


void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::findNewBasisElementsParallel()
{
    throw std::runtime_error("Parallel broken");
    //LoopsAlgs::Multithreading::Runner<Basis_t> r;
    //// Set number of threads to use
    //r.m_parallellism = m_numberOfThreads;

    //auto residual = m_decompObj->residual();

    //auto creator = [this, residual]()
    //{
    //    PathComputer computer(m_decompObj->m_relatedInstance->m_epsilon, m_decompObj->paths(), graph(), residual, m_maxNum, m_maxSearchTimeS);
    //    if (!m_logDir.empty())
    //    {
    //        if (m_logPrefix.empty())
    //        {
    //            m_logPrefix = "weightedfrechet";
    //        }
    //        std::string logPath = m_logDir + '/' + m_logPrefix;
    //        computer.setLogBase(logPath);
    //    }
    //    return std::move(computer);
    //};
    //auto progress = [](int thread, int progress)
    //{
    //    auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition>();
    //    logger.info("Progress thread ", thread, ": ", progress, "/100");
    //};
    //auto resultCb = [this](const Basis_t& basis) {this->extendBasis(basis); };

    //r.setProgressCallback(progress);
    //r.run(creator, m_decompObj->m_relatedInstance->m_availablePaths, resultCb);
}


void toMovetkTrajectory(const LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::BasisElement_t& trajectory,
    DS::EmbeddedGraph* graph,
    std::vector<MovetkGeometryKernel::MovetkPoint>& movetkTrajectory)
{
    // Convert to Movetk trajectory
    std::transform(trajectory.begin(), trajectory.end(), std::back_inserter(movetkTrajectory),
        [graph](const auto& eId)
    {
        return graph->locations()[graph->edge(eId)->m_source->id()];
    });
    movetkTrajectory.push_back(graph->locations()[graph->edge(trajectory.back())->m_sink->id()]);
}

void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::findNewBasisElements()
{
    if(m_numberOfThreads > 1)
    {
        findNewBasisElementsParallel();
        return;
    }
    std::unique_ptr<std::ostream> warnStream;
    std::unique_ptr<std::ostream> logStream;
    std::unique_ptr<std::ostream> errStream;
    std::unique_ptr<std::ostream> deeplogStream;
    if(!m_logDir.empty())
    {
        if(m_logFilePrefix.empty())
        {
            m_logFilePrefix = "weightedfrechetpaths";
        }
        std::string logPath = m_logDir + '/' + m_logFilePrefix;
        warnStream.reset(new std::ofstream(logPath + ".warn"));
        errStream.reset(new std::ofstream(logPath + ".err"));
        logStream.reset(new std::ofstream(logPath + ".log"));
        deeplogStream.reset(new std::ofstream(logPath + ".deeplog"));
        // Make the weighted Frechet log to files
        auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::WeightedStrongFrechet>();

        using namespace LoopsLib::Helpers;
        // Reset the streams to only use std::cout
        logger.resetLevelsOutput<LogLevel::Info,LogLevel::Warn, LogLevel::Error, LogLevel::Trace, LogLevel::Deep, LogLevel::Debug>();
        // Add the streams 
        logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Info>({ deeplogStream.get(),logStream.get() });
        logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Warn>({ deeplogStream.get(),warnStream.get(),logStream.get() });
        logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Error>({ deeplogStream.get(),errStream.get(),warnStream.get(),logStream.get() });
        // If enabled, write trace, deep and debug to the extensive log file.
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Trace>(deeplogStream.get());
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Deep>(deeplogStream.get());
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Debug>(deeplogStream.get());
    }

    auto logger = LoopsLib::Helpers::logFactory(this);
    const auto epsilon = m_decompObj->m_relatedInstance->m_epsilon;
    if(!m_noDataCaching)
    {
        if(m_graphDatas.size() != numberOfRepresentatives())
        {
            m_graphDatas.resize(numberOfRepresentatives());
            auto it = m_decompObj->m_relatedInstance->m_representatives.begin();
            // Precompute all frechet data objects
            for (std::size_t i = 0; i < m_graphDatas.size(); ++i)
            {
                // Set prefix for logging, if enabled
                auto prefixReset = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::WeightedStrongFrechet>().addGlobalPrefix(this->m_logPrefix+"[Path " + std::to_string(i) + "]");
                try
                {
                    Frechet::WeightedStrongFrechet weightedFrechet(graph());
                    weightedFrechet.setGreedyAttachEnd(m_greedyAttachEnds);
                    weightedFrechet.setMaxSearchTimeS(m_maxSearchTimeS);
                    weightedFrechet.precompute(*it, epsilon, m_graphDatas[i]);
                }
                catch(std::exception& e)
                {
                    logger.error("Caught exception ", e.what());
                }
                ++it;
            }
        }
        
        

        std::vector<NT> currentField = m_decompObj->residual();
        // For every known path, find extra paths
        for (std::size_t pId = 0; pId < m_graphDatas.size(); ++pId)
        {
            Basis_t extra;
            Frechet::WeightedStrongFrechet weightedFrechet(graph());
            weightedFrechet.setGreedyAttachEnd(m_greedyAttachEnds);
            weightedFrechet.setMaxSearchTimeS(m_maxSearchTimeS);
            weightedFrechet.compute(m_graphDatas[pId], currentField, extra);
            this->extendBasis(extra);
        }
    }
    else
    {
        const std::vector<NT> currentField = m_decompObj->residual();
        logger.info("Running uncached for ", numberOfRepresentatives(), " paths");
        // For every known path, find extra paths
        std::size_t i = 0;
        for (auto reprInd = 0; reprInd < numberOfRepresentatives(); ++reprInd)
        {
            const auto& repr = representative(reprInd);
            // Set prefix for logging, if enabled
            auto prefixReset = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::WeightedStrongFrechet>().addGlobalPrefix("[" + this->m_logPrefix + "|Path " + std::to_string(i) + "]");
            Basis_t extra;
            try
            {
                Frechet::WeightedStrongFrechet weightedFrechet(graph());
                weightedFrechet.setGreedyAttachEnd(m_greedyAttachEnds);
                weightedFrechet.setMaxSearchTimeS(m_maxSearchTimeS);
                weightedFrechet.setMaximumPerPath(m_maxNum);
                weightedFrechet.compute(repr, epsilon, currentField, extra);
                this->extendBasis(extra);
                // Verify Frechet
                for(const auto &el: extra)
                {

                    using SFD_tested = movetk_algorithms::StrongFrechet<MovetkGeometryKernel, movetk_support::squared_distance_d<MovetkGeometryKernel, movetk_support::FiniteNorm<MovetkGeometryKernel, 2>>>;
                    SFD_tested sfd;
                    sfd.setUpperbound(epsilon);
                    using FrMode = decltype(sfd.mode());
                    sfd.setMode(FrMode::BisectionSearch);
                    sfd.setTolerance(0.5);

                    std::vector<MovetkGeometryKernel::MovetkPoint> basisTraj;
                    toMovetkTrajectory(el, graph(), basisTraj);

                    NT actualEps;
                    bool success = sfd(repr.begin(), repr.end(), basisTraj.cbegin(), basisTraj.cend(), actualEps);
                    if(!success)
                    {
                        std::cout << "Generated element with too large epsilon" << std::endl;
                    }
                    else
                    {
                        std::cout << "Awesome element, actual epsilon: " << actualEps << ", allowed = " << epsilon << std::endl;
                    }
                }
            }
            catch(std::exception& e)
            {
                logger.error("Caught exception ", e.what(), " for path ", decompositionObject()->m_relatedInstance->representativeDescription(reprInd), " of size ", 
                    repr.size());
            }
            ++i;
        }
    }
    if (!m_logDir.empty())
    {
        using namespace LoopsLib::Helpers;
        // Reset the streams to only use std::cout
        logger.resetLevelsOutput<LogLevel::Info, LogLevel::Warn, LogLevel::Error, LogLevel::Trace, LogLevel::Deep, LogLevel::Debug>();
    }
}

void LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition::reset()
{
    IFlowDecomposition::reset();
    // Truly clear the memory
    m_graphDatas.clear();
    m_graphDatas.shrink_to_fit();
}
