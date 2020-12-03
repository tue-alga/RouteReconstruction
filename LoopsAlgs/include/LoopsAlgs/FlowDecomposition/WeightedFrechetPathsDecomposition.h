#ifndef ALGS_FLOWDECOMPOSITION_WEIGHTEDFRECHETPATHSDECOMPOSITION_H
#define ALGS_FLOWDECOMPOSITION_WEIGHTEDFRECHETPATHSDECOMPOSITION_H
#include <vector>
#include <LoopsAlgs/IFlowDecomposition.h>
#include <LoopsAlgs/Frechet/WeightedStrongFrechet.h>
#include <LoopsAlgs/Frechet/FrechetHelpers.h>
#include <LoopsLib/Helpers/Logger.h>
#include <LoopsAlgs/Multithreading/Helpers.h>
namespace LoopsAlgs::FlowDecomposition
{
    class WeightedFrechetPathsDecomposition;
}

namespace LoopsLib::Helpers
{
    template<>
    constexpr int logLevel<LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition>()
    {
        return LoopsLib::Helpers::LogLevel::Warn;
    }
}

namespace LoopsAlgs::FlowDecomposition
{
    /**
     * \brief Algorithm for a simple approach to path decomposition.
     * The algorithm decomposes the flow in cycles and simple paths.
     * If the given field is not an actual flow,
     * \tparam Graph
     * \tparam FlowField
     */
    class WeightedFrechetPathsDecomposition : public IFlowDecomposition
    {
        // Parameters

        // Don't cache data. Better for larger datasets due to memory usage...
        bool m_noDataCaching = false;
        // Maximum search time for weighted path
        double m_maxSearchTimeS = 30;
        // Maximum number of paths per search
        int m_maxNum = 1;
        //
        int m_numberOfThreads = 1;

        std::vector<Frechet::StrongFrechetGraphData> m_graphDatas;

        bool m_greedyAttachEnds = false;


        //class PathComputer : public LoopsAlgs::Multithreading::Task<
        //    LoopsLib::Models::DecompositionResult::BasisElement_t,  //Result type of transforming the value from the input container
        //    std::set<std::size_t>, //Input iterator container type
        //    Frechet::WeightedStrongFrechet //Logger base type
        //>
        //{
        //    LoopsLib::NT m_epsilon;
        //    const std::vector<LoopsLib::Models::DecompositionResult::BasisElement_t>* m_availablePaths;
        //    const LoopsLib::DS::EmbeddedGraph* m_graph;
        //    std::string m_logPathBase;
        //    const std::vector<LoopsLib::NT>* residual;
        //    int m_maxPaths;
        //    double m_maxSearchTime;
        //    // Loggers
        //    std::unique_ptr<std::ostream> warnStream;
        //    std::unique_ptr<std::ostream> logStream;
        //    std::unique_ptr<std::ostream> errStream;
        //    std::unique_ptr<std::ostream> deeplogStream;
        //public:
        //    PathComputer(NT epsilon, const std::vector<LoopsLib::Models::DecompositionResult::BasisElement_t>& availablePaths,
        //        const LoopsLib::DS::EmbeddedGraph* graph,
        //        const std::vector<LoopsLib::NT>& residual,
        //        int maxPaths,
        //        double maxSearchTime)
        //        :m_epsilon(epsilon),
        //        m_availablePaths(&availablePaths),
        //        m_graph(graph),
        //        residual(&residual),
        //        m_maxPaths(maxPaths),
        //        m_maxSearchTime(maxSearchTime)
        //    {
        //    }
        //    PathComputer(PathComputer&& other):
        //        m_epsilon(other.m_epsilon),
        //        m_availablePaths(other.m_availablePaths),
        //        m_graph(other.m_graph),
        //        residual(other.residual),
        //        m_maxPaths(other.m_maxPaths),
        //        m_maxSearchTime(other.m_maxSearchTime),
        //        warnStream(std::move(other.warnStream)),
        //        logStream(std::move(other.logStream)),
        //        errStream(std::move(other.errStream)),
        //        deeplogStream(std::move(other.deeplogStream))
        //    {   
        //    }
        //    PathComputer& operator=(PathComputer&& other)
        //    {
        //        m_epsilon = (other.m_epsilon);
        //        m_availablePaths = (other.m_availablePaths);
        //        m_graph = (other.m_graph);
        //        residual = (other.residual);
        //        m_maxPaths = (other.m_maxPaths);
        //        m_maxSearchTime = (other.m_maxSearchTime);
        //        warnStream = (std::move(other.warnStream));
        //        logStream = (std::move(other.logStream));
        //        errStream = (std::move(other.errStream));
        //        deeplogStream = (std::move(other.deeplogStream));
        //    }
        //    void setLogBase(const std::string& logBase)
        //    {
        //        m_logPathBase = logBase;
        //    }

        //    void init() override
        //    {
        //        if (!m_logPathBase.empty())
        //        {
        //            const auto threadIndex = m_threadNum;
        //            std::string logPath = m_logPathBase;
        //            auto pathWithExt = [logPath, threadIndex](const std::string& ext) {return logPath + ext + std::to_string(threadIndex); };

        //            warnStream.reset(new std::ofstream(pathWithExt(".warn")));
        //            errStream.reset(new std::ofstream(pathWithExt(".err")));
        //            logStream.reset(new std::ofstream(pathWithExt(".log")));
        //            deeplogStream.reset(new std::ofstream(pathWithExt(".deeplog")));
        //            // Make the weighted Frechet log to files
        //            // Add the streams 
        //            m_logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Info>(logStream.get());
        //            m_logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Warn>(warnStream.get());
        //            m_logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Error>(errStream.get());
        //            m_logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Trace>(deeplogStream.get());
        //            m_logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Deep>(deeplogStream.get());
        //            m_logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Debug>(deeplogStream.get());
        //            // Add extras
        //            m_logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Warn>({ logStream.get() });
        //            m_logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Error>({ warnStream.get(), logStream.get() });
        //        }
        //    }
        //    void wrapUp() override
        //    {
        //        if (!m_logPathBase.empty())
        //        {
        //            // Unregister and reset outputs
        //            m_logger.unregisterStream(warnStream.get());
        //            m_logger.unregisterStream(logStream.get());
        //            m_logger.unregisterStream(errStream.get());
        //            m_logger.unregisterStream(deeplogStream.get());

        //            // Add the streams 
        //            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Info>();
        //            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Warn>();
        //            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Error>();
        //            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Trace>();
        //            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Deep>();
        //            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Debug>();
        //        }
        //    }
        //    //PathComputer& operator=(const PathComputer&) = delete;

        //    void compute(std::set<std::size_t>::const_iterator current, std::vector<LoopsLib::Models::DecompositionResult::BasisElement_t>& output) override
        //    {
        //        Frechet::WeightedStrongFrechet weightedFrechet(m_graph);
        //        weightedFrechet.setMaxSearchTimeS(m_maxSearchTime);
        //        weightedFrechet.setMaximumPerPath(m_maxPaths);
        //        weightedFrechet.compute((*m_availablePaths)[*current], m_epsilon, *residual, output);
        //    }
        //};


    public:
        WeightedFrechetPathsDecomposition(LoopsLib::Models::DecompositionResult* decompObj);

    public: // Parameters
        bool noDataCaching() const;

        void setGreedyAttachEnds(bool value);

        bool greedyAttachEnds() const;

        void setNoDataCaching(bool value);

        void setMaxSearchTime(double value);

        double maxSearchTime() const;

        void setMaximumPerPath(int maxim);

        int maximumPerPath() const;

        void setNumberOfThreads(int maxim);

        int numberOfThreads() const;

    protected:
        void findNewBasisElementsParallel();
        void findNewBasisElements() override;
    public:

        std::map<std::string, std::string> metaData() const override
        {
            return {
            {"noDataCaching", std::to_string(m_noDataCaching)}
            };
        }
        void setFromMetaData(const std::map<std::string, std::string>& metData) override
        {
            m_noDataCaching = std::stoi(metData.at("noDataCaching").c_str());
        }

        void reset() override;
        std::string paramDescription() const override
        {
            std::stringstream ss;
            char sep = ' ';
            ss << "noDataCaching" << sep << m_noDataCaching << sep
                << "maxSearchTime" << sep << m_maxSearchTimeS << sep
            <<"numberOfThreads" << sep << m_numberOfThreads << sep
            << "maxPerPath" << sep << m_maxNum;
            return ss.str();
        }
    };
}
#endif