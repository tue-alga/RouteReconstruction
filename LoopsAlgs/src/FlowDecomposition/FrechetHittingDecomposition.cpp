#include <LoopsAlgs/FlowDecomposition/FrechetHittingDecomposition.h>
#include <future>
#include <LoopsAlgs/Multithreading/Helpers.h>
using namespace LoopsLib;
LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::FrechetHittingDecomposition(
    Models::DecompositionResult* decompObj): IFlowDecomposition("Frechethitting", decompObj)
{
}

void LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::extendEnds(const Frechet::Polyline& representative,
    const std::vector<LoopsLib::NT>& residual, std::vector<long long>& path)
{
    // Do begin of line first
    std::set<long long> occupiedVertices;
    std::copy(path.begin(), path.end(), std::inserter(occupiedVertices, occupiedVertices.end()));

    std::set<long long >vertSet;
    std::vector<long long> availVec;
    graph()->getIndex().containedInDisk(representative.front().m_p0.x(), representative.front().m_p0.y(),
        this->decompositionObject()->m_relatedInstance->m_epsilon, availVec);
    std::copy(availVec.begin(), availVec.end(), std::inserter(vertSet, vertSet.end()));

    std::size_t totalSize = path.size();
    LoopsLib::NT totalWeight = std::accumulate(path.begin(), path.end(), (NT)0.0, [&residual](LoopsLib::NT v, auto e)
    {
        return v + residual[e];
    });
    std::vector<DS::BaseGraph::Id_t> prefix,postfix;


    //Greedily add high weight edges at start
    auto currV = graph()->edge(path.front())->m_source;
    while (true)
    {
        long long maxEdge = -1;
        NT maxWeight = std::numeric_limits<NT>::lowest();
        for (auto* e : currV->m_inEdges)
        {
            // Vert is not in epsilon distance of front
            if (vertSet.find(e->m_source->id()) == vertSet.end()) continue;

            if (occupiedVertices.find(e->m_source->id()) != occupiedVertices.end()) continue;
            auto w = residual.at(e->id());
            if (w > maxWeight)
            {
                maxWeight = w;
                maxEdge = e->id();
            }
        }
        // Heuristic: only add if mean becomes better.
        if (maxEdge != -1 && (totalWeight + maxWeight) / (NT)(totalSize + 1) > totalWeight / (NT)(totalSize))
        {
            //Update size and weight
            totalWeight += maxWeight;
            ++totalSize;

            prefix.push_back(maxEdge);
            currV = graph()->edge(maxEdge)->m_source;
            occupiedVertices.insert(currV->id());
        }
        else
        {
            break;
        }
    }
    availVec.clear();
    graph()->getIndex().containedInDisk(representative.back().m_p0.x(), representative.back().m_p0.y(),
        this->decompositionObject()->m_relatedInstance->m_epsilon, availVec);
    vertSet.clear();
    std::copy(availVec.begin(), availVec.end(), std::inserter(vertSet, vertSet.end()));

    //Greedily add high weight edges at end
    currV = graph()->edge(path.back())->m_sink;
    while (true)
    {
        long long maxEdge = -1;
        NT maxWeight = std::numeric_limits<NT>::lowest();
        for (auto* e : currV->m_outEdges)
        {
            // Vert is not in epsilon distance of front
            if (vertSet.find(e->m_sink->id()) == vertSet.end()) continue;

            if (occupiedVertices.find(e->m_sink->id()) != occupiedVertices.end()) continue;
            auto w = residual.at(e->id());
            if (w > maxWeight)
            {
                maxWeight = w;
                maxEdge = e->id();
            }
        }
        // Heuristic: only add if mean becomes better.
        if (maxEdge != -1 && (totalWeight + maxWeight) / (NT)(totalSize + 1) > totalWeight / (NT)(totalSize))
        {
            //Update size and weight
            totalWeight += maxWeight;
            ++totalSize;

            postfix.push_back(maxEdge);
            currV = graph()->edge(maxEdge)->m_sink;
            occupiedVertices.insert(currV->id());
        }
        else
        {
            break;
        }
    }
    std::vector<DS::BaseGraph::Id_t> total;
    // Prefix was constructed in reverse order relative to path
    total.insert(total.end(), prefix.rbegin(), prefix.rend());
    total.insert(total.end(), path.begin(), path.end());
    total.insert(total.end(), postfix.begin(), postfix.end());
    path = total;
}

template<typename ReturnType>
struct Task
{
public:
    virtual ~Task(){}
    virtual ReturnType run() const = 0;
};

//class PathComputer: public LoopsAlgs::Multithreading::Task<
//    LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::BasisElement_t,
//    std::set<std::size_t>,
//    LoopsAlgs::Frechet::PathHittingStrongFrechet
//>
//{
//public:
//    LoopsLib::NT m_epsilon;
//    const std::vector<LoopsLib::Models::DecompositionResult::BasisElement_t>& m_availablePaths;
//    const LoopsLib::DS::EmbeddedGraph* m_graph;
//    std::string m_logPathBase;
//    const std::vector<LoopsLib::NT>& residual;
//    int m_maxPaths;
//    std::unique_ptr<std::ostream> warnStream;
//    std::unique_ptr<std::ostream> logStream;
//    std::unique_ptr<std::ostream> errStream;
//    std::unique_ptr<std::ostream> deeplogStream;
//
//    PathComputer(NT epsilon, const std::vector<Models::DecompositionResult::BasisElement_t>& availablePaths, 
//        const LoopsLib::DS::EmbeddedGraph* graph,
//        const std::vector<LoopsLib::NT>& residual,
//        int maxPaths)
//        :m_epsilon(epsilon),
//        m_availablePaths(availablePaths),
//        m_graph(graph),
//    residual(residual),
//    m_maxPaths(maxPaths)
//    {
//    }
//
//    void init() override
//    {
//        const auto threadIndex = m_threadNum;
//        if (!m_logPathBase.empty())
//        {
//            std::string logPath = m_logPathBase;
//            auto pathWithExt = [logPath, threadIndex](const std::string& ext) {return logPath + ext + std::to_string(threadIndex); };
//
//            warnStream.reset(new std::ofstream(pathWithExt(".warn")));
//            errStream.reset(new std::ofstream(pathWithExt(".err")));
//            logStream.reset(new std::ofstream(pathWithExt(".log")));
//            deeplogStream.reset(new std::ofstream(pathWithExt(".deeplog")));
//            // Make the weighted Frechet log to files
//
//            // Reset the streams
//            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Info>();
//            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Warn>();
//            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Error>();
//            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Trace>();
//            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Deep>();
//            m_logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Debug>();
//
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
//
//        }
//    }
//
//    void wrapUp() override
//    {
//        if (!m_logPathBase.empty())
//        {
//            // Make the weighted Frechet log to files
//            auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>();
//
//            logger.unregisterStream(warnStream.get());
//            logger.unregisterStream(logStream.get());
//            logger.unregisterStream(errStream.get());
//            logger.unregisterStream(deeplogStream.get());
//
//            // Add the streams 
//            logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Info>();
//            logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Warn>();
//            logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Error>();
//            logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Trace>();
//            logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Deep>();
//            logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Debug>();
//        }
//    }
//
//    void compute(std::set<std::size_t>::const_iterator knownPathIndIt,
//        LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::Basis_t& output) override
//    {
//        std::size_t knownPathInd = *knownPathIndIt;
//        LoopsAlgs::Frechet::PathHittingStrongFrechet hitAlg(m_graph);
//        auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>();
//
//        LoopsAlgs::Frechet::StrongFrechetGraphData data;
//        hitAlg.precompute(m_availablePaths[knownPathInd], m_epsilon, data);
//
//        {
//            const auto sizeEstimate = data.byteSizeEstimate();
//            m_logger.info("Bytes in data object approx: ", sizeEstimate, " bytes = ~", sizeEstimate / 1024 / 1024, " MB, for path size ", m_availablePaths[knownPathInd].size());
//        }
//        
//
//        const auto epsilon = data.m_epsilon;
//
//        std::vector<std::vector<LoopsAlgs::Frechet::PathPointer>> precomputePointers;
//        hitAlg.precomputePathPointers(data, precomputePointers);
//        {
//            std::size_t sizeEstimate = 0;
//            for(const auto& el: precomputePointers)
//            {
//                sizeEstimate += sizeof(el) + sizeof(LoopsAlgs::Frechet::PathPointer) * el.size();
//            }
//            m_logger.info("Bytes in precompute pointers ", sizeEstimate, " bytes = ~", sizeEstimate / 1024 / 1024, " MB ");
//        }
//
//        const auto totalV = data.m_view.number_of_vertices();
//
//        // Collect edges
//        std::vector<LoopsLib::DS::BaseGraph::Id_t> edgeIds;
//        for (std::size_t i = 0; i < totalV; ++i)
//        {
//            for (auto e : data.m_view.outEdgesByIndex(i))
//            {
//                edgeIds.push_back(e->id());
//            }
//        }
//        std::sort(edgeIds.begin(), edgeIds.end(), [this](DS::BaseGraph::Id_t e0, DS::BaseGraph::Id_t e1)
//        {
//            return residual[e0] > residual[e1];
//        });
//        {
//            std::size_t sizeEstimate = 0;
//            for (const auto& el : edgeIds)
//            {
//                sizeEstimate += sizeof(el);
//            }
//            m_logger.info("Bytes in edge list ", sizeEstimate, " bytes = ~", sizeEstimate / 1024 / 1024, " MB ");
//        }
//        int foundPaths = 0;
//
//        m_logger.info("Number of streams: ", m_logger.getRawLogger<Helpers::LogLevel::Info>().localStream.streams.size());
//        
//
//        std::size_t eInd = 0;
//        for (const auto& eId: edgeIds)
//        {
//            LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>::GlobalPrefix = "[Path " + std::to_string(knownPathInd) +
//                ":e-" + std::to_string(eInd) + "/"+std::to_string(edgeIds.size())+"]";
//            std::vector<DS::BaseGraph::Id_t> outPath;
//            try
//            {
//                hitAlg.computeNextUsingPathPointers(data, eId, precomputePointers, outPath);
//            }
//            catch (std::exception& except)
//            {
//                logger.error("Caught exception ", except.what(), " for edge ", eId, " at path ", knownPathInd);
//            }
//            ++eInd;
//            if (outPath.empty()) continue;
//            output.push_back(outPath);
//            ++foundPaths;
//            if (foundPaths == m_maxPaths)break;
//        }
//        {
//            std::size_t sizeEstimate = 0;
//            for (const auto& el : output)
//            {
//                sizeEstimate += sizeof(el) + sizeof(el[0]) * el.size();
//            }
//            m_logger.info("Bytes in thread basis after add ", sizeEstimate, " bytes = ~", sizeEstimate / 1024 / 1024, " MB ");
//        }
//    }
//};

void LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::findPaths(LoopsAlgs::Frechet::StrongFrechetGraphData& data,
    const std::vector<LoopsLib::NT>& residual,
    std::size_t pathIndex, std::size_t currentBasisSize,
                                                                     Basis_t& output)
{
    LoopsAlgs::Frechet::PathHittingStrongFrechet hitAlg(graph());
    auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>();
    const auto epsilon = m_decompObj->m_relatedInstance->m_epsilon;

    int foundPaths = 0;

    std::vector<std::vector<LoopsAlgs::Frechet::PathPointer>> precomputePointers;
    hitAlg.precomputePathPointers(data, precomputePointers);

    const auto totalV = data.m_view.number_of_vertices();

    // Collect edges
    std::vector<LoopsLib::DS::BaseGraph::Id_t> edgeIds;
    for (std::size_t i = 0; i < totalV; ++i)
    {
        for (auto e : data.m_view.outEdgesByIndex(i))
        {
            edgeIds.push_back(e->id());
        }
    }
    std::sort(edgeIds.begin(), edgeIds.end(), [&residual](DS::BaseGraph::Id_t e0, DS::BaseGraph::Id_t e1)
    {
        return residual[e0] > residual[e1];
    });


    std::string pathPref = "[Path " + std::to_string(pathIndex) + "-sz=" + std::to_string(data.polylineEdgeCount());

    std::size_t eInd = 0;
    std::size_t perc = 0;
    std::size_t totalEs = data.m_view.numberOfEdges();
    for (auto eId : edgeIds)
    {
        if(eInd > (perc+1)*10 / (double)100 * totalEs)
        {
            ++perc;
            logger.info("Checking now at ", perc*10,"%");
        }
        auto prefixReset = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>().addGlobalPrefix(pathPref +
            ":e-" + std::to_string(eInd) + "/" + std::to_string(totalEs) + ",bsz-" + std::to_string(currentBasisSize + output.size()) + "]");
        if (residual[eId] < 0) {
            logger.info("Reached negative weights for ", eId);
            break;
        }

        std::vector<DS::BaseGraph::Id_t> outPath;
        try
        {
            hitAlg.computeNextUsingPathPointers(data, eId, precomputePointers, outPath);
        }
        catch (std::exception& except)
        {
            logger.error("Caught exception ", except.what(), " for edge ", eId, " at vert ", eId);
        }
        ++eInd;
        if (outPath.empty()) continue;

        if(m_greedyAttachEnds)
        {
            extendEnds(data.polyline, residual, outPath);
        }
        logger.info("Adding path");
        output.push_back(outPath);
        ++foundPaths;
        if (foundPaths == m_num) break;
    }
}

void LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::findNewBasisElementsParallel()
{
    throw std::runtime_error("Parallel broken");
    //LoopsAlgs::Multithreading::Runner<Basis_t> r;
    //// Set number of threads to use
    //r.m_parallellism = m_threads;

    //auto residual = m_decompObj->residual();

    //auto creator = [this, residual]()
    //{
    //    PathComputer computer(m_decompObj->m_relatedInstance->m_epsilon, m_decompObj->paths(), graph(),residual, m_num);
    //    if(!m_logDir.empty())
    //    {
    //        if (m_logPrefix.empty())
    //        {
    //            m_logPrefix = "frechethitting";
    //        }
    //        std::string logPath = m_logDir + '/' + m_logPrefix;
    //        computer.m_logPathBase = logPath;
    //    }
    //    return computer;
    //};
    //auto progress = [](int thread, int progress)
    //{
    //    auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>();
    //    logger.info("Progress thread ", thread, ": ", progress, "/100");
    //};
    //auto resultCb = [this](const Basis_t& basis) {this->extendBasis(basis); };

    //r.setProgressCallback(progress);
    //r.run(creator, m_decompObj->m_relatedInstance->m_availablePaths, resultCb);
}

void LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::findNewBasisElements()
{
    if(m_threads > 1)
    {
        findNewBasisElementsParallel();
        return;
    }
    std::unique_ptr<std::ostream> warnStream;
    std::unique_ptr<std::ostream> logStream;
    std::unique_ptr<std::ostream> errStream;
    std::unique_ptr<std::ostream> deeplogStream;
    // Make the weighted Frechet log to files
    auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>();
    auto prefixResetter = logger.addGlobalPrefix(this->m_logPrefix);
    if (!m_logDir.empty())
    {
        if (m_logFilePrefix.empty())
        {
            m_logFilePrefix = "frechethitting";
        }
        std::string logPath = m_logDir + '/' + m_logFilePrefix;
        warnStream.reset(new std::ofstream(logPath + ".warn"));
        errStream.reset(new std::ofstream(logPath + ".err"));
        logStream.reset(new std::ofstream(logPath + ".log"));
        deeplogStream.reset(new std::ofstream(logPath + ".deeplog"));
        
        // Add the streams 
        logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Info>({ logStream.get() });
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Warn>(warnStream.get());
        logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Warn>({ logStream.get() });
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Error>(errStream.get());
        logger.addLevelOutput<LoopsLib::Helpers::LogLevel::Error>({ warnStream.get(), logStream.get() });
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Trace>(deeplogStream.get());
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Deep>(deeplogStream.get());
        logger.setLevelOutput<LoopsLib::Helpers::LogLevel::Debug>(deeplogStream.get());
    }

    const auto epsilon = m_decompObj->m_relatedInstance->m_epsilon;
    std::size_t pInd = 0;
    auto resid = m_decompObj->residual();
    for (std::size_t reprInd = 0; reprInd < numberOfRepresentatives(); ++reprInd)
    {
        const auto& repr = representative(reprInd);

        try
        {
            LoopsAlgs::Frechet::PathHittingStrongFrechet hitAlg(graph());
            logger.info("Computing Frechet graph data");
            Frechet::StrongFrechetGraphData data;
            hitAlg.precompute(repr, epsilon, data);
            logger.info("Frechet graph data computed");

            {
                const auto sizeEstimate = data.byteSizeEstimate();
                logger.info("Bytes in data object approx: ", sizeEstimate, " bytes = ~", sizeEstimate / 1024 / 1024, " MB, for path size ", repr.size());
            }

            Basis_t extra;

            logger.info("Starting search");
            findPaths(data, resid, pInd, basis().size(), extra);
            basis().insert(basis().end(), extra.begin(), extra.end());
        }
        catch(std::exception& e)
        {
            logger.error("Caught exception ", e.what(), " for path ", m_decompObj->m_relatedInstance->representativeDescription(reprInd), " of size ", repr.size());
        }

        ++pInd;
    }
    if (!m_logDir.empty())
    {
        // Make the weighted Frechet log to files
        auto logger = LoopsLib::Helpers::LogFactory<LoopsAlgs::Frechet::PathHittingStrongFrechet>();
        // Add the streams 
        logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Info>();
        logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Warn>();
        logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Error>();
        logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Trace>();
        logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Deep>();
        logger.resetLevelOutput<LoopsLib::Helpers::LogLevel::Debug>();
    }
}

void LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::setGreedyAttachEnds(const bool& greedyAttachEnds)
{
    m_greedyAttachEnds = greedyAttachEnds;
}

bool LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition::greedyAttachEnds() const
{
    return m_greedyAttachEnds;
}
