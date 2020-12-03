#include <LoopsAlgs/FlowDecomposition/ShortestPathsDecomposition.h>
#include <LoopsLib/DS/BoostInterface.h>
#include <boost/graph/filtered_graph.hpp>
#include <LoopsLib/DS/BoostModificationGraph.h>
#include <LoopsLib/GraphAlgs/WidestPath.h>

LoopsAlgs::FlowDecomposition::ShortestPathsDecomposition::ShortestPathsDecomposition(
    LoopsLib::Models::DecompositionResult* decompObj): IFlowDecomposition("Widest paths Decomp", decompObj)
{

}

void LoopsAlgs::FlowDecomposition::ShortestPathsDecomposition::setMaxPathsPerAvailable(int value)
{
    m_maxPathsPerAvailable = value;
}

void LoopsAlgs::FlowDecomposition::ShortestPathsDecomposition::findNewBasisElements()
{
    // Setup index if needed
    if (graph()->getIndex().isEmpty())
    {
        graph()->getIndex().construct(graph()->locations());
    }

    double epsilon = m_decompObj->m_relatedInstance->m_epsilon;

    int currPath = 1;
    const auto totalPath = numberOfRepresentatives();

    // Logger
    auto logger = LoopsLib::Helpers::logFactory(this);

    for (auto reprInd = 0; reprInd < numberOfRepresentatives(); ++reprInd)
    {
        const auto& repr = representative(reprInd);
        struct VertexSelection
        {
            using vertex_descriptor = typename boost::graph_traits<LoopsLib::DS::EmbeddedGraph>::vertex_descriptor;
            std::set<vertex_descriptor>* available = nullptr;

            VertexSelection(){}
            VertexSelection(std::set<vertex_descriptor>* available):available(available){}

            bool operator()(typename boost::graph_traits<LoopsLib::DS::EmbeddedGraph>::vertex_descriptor v) const
            {
                return available->find(v) != available->end();
            }
        };
        std::set<typename boost::graph_traits<LoopsLib::DS::EmbeddedGraph>::vertex_descriptor> available;

        const auto desc = decompositionObject()->m_relatedInstance->representativeDescription(reprInd);
        graph()->getIndex().containedInBuffer(repr, epsilon, available);

        logger.debug( "Path " ,desc ,": " ,currPath ,"/" ,totalPath );
        logger.debug("Path ", desc, ": element count: ", repr.size());
        logger.debug("Path ", desc, ": view vertex count: ", available.size());
        logger.debug("Begin loc: ", repr.front().x(), ',', repr.front().y());
        

        boost::filtered_graph<LoopsLib::DS::EmbeddedGraph, boost::keep_all, VertexSelection> filtGraph(*graph(), {}, VertexSelection(&available));

        // Find widesth paths
        LoopsLib::DS::AddOnlyModifyGraph<decltype(filtGraph)> modGraph(filtGraph);
        // Get source and sink locations for the path
        std::vector<long long> srcLocations, sinkLocations;
        graph()->getIndex().containedInDisk(repr[0].x(), repr[0].y(), epsilon, srcLocations);
        graph()->getIndex().containedInDisk(repr.back().x(), repr.back().y(), epsilon, sinkLocations);
        // Create super source and super sink
        auto srcVert = boost::add_vertex(modGraph);
        std::for_each(srcLocations.begin(), srcLocations.end(), [&modGraph,&srcVert](const auto& v )
        {
            boost::add_edge(srcVert, v, modGraph);
        });
        auto sinkVert = boost::add_vertex(modGraph);
        std::for_each(sinkLocations.begin(), sinkLocations.end(), [&modGraph, &sinkVert](const auto& v)
        {
            boost::add_edge(v, sinkVert, modGraph);
        });

        // Make new vertices part of the view.
        available.insert(srcVert);
        available.insert(sinkVert);

        logger.debug("Src connections: ", boost::out_degree(srcVert, modGraph));
        logger.debug("Sink connections: ", boost::in_degree(sinkVert, modGraph));
        
        // The current residual
        auto residual = m_decompObj->residual();

        // Add stub values for the added edges for the super source/sink that do not contribute to the actual problem
        for (int i = 0; i < srcLocations.size() + sinkLocations.size(); ++i)
        {
            residual.push_back(std::numeric_limits<NT>::max());
        }

        // Pass residual by reference, we will update it after finding a path
        LoopsLib::GraphAlgs::EdgeToWeightMapper<std::decay_t<decltype(modGraph)>, decltype(residual)&>
            weightProvider(residual);
        // Find as many positive paths as possible
        int pathAddCount = 0;
        while (true)
        {
            LoopsLib::GraphAlgs::WidestPath<std::decay_t<decltype(modGraph)>, decltype(weightProvider)> wp;
            NT outValue;
            using OutPath_t = std::vector<decltype(wp)::Edge_t>;
            std::vector<decltype(wp)::Edge_t> outPath;
            wp.compute(modGraph, weightProvider, srcVert, sinkVert, outPath, outValue);
            if (outPath.empty())
            {
                logger.debug("No widest path found");
                break;
            }
            // Determine total weight
            NT total = 0;
            NT minVal = std::numeric_limits<NT>::max();
            for (int i = 1; i < outPath.size() - 1; ++i)
                //Explicitly ignore first and last edge, since these are not part ofthe actual road network
            {
                total += residual[outPath[i]];
                minVal = std::min(minVal, residual[outPath[i]]);
            }
            // Done if we found a negative path
            if (total < 1e-7)
            {
                logger.debug("Small or negative path found");
                break;
            }
            logger.trace("Path with total ", total, ", min : ", minVal);
            // Remove virtual edges from path
            basis().emplace_back(OutPath_t(std::next(outPath.begin()), std::prev(outPath.end())));

            // Subtract mean value
            NT mean = total / (NT)outPath.size();
            for (const auto& edge : outPath)
            {
                residual[edge] -= mean;
            }
            ++pathAddCount;
        }
        logger.debug("--- Paths added: ", pathAddCount);
        ++currPath;
    }
    logger.debug("Epsilon:", epsilon);
}

std::map<std::string, std::string> LoopsAlgs::FlowDecomposition::ShortestPathsDecomposition::metaData() const
{
    return {};
}

void LoopsAlgs::FlowDecomposition::ShortestPathsDecomposition::setFromMetaData(
    const std::map<std::string, std::string>& metData)
{
}

std::string LoopsAlgs::FlowDecomposition::ShortestPathsDecomposition::paramDescription() const
{
    return "";
}
