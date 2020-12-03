#include <LoopsLib/DS/OwnGraph.h>

using namespace LoopsLib::DS;


bool LoopsLib::DS::isConnected(BaseGraph* graph, long long source, long long sink)
{
    std::queue<long long> toProcess;
    toProcess.push(source);
    std::set<long long> seen;
    while (!toProcess.empty())
    {
        auto el = toProcess.front();
        toProcess.pop();
        seen.insert(el);
        if (el == sink) return true;
        for (auto e : graph->vertex(el)->m_outEdges)
        {
            auto newVId = e->m_sink->id();
            if (seen.find(newVId) == seen.end())
                toProcess.push(newVId);
        }
    }
    return false;
}

bool LoopsLib::DS::isDagConnected(BaseGraph* graph, const std::vector<int>& dagMapping, long long source, long long sink)
{
    std::queue<long long> toProcess;
    toProcess.push(source);
    std::set<long long> seen;
    while (!toProcess.empty())
    {
        auto el = toProcess.front();
        toProcess.pop();
        seen.insert(el);
        if (el == sink) return true;
        for (auto e : graph->vertex(el)->m_outEdges)
        {
            // Ignore edges that are not in the DAG
            if (dagMapping[e->m_sink->id()] < dagMapping[el]) continue;

            auto newVId = e->m_sink->id();
            if (seen.find(newVId) == seen.end())
                toProcess.push(newVId);
        }
    }
    return false;
}
