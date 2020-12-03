#ifndef GRAPH_EMBEDDING_H
#define GRAPH_EMBEDDING_H
#include <LoopsLib/DS/OwnGraph.h>

namespace LoopsLib::DS
{
    template<typename MovetkKernel>
    class GraphEmbedding : public BaseGraph
    {
    public:
        using Point = typename MovetkKernel::MovetkPoint;
        using NT = typename MovetkKernel::NT;
    private:
        std::vector<Point> m_locations;
    public:
        void setLocations(const std::vector<Point>& locations)
        {
            m_locations = locations;
        }
        std::vector<Point>& locations()
        {
            return m_locations;
        }
        const std::vector<Point>& locations() const
        {
            return m_locations;
        }
        NT edgeLength(DS::BaseGraph::Id_t edgeId) const
        {
            //TODO
            return 0;
        }
    };
}
#endif