#ifndef DS_GRAPHHELPERS_GRAPHMODIFICATIONGUARD_H
#define DS_GRAPHHELPERS_GRAPHMODIFICATIONGUARD_H
#include <LoopsLib/DS/OwnGraph.h>
namespace LoopsLib::DS::GraphHelpers
{

    class GraphModificationGuard
    {
        using Id_t = DS::BaseGraph::Id_t;
        std::vector<DS::BaseGraph::Edge*> m_addedEdges;
        std::vector<DS::BaseGraph::Vertex*> m_addedVertices;
        bool m_wasReverted = false;

        struct RemovedEdge
        {
            //
            DS::BaseGraph::Edge* edge;
            Id_t source;
            Id_t sink;
        };
        struct RemovedVertex
        {
            DS::BaseGraph::Vertex* vert = nullptr;
            // In and out edges. Note that they still have their pointers set!
            std::vector<BaseGraph::Edge*> outEdges;
            std::vector<BaseGraph::Edge*> inEdges;
        };
        std::vector<RemovedEdge> m_removedEdges;
        std::vector<RemovedVertex> m_removedVertices;

        BaseGraph* m_target;

        /**
         * \brief Remove the created edges during the lifetime of the modification guard.
         */
        void removeNewEdges();

        void removeNewVertices();

        void addOldVertices();

        void addOldEdges();
    public:
        GraphModificationGuard(BaseGraph* target);

        ~GraphModificationGuard();

        void revertModifications();

        /**
         * \brief Add a new edge
         * \param source The source of the edge
         * \param sink The sink of the edge
         * \return The created edge
         */
        BaseGraph::Edge* addEdge(DS::BaseGraph::Vertex* source, BaseGraph::Vertex* sink);

        std::vector<BaseGraph::Edge*> addEdges(DS::BaseGraph::Vertex* source, const std::vector<long long>& vertexIds);
        std::vector<BaseGraph::Edge*> addEdges(const std::vector<long long>& vertexIds, DS::BaseGraph::Vertex* sink);

        /**
         * \brief Adds a new vertex
         * \return The created vertex
         */
        BaseGraph::Vertex* addVertex();

        /**
         * \brief Removes the given edge from the graph
         * \param e The edge to remove
         */
        void removeEdge(BaseGraph::Edge* e);

        void removeVertex(BaseGraph::Vertex* v);
    };
}
#endif