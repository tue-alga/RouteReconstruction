#ifndef GRAPHALGS_DFS_H
#define GRAPHALGS_DFS_H
#include <LoopsLib/DS/OwnGraph.h>
namespace LoopsLib::GraphAlgs
{
    class DFSIterator;
    /**
         * \brief Data returned by the iterator
         */
    class DFSResult
    {
        friend class DFSIterator;
        // The parent iterator element
        DFSIterator& m_parent;
        DFSResult(DFSIterator& parent);
    public:
        /**
         * \brief The current edge in the search result
         * \return
         */
        DS::BaseGraph::Edge* edge() const;

        /**
         * \brief Was the sink vertex already seen
         * \return
         */
        bool sinkSeen() const;

        DS::BaseGraph::Vertex* source() const;

        DS::BaseGraph::Vertex* sink() const;

        void stopIterations() const;
    };

    class DFSIterator
    {
        friend class DFSResult;
        // Source graph
        DS::BaseGraph* m_graph;
        // Current vertex
        DS::BaseGraph::Vertex* m_current = nullptr;
        // Set of seen vertices
        std::unordered_set<DS::BaseGraph::Id_t> m_seen;
        // Current stack.
        std::stack<DS::BaseGraph::Vertex*> m_queue;

        std::unordered_set<DS::BaseGraph::Id_t> m_validEdges;

        using EdgeIt = decltype(m_current->m_outEdges.begin());

        // Iterators
        EdgeIt m_edgeIt;
        EdgeIt m_end;

        /**
         * \brief Returns whether the given vertex was already seen
         * \param vert The vertex
         * \return Whether the vertex was already seen in the DFS
         */
        bool seen(DS::BaseGraph::Vertex* vert) const;

        /**
         * \brief Make the iterator equal to the end iterator (early out).
         */
        void setToEnd();
    public:
        DFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Vertex* start, const std::unordered_set<DS::BaseGraph::Id_t>& validEdges);
        DFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Vertex* start);

        DFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Id_t start);
        // End iterator
        DFSIterator(DS::BaseGraph* graph);

        void stopIterations();

        std::vector<DS::BaseGraph::Vertex*> currentStack();

        DFSIterator& operator++();

        DFSResult operator*();

        bool operator!=(const DFSIterator& other) const;
    };
    class DFSIterable
    {
        DS::BaseGraph* m_graph;
        DS::BaseGraph::Id_t m_start;
    public:
        DFSIterable(DS::BaseGraph* graph, DS::BaseGraph::Id_t start);

        DFSIterator begin();

        DFSIterator end();
    };

}
#endif