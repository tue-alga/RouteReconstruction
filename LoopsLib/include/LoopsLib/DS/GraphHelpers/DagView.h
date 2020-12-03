#ifndef DS_GRAPHHELPERS_DAGVIEW_H
#define DS_GRAPHHELPERS_DAGVIEW_H
#include <LoopsLib/DS/BaseGraph.h>

namespace LoopsLib::DS::GraphHelpers
{

    class DagView
    {
        // The underlying graph
        BaseGraph* m_graph;
        // The mapping from vertex to location in topological order of DAG.
        const std::vector<int>& m_dagMapping;
    public:
        DagView(BaseGraph* graph, const std::vector<int>& dagMapping);

        class DagEdgeIterator
        {
            DagView* m_parent;
            BaseGraph::Vertex* m_vertex;
            decltype(m_vertex->m_outEdges.begin()) it;
            decltype(m_vertex->m_outEdges.begin()) m_end;

            bool isAbsent(BaseGraph::Edge* edge);
        public:
            static DagEdgeIterator end(DagView* parent, BaseGraph::Vertex* vertex, bool isOut);

            DagEdgeIterator(DagView* parent, BaseGraph::Vertex* vertex, bool isOut);

            BaseGraph::Edge* operator*() const;

            BaseGraph::Edge* operator->() const;

            bool operator!=(const DagEdgeIterator& other) const;

            bool operator==(const DagEdgeIterator& other) const;

            DagEdgeIterator& operator++();
        };

        /**
         * \brief Iterable object for DAG edges.
         */
        class DagEdges
        {
            friend class DagView;
            DagView* m_parent;
            BaseGraph::Vertex* m_vert;
            bool m_isOut;
            DagEdges(DagView* parent, BaseGraph::Vertex* vert, bool isOut);
        public:
            DagEdgeIterator begin();

            DagEdgeIterator end();
        };

        DagEdgeIterator outEdgesBegin(BaseGraph::Vertex* vert);

        DagEdgeIterator outEdgesEnd(BaseGraph::Vertex* vert);

        DagEdgeIterator inEdgesBegin(BaseGraph::Vertex* vert);

        DagEdgeIterator inEdgesEnd(BaseGraph::Vertex* vert);

        DagEdges dagEdgesOut(BaseGraph::Vertex* vert);

        DagEdges dagEdgesIn(BaseGraph::Vertex* vert);
    };
}
#endif