#ifndef LOOPSLIB_DS_GRAPHVIEW_H
#define LOOPSLIB_DS_GRAPHVIEW_H
#include <vector>
#include <set>
#include <queue>
#include <stack>
#include <Eigen/Eigen>
#include <unordered_set>
#include <LoopsLib/Helpers/Iterators.h>
#include "OwnGraph.h"

namespace LoopsLib::DS
{
    // Basic graph view
    class AbstractGraphView
    {
    public:
        using Id_t = BaseGraph::Id_t;
        using EdgeHandle = BaseGraph::Edge*;
        using VertexHandle = BaseGraph::Vertex*;
    public:

        virtual std::size_t number_of_vertices() const = 0;
        virtual EdgeHandle edge(Id_t edgeId) const = 0;
        virtual VertexHandle vertex(Id_t vertexId) const = 0;
        virtual EdgeHandle findConnection(VertexHandle v0, Id_t targetVertex) const = 0;
    };

    // Basic graph view
    class GraphView
    {
    public:
        using Id_t = BaseGraph::VertexIndex;
        using EdgeHandle = const BaseGraph::Edge*;
        using VertexHandle = const BaseGraph::Vertex*;
        using VertexIndex = BaseGraph::VertexIndex;
    private:
        const BaseGraph* m_targetGraph;
        std::set<Id_t> m_availableVertices;
        std::vector<Id_t> m_availalbleVerticesList;

        std::size_t m_edgeCount = 0;
    public:
        bool isAvailable(Id_t vertId) const;
        GraphView();
        void clear()
        {
            m_availableVertices.clear();
            m_availalbleVerticesList.clear();
        }
        GraphView(const BaseGraph* targetGraph);
        void setGraph(const BaseGraph* graph);

        const BaseGraph* rawGraph() const;

        std::size_t indexForVertex(Id_t vertId) const;
        std::size_t indexForVertex(VertexHandle vert) const;

        /**
         * \brief Returns the set of vertex IDs of available vertices, where the ID is
         * in the original graph
         * \return The set of available vertices
         */
        std::set<Id_t>& availableVertices();

        /**
         * \brief Returns the set of vertex IDs of available vertices, where the ID is
         * in the original graph
         * \return The set of available vertices
         */
        const std::set<Id_t>& availableVertices() const;

        std::size_t numberOfEdges() const;

        void updateAvailableVerts();

        inline auto outEdges(Id_t vertxId) const
        {
            /*auto begin = Helpers::Iterators::filter_iterator(m_targetGraph->vertex(vertxId)->m_outEdges,
                [this](std::size_t inx, DS::BaseGraph::Edge* val)
            {
                return isAvailable(val->m_source->id()) && isAvailable(
                    val->m_sink->id());
            });*/
            std::vector<DS::BaseGraph::Edge*> edges;
            for (auto* e : m_targetGraph->vertex(vertxId)->m_outEdges)
            {
                if (isAvailable(e->m_sink->id())) edges.push_back(e);
            }
            return edges;
        }
        inline auto outEdges(VertexHandle handle) const
        {
            /*auto begin = Helpers::Iterators::filter_iterator(m_targetGraph->vertex(vertxId)->m_outEdges,
                [this](std::size_t inx, DS::BaseGraph::Edge* val)
            {
                return isAvailable(val->m_source->id()) && isAvailable(
                    val->m_sink->id());
            });*/
            std::vector<DS::BaseGraph::Edge*> edges;
            for (auto* e : handle->m_outEdges)
            {
                if (isAvailable(e->m_sink->id())) edges.push_back(e);
            }
            return edges;
        }
        inline auto outEdgesByIndex(Id_t vertxIndex) const
        {
            return outEdges(m_availalbleVerticesList[vertxIndex]);
        }

        inline auto inEdges(Id_t vertxId) const
        {
            auto begin = Helpers::Iterators::filter_iterator(m_targetGraph->vertex(vertxId)->m_inEdges,
                [this](std::size_t inx, DS::BaseGraph::Edge* val)
            {
                return isAvailable(val->m_source->id()) && isAvailable(
                    val->m_sink->id());
            });
            return Helpers::Iterators::Iterable(begin, begin.associatedEnd());
        }

        std::size_t number_of_vertices() const;

        EdgeHandle edge(Id_t edgeId) const;

        VertexHandle vertexByIndex(std::size_t index) const;

        /**
         * \brief Retrieve a vertex handle by using an ID in the original graph
         * \param vertexId The vertex ID
         * \return The handle, or an invalid handle if the ID is not part of the view.
         */
        VertexHandle vertex(Id_t vertexId) const;

        /**
         * \brief Searches for an edge connecting v0 to the target vertex while taking into account the allowed vertices.
         * Returns nullptr if no such edge exists.
         * \param v0
         * \param targetVertex
         * \return
         */
        EdgeHandle findConnection(VertexHandle v0, Id_t targetVertex);
    };
}
#endif