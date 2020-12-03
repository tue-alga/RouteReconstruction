#ifndef GRAPHALGS_BFS_H
#define GRAPHALGS_BFS_H
#include <LoopsLib/DS/BaseGraph.h>
namespace LoopsLib::GraphAlgs
{
    class BFSIterator
    {
        DS::BaseGraph* m_graph;
        DS::BaseGraph::Vertex* m_current = nullptr;
        std::unordered_set<DS::BaseGraph::Id_t> m_seen;
        std::queue<DS::BaseGraph::Vertex*> m_queue;
        using EdgeIt = decltype(m_current->m_outEdges.begin());
        EdgeIt m_edgeIt;
        EdgeIt m_end;

        bool seen(DS::BaseGraph::Vertex* vert) const
        {
            return m_seen.find(vert->id()) != m_seen.end();
        }
        void setToEnd()
        {
            while (!m_queue.empty())m_queue.pop();
            // Set iterators to arbitrary but fixed value
            m_edgeIt = m_end = m_graph->vertex(0)->m_outEdges.begin();
            m_current = nullptr;
        }
    public:
        BFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Vertex* start) :
            m_graph(graph),
            m_current(start),
            m_edgeIt(start->m_outEdges.begin()),
            m_end(start->m_outEdges.end())
        {

        }
        BFSIterator(DS::BaseGraph* graph, DS::BaseGraph::Id_t start): BFSIterator(graph, graph->vertex(start))
        {
            
        }
        // End iterator
        BFSIterator(DS::BaseGraph* graph):m_graph(graph)
        {
            setToEnd();
        }
        
        void stopIterations()
        {
            setToEnd();
        }
        class BFSResult
        {
            friend class BFSIterator;
            BFSIterator& m_parent;
            BFSResult(BFSIterator& parent) : m_parent(parent){}
        public:
            DS::BaseGraph::Edge* edge() const
            {
                return *(m_parent.m_edgeIt);
            }
            bool sinkSeen() const
            {
                return m_parent.seen(sink());
            }
            DS::BaseGraph::Vertex* source() const
            {
                return (*m_parent.m_edgeIt)->m_source;
            }
            DS::BaseGraph::Vertex* sink() const
            {
                return (*m_parent.m_edgeIt)->m_sink;
            }
            void stopIterations() const
            {
                m_parent.stopIterations();
            }
        };
        BFSIterator& operator++()
        {
            if (!seen((*m_edgeIt)->m_sink))
            {
                m_seen.insert((*m_edgeIt)->m_sink->id());
                m_queue.push((*m_edgeIt)->m_sink);
            }
            ++m_edgeIt;
            if(m_edgeIt == m_end)
            {
                // Done
                if (m_queue.empty()) {
                    setToEnd();
                    return *this;
                }

                auto* vert = m_queue.front();
                m_queue.pop();
                m_edgeIt = vert->m_outEdges.begin();
                m_end = vert->m_outEdges.end();
                m_current = vert;
            }
            return *this;
        }
        BFSResult operator*()
        {
            return BFSResult(*this);
        }
        bool operator!=(const BFSIterator& other) const
        {
            return m_queue.size() != other.m_queue.size() ||
                m_current != other.m_current ||
                m_edgeIt != other.m_edgeIt || m_end != other.m_end
                ;
        }
    };
    class BFSIterable
    {
        DS::BaseGraph* m_graph;
        DS::BaseGraph::Id_t m_start;
    public:
        BFSIterable(DS::BaseGraph* graph, DS::BaseGraph::Id_t start)
            : m_graph(graph),
              m_start(start)
        {
        }

        BFSIterator  begin()
        {
            return BFSIterator(m_graph, m_start);
        }
        BFSIterator  end()
        {
            return BFSIterator(m_graph);
        }
    };

    template<typename Graph_t>
    class TypedBFSIterator
    {
    public:
        using VertexHandle = typename Graph_t::VertexHandle;
        using EdgeHandle = typename Graph_t::EdgeHandle;
        using Idx_t = typename Graph_t::Idx_t;
    private:
        Graph_t* m_graph;
        VertexHandle m_current = nullptr;
        std::unordered_set<Idx_t> m_seen;
        std::queue<VertexHandle> m_queue;
        using EdgeIt = decltype(m_current->outEdges().begin());
        EdgeIt m_edgeIt;
        EdgeIt m_end;

        bool seen(VertexHandle vert) const
        {
            return m_seen.find(vert->id()) != m_seen.end();
        }
        void setToEnd()
        {
            while (!m_queue.empty())m_queue.pop();
            // Set iterators to arbitrary but fixed value
            m_edgeIt = m_end = m_graph->vertex(0)->outEdges().begin();
            m_current = nullptr;
        }
    public:
        TypedBFSIterator(Graph_t* graph, VertexHandle start) :
            m_graph(graph),
            m_current(start),
            m_edgeIt(start->m_outEdges.begin()),
            m_end(start->m_outEdges.end())
        {

        }
        TypedBFSIterator(Graph_t* graph, Idx_t start) : TypedBFSIterator(graph, graph->vertex(start))
        {

        }
        // End iterator
        TypedBFSIterator(DS::BaseGraph* graph) :m_graph(graph)
        {
            setToEnd();
        }

        void stopIterations()
        {
            setToEnd();
        }
        class BFSResult
        {
            friend class TypedBFSIterator;
            TypedBFSIterator& m_parent;
            BFSResult(TypedBFSIterator& parent) : m_parent(parent) {}
        public:
            EdgeHandle edge() const
            {
                return *(m_parent.m_edgeIt);
            }
            bool sinkSeen() const
            {
                return m_parent.seen(sink());
            }
            VertexHandle source() const
            {
                return (*m_parent.m_edgeIt)->sourceVertex();
            }
            VertexHandle sink() const
            {
                return (*m_parent.m_edgeIt)->sinkVertex();
            }
            void stopIterations() const
            {
                m_parent.stopIterations();
            }
        };
        TypedBFSIterator& operator++()
        {
            if (!seen((*m_edgeIt)->sinkVertex()->id()))
            {
                m_seen.insert((*m_edgeIt)->sinkVertex()->id());
                m_queue.push((*m_edgeIt)->sinkVertex()->id());
            }
            ++m_edgeIt;
            if (m_edgeIt == m_end)
            {
                // Done
                if (m_queue.empty()) {
                    setToEnd();
                    return *this;
                }

                auto* vert = m_queue.front();
                m_queue.pop();
                m_edgeIt = vert->outEdges().begin();
                m_end = vert->outEdges().end();
                m_current = vert;
            }
            return *this;
        }
        BFSResult operator*()
        {
            return BFSResult(*this);
        }
        bool operator!=(const TypedBFSIterator& other) const
        {
            return m_queue.size() != other.m_queue.size() ||
                m_current != other.m_current ||
                m_edgeIt != other.m_edgeIt || m_end != other.m_end
                ;
        }
    };
    template<typename Graph_t>
    class TypedBFSIterable
    {
        Graph_t* m_graph;
        typename Graph_t::Idx_t m_start;
    public:
        TypedBFSIterable(Graph_t* graph, typename Graph_t::Idx_t start)
            : m_graph(graph),
            m_start(start)
        {
        }

        TypedBFSIterator<Graph_t>  begin()
        {
            return TypedBFSIterator<Graph_t>(m_graph, m_start);
        }
        TypedBFSIterator<Graph_t> end()
        {
            return TypedBFSIterator<Graph_t>(m_graph);
        }
    };

}
#endif