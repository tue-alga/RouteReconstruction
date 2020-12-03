#ifndef DS_BASEGRAPH_H
#define DS_BASEGRAPH_H
#include <tuple>
#include <algorithm>
#include <set>
#include <vector>
#include <Eigen/Eigen>

namespace LoopsLib::DS
{
    /**
     * \brief Base graph for any simple graph, i.e. without self-loops and
     * double edges.
     */
    class BaseGraph
    {
    public:

        using Id_t = long long;
        using Idx_t = Id_t;

        // Forward declare
        struct Vertex;

        struct Edge
        {
            // The edge ID
            long long m_id;
            // Source and sink vertices
            Vertex* m_source = nullptr, *m_sink = nullptr;

            /**
             * \brief Retrieve the ID of the edge
             * \return The ID of the edge
             */
            long long id() const;

            /**
             * \brief Constructs a new edge
             * \param id The unique ID of the edge
             */
            explicit Edge(long long id);

            /**
             * \brief Implicitly converts to the edge ID
             */
            operator long long() const;

            /**
             * \brief Returns the a pointer to the source and sink vertices of the edges,
             * given as an std::pair
             * \return The pair of vertices
             */
            std::pair<Vertex*, Vertex*> verts() const;

            /**
             * \brief Returns whether this edge has the vertex, identified by the given ID
             * as sink
             * \param id The vertex ID
             * \return Whether the sink of this edge is the given vertex
             */
            bool hasSink(long long id) const;

            /**
             * \brief Returns whether this edge has the vertex, identified by the given ID
             * as source
             * \param id The vertex ID
             * \return Whether the source of this edge is the given vertex
             */
            bool hasSource(long long id) const;

            /**
             * \brief Detaches the edge from its vertices
             */
            void detach(bool nullSourceAndSink = true);

            /**
             * \brief Reconnects to the assigned source and sink node. Assumes both are not null
             */
            void reconnect();

            /**
             * \brief Flips the direction of the edge.
             * Effectively swaps the source and sink vertex
             * and corrects the connections in the vertices.
             */
            void flip();
        };
        struct EdgeSortSink
        {
            bool operator()(Edge* e0, Edge* e1) const;
        };
        struct EdgeSortSource
        {
            bool operator()(Edge* e0, Edge* e1) const;
        };
        struct Vertex
        {
            // ID of the vertex
            long long m_id;
            // Outgoing edges
            //std::set<Edge*, EdgeSortSink> m_outEdges;
            std::set<Edge*> m_outEdges;
            // Incoming edges
            //std::set<Edge*, EdgeSortSource> m_inEdges;
            std::set<Edge*> m_inEdges;

            const std::set<Edge*>& outEdges() const
            {
                return m_outEdges;
            }

            const std::set<Edge*>& inEdges() const
            {
                return m_inEdges;
            }

            /**
             * \brief Returns the vertex ID
             * \return The id
             */
            long long id() const;

            /**
             * \brief Construts a vertex
             * \param id The unique identifier for this vertex
             */
            explicit Vertex(long long id);

            /**
             * \brief Can implicitly convert the vertex to its ID value for ease of use.
             */
            operator long long() const;

            Edge* findOutEdge(Vertex* sink) const;

            Edge* findInEdge(Vertex* source) const;

            Edge* findOutEdge(DS::BaseGraph::Id_t sink) const;

            Edge* findInEdge(DS::BaseGraph::Id_t source) const;

            /**
             * \brief Check if the vertex is connected to the given vertex
             * \param otherVertex Other vertex, identified by its ID
             * \return Whether or not the vertex is connected to the given vertex.
             */
            bool connectedTo(long long otherVertex) const;
        };
        using VertexHandle = Vertex * ;
        using EdgeHandle = Edge * ;
        using EdgeIndex = long long;
        using VertexIndex = long long;
    protected:
        // The vertices in the graph
        std::vector<Vertex*> m_vertices;
        // The edges in the graph
        std::vector<Edge*> m_edges;
    public:
        virtual ~BaseGraph();

        /**
         * \brief Empty constructor. Allocate vertices afterwards with
         * allocateVertices()
         */
        BaseGraph();

        /**
         * \brief Clears the graph. All vertices and edges will be deleted
         */
        void clear();

        /**
         * \brief Allocates the given number of vertices. These can then be accessed via
         * vertex() with the IDs up to numberOfVertices-1
         * \param numberOfVertices The number of vertices to allocate
         */
        void allocateVertices(long long numberOfVertices);

        void allocateEdges(long long numberOfEdges);

        BaseGraph& operator=(BaseGraph&& other)
        {

            for (auto* e : m_edges)
            {
                delete e;
            }

            for (auto* v : m_vertices)
            {
                delete v;
            }
            m_vertices = std::move(other.m_vertices);
            m_edges = std::move(other.m_edges);
            return *this;
        }

        BaseGraph(long long numberOfVertices);

        BaseGraph(long long numberOfVertices, long long numberOfEdges);

        const std::vector<Vertex*>& vertices() const;

        const std::vector<Edge*>& edges() const;

        std::size_t number_of_vertices() const;

        Idx_t numberOfVertices() const;

        std::size_t number_of_edges() const;
        Idx_t numberOfEdges() const;

        bool isSimplePath(const std::vector<Id_t>& vertices) const;

        bool isSimpleEdgePath(const Eigen::VectorXd& edges) const;

        bool isSimpleVertPath(const Eigen::VectorXd& verts) const;

        bool isSimplePath(const std::vector<Edge*>& edges) const;

        bool isSimplePath(const std::vector<Vertex*>& vertices) const;

        Edge* edge(long long id);

        Vertex* vertex(long long id);

        const Edge* edge(long long id) const;
        /**
         * \brief Get the constant version of a vertex
         * \param id The ID of the vertex
         * \return The vertex
         */
        const Vertex* vertex(long long id) const;

        virtual Edge* addEdge(long long source, long long sink);

        void setEdgeConnection(long long id, long long source, long long sink)
        {
            initEdge(id, source, sink);
        }

        void initEdge(long long id, long long source, long long sink)
        {
            assert(id < m_edges.size());

            auto* edge = this->edge(id);
            assert(edge->m_source == nullptr);
            edge->m_source = m_vertices[source];
            edge->m_sink = m_vertices[sink];
            edge->m_source->m_outEdges.insert(edge);
            edge->m_sink->m_inEdges.insert(edge);
        }

        virtual Edge* addEdge(Vertex* source, Vertex* sink);

        virtual Vertex* addVertex();

        void deleteEdge(Edge* e);

        void deleteVertex(Vertex* v);

        std::vector<Edge*> addEdges(const std::vector<std::pair<Id_t, Id_t>>& connections);
    };
}
#endif