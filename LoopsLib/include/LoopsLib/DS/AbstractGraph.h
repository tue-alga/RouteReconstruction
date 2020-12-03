#ifndef ABSTRACTGRAPH_H
#define ABSTRACTGRAPH_H
#include <LoopsLib/DS/OwnGraph.h>
namespace LoopsLib::DS
{
    template<typename VertexData, typename EdgeData>
    struct TypedComponents
    {
        // Forward declare
        struct Vertex;

        struct Edge
        {
            EdgeData m_data;

            // The edge ID
            long long m_id;
            // Source and sink vertices
            Vertex* m_source = nullptr, *m_sink = nullptr;

            EdgeData& data()
            {
                return m_data;
            }
            const EdgeData& data() const
            {
                return m_data;
            }

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
            bool operator()(Edge* e0, Edge* e1) const
            {
                return static_cast<long long>(*e0->m_sink) < static_cast<long long>(*e1->m_sink);
            }
        };
        struct EdgeSortSource
        {
            bool operator()(Edge* e0, Edge* e1) const
            {
                return static_cast<long long>(*e0->m_source) < static_cast<long long>(*e1->m_source);
            }
        };
        struct Vertex
        {
            VertexData m_data;

            VertexData& data()
            {
                return m_data;
            }
            const VertexData& data() const
            {
                return m_data;
            }
            // ID of the vertex
            long long m_id;
            // Outgoing edges
            std::set<Edge*, EdgeSortSink> m_outEdges;
            // Incoming edges
            std::set<Edge*, EdgeSortSource> m_inEdges;

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

            Edge* findOutEdge(Vertex* sink);

            Edge* findInEdge(Vertex* source);

            /**
             * \brief Check if the vertex is connected to the given vertex
             * \param otherVertex Other vertex, identified by its ID
             * \return Whether or not the vertex is connected to the given vertex.
             */
            bool connectedTo(long long otherVertex) const;
        };
    };

    template<typename VertexType, typename EdgeType>
    class TypedGraph
    {
    public:
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
            bool operator()(Edge* e0, Edge* e1) const
            {
                return static_cast<long long>(*e0->m_sink) < static_cast<long long>(*e1->m_sink);
            }
        };
        struct EdgeSortSource
        {
            bool operator()(Edge* e0, Edge* e1) const
            {
                return static_cast<long long>(*e0->m_source) < static_cast<long long>(*e1->m_source);
            }
        };
        struct Vertex
        {
            // ID of the vertex
            long long m_id;
            // Outgoing edges
            std::set<Edge*, EdgeSortSink> m_outEdges;
            // Incoming edges
            std::set<Edge*, EdgeSortSource> m_inEdges;

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

            Edge* findOutEdge(Vertex* sink);

            Edge* findInEdge(Vertex* source);

            /**
             * \brief Check if the vertex is connected to the given vertex
             * \param otherVertex Other vertex, identified by its ID
             * \return Whether or not the vertex is connected to the given vertex.
             */
            bool connectedTo(long long otherVertex) const;
        };

    protected:
        // The vertices in the graph
        std::vector<Vertex*> m_vertices;
        // The edges in the graph
        std::vector<Edge*> m_edges;
    public:
        using Id_t = long long;
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

        BaseGraph(long long numberOfVertices);

        BaseGraph(long long numberOfVertices, long long numberOfEdges);

        const std::vector<Vertex*>& vertices() const;

        const std::vector<Edge*>& edges() const;

        std::size_t number_of_vertices() const;

        std::size_t number_of_edges() const;

        Edge* edge(long long id);

        Vertex* vertex(long long id);

        /**
         * \brief Get the constant version of a vertex
         * \param id The ID of the vertex
         * \return The vertex
         */
        const Vertex* vertex(long long id) const;

        virtual Edge* addEdge(long long source, long long sink);

        virtual Edge* addEdge(Vertex* source, Vertex* sink);

        virtual Vertex* addVertex();

        void deleteEdge(Edge* e);

        void deleteVertex(Vertex* v);

        std::vector<Edge*> addEdges(const std::vector<std::pair<Id_t, Id_t>>& connections);
    };
}
#endif