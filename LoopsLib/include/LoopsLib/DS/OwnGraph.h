#ifndef DS_OWNGRAPH_H
#define DS_OWNGRAPH_H
#include <vector>
#include <set>
#include <queue>
#include <stack>
#include <Eigen/Eigen>
#include <unordered_set>
#include <LoopsLib/Helpers/Iterators.h>
#include <LoopsLib/DS/BaseGraph.h>


namespace LoopsLib::DS
{
    // Default for edge and vertex data when no extra data is needed
    struct EmptyStruct {  };

    template<typename Type>
    struct GetId{
        auto operator()(Type t){
            if constexpr(std::is_pointer_v<Type>){
                return t->id();
            }
            return t.id();
        }
    };

    class AbstractGraph
    {
    public:
        // Forward declare
        class Vertex;

        class Edge
        {
        protected:
            // The edge ID
            long long m_id;
        public:
            virtual ~Edge() = default;
            virtual Vertex* source() = 0;
            virtual Vertex* sink() = 0;
            virtual const Vertex* source() const = 0;
            virtual const Vertex* sink() const = 0;

            /**
             * \brief Retrieve the ID of the edge
             * \return The ID of the edge
             */
            long long id() const { return m_id; }

            /**
             * \brief Constructs a new edge
             * \param id The unique ID of the edge
             */
            explicit Edge(long long id):m_id(id){}

            /**
             * \brief Implicitly converts to the edge ID
             */
            operator long long() const { return m_id; }

            /**
             * \brief Returns the a pointer to the source and sink vertices of the edges,
             * given as an std::pair
             * \return The pair of vertices
             */
            virtual std::pair<Vertex*, Vertex*> verts() const = 0;

            /**
             * \brief Returns whether this edge has the vertex, identified by the given ID
             * as sink
             * \param id The vertex ID
             * \return Whether the sink of this edge is the given vertex
             */
            bool hasSink(long long id) const
            {
                return sink()->id() == id;
            }

            /**
             * \brief Returns whether this edge has the vertex, identified by the given ID
             * as source
             * \param id The vertex ID
             * \return Whether the source of this edge is the given vertex
             */
            bool hasSource(long long id) const
            {
                return source()->id() == id;
            }

            /**
             * \brief Detaches the edge from its vertices
             */
            virtual void detach(bool nullSourceAndSink = true) = 0;

            /**
             * \brief Reconnects to the assigned source and sink node. Assumes both are not null
             */
            virtual void reconnect() = 0;

            /**
             * \brief Flips the direction of the edge.
             * Effectively swaps the source and sink vertex
             * and corrects the connections in the vertices.
             */
            void flip();
        };
        class Vertex
        {
        protected:
            // ID of the vertex
            long long m_id;
        public:
            /**
             * \brief Returns the vertex ID
             * \return The id
             */
            long long id() const { return m_id; }

            /**
             * \brief Construts a vertex
             * \param id The unique identifier for this vertex
             */
            explicit Vertex(long long id):m_id(id){}

            /**
             * \brief Can implicitly convert the vertex to its ID value for ease of use.
             */
            operator long long() const { return m_id; }

            virtual Edge* findOutEdge(Vertex* sink) = 0;

            virtual Edge* findInEdge(Vertex* source) = 0;

            /**
             * \brief Check if the vertex is connected to the given vertex
             * \param otherVertex Other vertex, identified by its ID
             * \return Whether or not the vertex is connected to the given vertex.
             */
            virtual bool connectedTo(long long otherVertex) const = 0;
        };

    public:
        using Id_t = long long;
        virtual ~AbstractGraph();

        /**
         * \brief Empty constructor. Allocate vertices afterwards with
         * allocateVertices()
         */
        AbstractGraph() = default;

        /**
         * \brief Clears the graph. All vertices and edges will be deleted
         */
        void clear();

        /**
         * \brief Allocates the given number of vertices. These can then be accessed via
         * vertex() with the IDs up to numberOfVertices-1
         * \param numberOfVertices The number of vertices to allocate
         */
        virtual void allocateVertices(long long numberOfVertices) = 0;

        AbstractGraph(long long numberOfVertices);

        AbstractGraph(long long numberOfVertices, long long numberOfEdges);

        const std::vector<Vertex*>& vertices() const;

        const std::vector<Edge*>& edges() const;

        std::size_t number_of_vertices() const;

        std::size_t number_of_edges() const;

        bool isSimplePath(const std::vector<Id_t>& vertices) const;

        bool isSimpleEdgePath(const Eigen::VectorXd& edges) const;

        bool isSimpleVertPath(const Eigen::VectorXd& verts) const;

        bool isSimplePath(const std::vector<Edge*>& edges) const;

        bool isSimplePath(const std::vector<Vertex*>& vertices) const;

        virtual Edge* edge(long long id) = 0;

        virtual Vertex* vertex(long long id) = 0;

        virtual const Edge* edge(long long id) const = 0;
        /**
         * \brief Get the constant version of a vertex
         * \param id The ID of the vertex
         * \return The vertex
         */
        virtual const Vertex* vertex(long long id) const = 0;

        virtual Edge* addEdge(long long source, long long sink);

        virtual Edge* addEdge(Vertex* source, Vertex* sink);

        virtual Vertex* addVertex();

        void deleteEdge(Edge* e);

        void deleteVertex(Vertex* v);

        std::vector<Edge*> addEdges(const std::vector<std::pair<Id_t, Id_t>>& connections);
    };


    template<typename VertexData, typename EdgeData>
    class GraphWithData : public AbstractGraph
    {
    public:
        class DataVert;
        class DataEdge : public AbstractGraph::Edge
        {
            EdgeData m_data;
            DataVert* m_source = nullptr, m_sink = nullptr;
        public:
            DataEdge(long long id, const EdgeData& data)
                : AbstractGraph::Edge(id),
                  m_data(data)
            {
            }

            
            EdgeData& data()
            {
                return m_data;
            }
            const EdgeData& data() const
            {
                return m_data;
            }

            Vertex* source() override
            {
                return m_source;
            }
            Vertex* sink() override
            {
                return m_sink;
            }
            std::pair<Vertex*, Vertex*> verts() const override
            {
                return std::make_pair(m_source, m_sink);
            }
            void detach(bool nullSourceAndSink) override;
            void reconnect() override;
        };
        class DataVert : public AbstractGraph::Vertex
        {
        public:
            Edge* findOutEdge(Vertex* sink) override
            {
                return m_outEdges.at(sink->id());
            }
            Edge* findInEdge(Vertex* source) override
            {
                return m_inEdges.at(source->id());
            }
            bool connectedTo(long long otherVertex) const override
            {
                return m_inEdges.find(otherVertex) != m_inEdges.end() ||
                    m_outEdges.find(otherVertex) != m_outEdges.end();
            }
        private:
            VertexData m_data;
            std::map<Id_t, DataEdge*> m_outEdges;
            std::map<Id_t, DataEdge*> m_inEdges;
        };


        void allocateVertices(long long numberOfVertices) override
        {
            const auto diff = numberOfVertices - m_verts.size();
            const auto initSize = m_verts.size();
            for(auto i =0; i < diff; i++)
            {
                m_verts.push_back(new DataVert(initSize+i));
            }
        }
        Edge* edge(long long id) override
        {
            return m_edges[id];
        }
        Vertex* vertex(long long id) override
        {
            return m_verts[id];
        }
        const Edge* edge(long long id) const override
        {
            return m_edges[id];
        }
        const Vertex* vertex(long long id) const override
        {
            return m_verts[id];
        }
    protected:
        std::vector<DataVert*> m_verts;
        std::vector<DataVert*> m_edges;
    };
    
    /**
     * \brief Checks if the source is connected to the sink.
     * \param graph The graph
     * \param source The source ID
     * \param sink The sink ID
     * \return Whether or not the source is connected to the sink.
     */
    bool isConnected(DS::BaseGraph* graph, long long source, long long sink);

    /**
     * \brief Check if the DAG, constructed on the graph via the given mapping, is connected from source to sink.
     * The mapping specifies the index in the order of the vertex with a given ID.
     * \param graph The underlying graph
     * \param dagMapping The DAG mapping
     * \param source The source node
     * \param sink The sink node
     * \return Is the source connected with the sink in the DAG?
     */
    bool isDagConnected(BaseGraph* graph, const std::vector<int>& dagMapping, long long source, long long sink);

    /**
     * \brief Holder for graph components with extra data
     * \tparam VertexData Data to associate with vertices
     * \tparam EdgeData Data to associate with edges
     */
    template<typename VertexData = EmptyStruct, typename EdgeData = EmptyStruct>
    struct GraphComponents
    {
        struct Edge;
        struct Vertex
        {
            // ID of the vertex
            long long m_id;
            // Outgoing edges
            std::set<Edge*> m_outEdges;
            // Incoming edges
            std::set<Edge*> m_inEdges;
            // Associated data
            VertexData m_data;

            /**
             * \brief Returns the vertex ID
             * \return The id
             */
            long long id() const
            {
                return m_id;
            }
            Vertex(long long id):m_id(id){}
            Vertex(long long id, const VertexData& data):m_id(id),m_data(data){}

            const VertexData& data() const
            {
                return m_data;
            }
            VertexData& data()
            {
                return m_data;
            }

            /**
             * \brief Check if the vertex is connected to the given vertex
             * \param otherVertex Other vertex, identified by its ID
             * \return Whether or not the vertex is connected to the given vertex.
             */
            bool connectedTo(long long otherVertex) const
            {
                for(auto e : m_outEdges)
                {
                    if (e->m_sink.id() == otherVertex)return true;
                }
                for (auto e : m_inEdges)
                {
                    if (e->m_source.id() == otherVertex)return true;
                }
                return false;
            }
        };

        struct Edge
        {
            long long m_id;
            Vertex* m_source = nullptr, *m_sink = nullptr;
            EdgeData m_data;
            long long id() const
            {
                return m_id;
            }
            explicit Edge(long long id) :m_id(id) {}
            Edge(long long id, const EdgeData& data) :m_id(id), m_data(data) {}

            /**
             * \brief Returns the a pointer to the source and sink vertices of the edges,
             * given as an std::pair
             * \return The pair of vertices
             */
            std::pair<Vertex*, Vertex*> verts() const
            {
                return std::make_pair(m_source, m_sink);
            }

            /**
             * \brief Detaches the edge from its vertices
             */
            void detach()
            {
                m_sink->m_inEdges.erase(this);
                m_source->m_outEdges.erase(this);
                m_source = nullptr;
                m_sink = nullptr;
            }

            /**
             * \brief Flips the direction of the edge.
             * Effectively swaps the source and sink vertex
             * and corrects the connections in the vertices.
             */
            void flip()
            {
                m_source->m_outEdges.erase(this);
                m_source->m_inEdges.insert(this);
                m_sink->m_inEdges.erase(this);
                m_sink->m_outEdges.insert(this);
                std::swap(m_source, m_sink);
            }
        };
    };
    template<typename EdgeData, typename VertexData>
    class OwnGraph
    {
    public:
        using Edge = typename GraphComponents<EdgeData, VertexData>::Edge;
        using Vertex = typename GraphComponents<EdgeData, VertexData>::Vertex;
        using Id_t = long long;
    private:
        // The vertices in the graph
        std::vector<Vertex*> m_vertices;
        // The edges in the graph
        std::vector<Edge*> m_edges;
    public:
        ~OwnGraph()
        {
            for(auto el: m_vertices)
            {
                delete el;
            }
            for(auto el: m_edges)
            {
                delete el;
            }
        }
        OwnGraph(){}
        OwnGraph(long long numberOfVertices)
        {
            // TODO placement new for more aligned data
            for(auto i = 0; i < numberOfVertices; ++i)
            {
                m_vertices.push_back(new Vertex(i));
            }
        }
        OwnGraph(long long numberOfVertices, long long numberOfEdges)
        {
            // TODO placement new for more aligned data
            for (auto i = 0; i < numberOfVertices; ++i)
            {
                m_vertices.push_back(new Vertex(i));
            }
        }
        /*OwnGraph(Id_t numberOfVertices, const std::vector<std::pair<Id_t, Id_t>>& edges,
            const std::vector<VertexData>& vertData, const std::vector<EdgeData>& edgeData):
        {
            for (auto i = 0; i < numberOfVertices; ++i)
            {
                m_vertices.push_back(new Vertex(i));
                m_vertices.back()->m_data = vertData[i];
            }
        }*/
        const std::vector<Vertex*>& vertices() const
        {
            return m_vertices;
        }
        const std::vector<Edge*>& edges() const 
        {
            return m_edges;
        }

        std::size_t number_of_vertics() const
        {
            return m_vertices.size();
        }
        std::size_t number_of_edges() const
        {
            return m_edges.size();
        }
        Edge* edge(long long id)
        {
            return m_edges[id];
        }
        Vertex* vertex(long long id)
        {
            return m_vertices[id];
        }
        Edge* addEdge(long long source, long long sink)
        {
            auto* edge = new Edge(m_edges.size());
            m_edges.push_back(edge);
            edge->m_source = m_vertices[source];
            edge->m_sink = m_vertices[sink];
            edge->m_source->m_outEdges.push_back(edge);
            edge->m_sink->m_inEdges.push_back(edge);
            return edge;
        }
        Edge* addEdge(Vertex* source, Vertex* sink)
        {
            auto* edge = new Edge(m_edges.size());
            m_edges.push_back(edge);
            edge->m_source = source;
            edge->m_sink = sink;
            source->m_outEdges.push_back(edge);
            sink->m_inEdges.push_back(edge);
            return edge;
        }
        Vertex* addVertex()
        {
            auto* vert = new Vertex(m_vertices.size());
            m_vertices.push_back(vert);
            return vert;
        }
        void deleteEdge(Edge* e)
        {
            e->detach();
            std::swap(m_edges[e->id()].m_id, m_edges.back().m_id);
            std::swap(m_edges[e->id()], m_edges.back());
            delete m_edges.back();
            m_edges.resize(m_edges.size() - 1);
        }
        std::vector<Edge*> addEdges(const std::vector<std::pair<Id_t,Id_t>>& connections)
        {
            std::vector<Edge*> out;
            for(auto conn : connections)
            {
                out.push_back(addEdge(conn.first, conn.second));
            }
            return out;
        }
    };

    class GraphTraversal
    {
    public:
        // Result to be returned by callback
        enum class CbResult
        {
            StopIteration,
            Continue
        };
        template<typename GraphType, typename Func>
        void bfs(GraphType& graph, long long start, Func callback)
        {
            std::queue<long long> q;
            q.push(start);
            std::set<long long> visited;
            visited.insert(start);
            while(!q.empty())
            {
                auto id = q.front();
                q.pop();
                for(auto* e : graph.vertex(id)->m_outEdges)
                {
                    if(callback(e, graph.vertex(id)) == CbResult::StopIteration)
                    {
                        return;
                    }
                    // Add vert to queue
                    if(visited.find(e->m_sink->id()) == visited.end())
                    {
                        q.push(e->m_sink->id());
                        visited.insert(id); //Avoid duplicate entries in the queue.
                    }
                }
            }
        }
        template<typename GraphType, typename Func>
        void dfs(GraphType& graph, long long start, Func callback)
        {
            std::stack<long long> q;
            q.push(start);
            std::set<long long> visited;
            visited.insert(start);
            while (!q.empty())
            {
                auto id = q.top();
                q.pop();
                for (auto* e : graph.vertex(id)->m_outEdges)
                {
                    if (callback(e, graph.vertex(id)) == CbResult::StopIteration)
                    {
                        return;
                    }
                    // Add vert to queue
                    if (visited.find(e->m_sink->id()) == visited.end())
                    {
                        q.push(e->m_sink->id());
                        visited.insert(id); //Avoid duplicate entries in the queue.
                    }
                }
            }
        }
    };

    template<typename EdgeData, typename VertexData>
    class Dijkstra
    {
    public:
        using Edge = typename OwnGraph<EdgeData, VertexData>::Edge;
        using Vertex = typename OwnGraph<EdgeData, VertexData>::Vertex;
        struct Dist
        {
            double val = std::numeric_limits<double>::max();
        };

        struct ShortestPathRule
        {
            struct VertData
            {
                double val = std::numeric_limits<double>::max();
                bool operator<(const VertData& other) const
                {
                    return val < other.val;
                }
            };
            using Tree = OwnGraph<EmptyStruct, VertData>;
            using TreeVertex = typename Tree::Vertex;
            using TreeEdge = typename Tree::Edge;

            void initializeSource(TreeVertex* source) const
            {
                source->m_data.val = 0;
            }

            std::vector<double> m_weights;
            ShortestPathRule(const std::vector<double>& weights):m_weights(weights){}

            bool operator()(TreeVertex* v0, Edge* e0, TreeVertex* v1)
            {
                // Update distance if weight is better
                if (v0->m_data.val + m_weights[e0->id()] < v1->m_data.val)
                {
                    v1->m_data.val = v0->m_data.val + m_weights[e0->id()];
                    return true;
                }
                return false;
            }
        };
        struct WidestPathRule
        {
            struct VertData
            {
                double val = -std::numeric_limits<double>::max();
                bool operator<(const VertData& other) const
                {
                    return val < other.val;
                }
            };
            using Tree = OwnGraph<EmptyStruct, VertData>;
            using TreeVertex = typename Tree::Vertex;
            using TreeEdge = typename Tree::Edge;
            
            void initializeSource(TreeVertex* source) const
            {
                source->m_data.val = std::numeric_limits<double>::max();;
            }

            std::vector<double> m_weights;
            WidestPathRule(const std::vector<double>& weights) :m_weights(weights) {}

            bool operator()(TreeVertex* v0, Edge* e0, TreeVertex* v1)
            {
                // Update distance if weight is better
                if (std::min(v0->m_data.val,m_weights[e0->id()]) > v1->m_data.val)
                {
                    v1->m_data.val = std::min(v0->m_data.val, m_weights[e0->id()]);
                    return true;
                }
                return false;
            }
        };

        template<typename Rule>
        std::vector<typename OwnGraph<EdgeData,VertexData>::Edge*> operator()(OwnGraph<EdgeData, VertexData>* graph,
            long long source,
            long long sink,
            Rule rule)
        {
            using Tree = typename Rule::Tree;
            using TreeVert = typename Tree::Vertex;
            // The distance tree
            Tree distTree(graph->number_of_vertics());

            // Get the source and initialize it
            auto* srcVert = distTree.vertex(source);
            rule.initializeSource(srcVert);

            // Prio queue
            struct Compare
            {
                bool operator()(TreeVert* v1, TreeVert* v2)
                {
                    return v1->m_data < v2->m_data || v1->id() < v2->id();
                }
            };
            std::set<TreeVert*, Compare> q;

            std::unordered_set<long long> visited;
            auto wasVisited = [&visited](TreeVert* vert)
            {
                return visited.find(vert->id()) != visited.end();
            };
            q.insert(srcVert);
            visited.insert(srcVert->id());

            // Peform BFS
            while(!q.empty())
            {
                // Pop next vert and mark visited
                auto el = *q.begin();
                visited.insert(el->id());
                q.erase(q.begin());
                // Go over all adjacent vertices
                for(auto e : graph->vertex(el->id())->m_outEdges)
                {
                    auto vs = e->verts();
                    auto treeStartVert = distTree.vertex(el->id());
                    auto treeEndVert= distTree.vertex(e->m_sink->id());

                    // If the rule requires update
                    if(rule(treeStartVert, e, treeEndVert))
                    {
                        if (treeEndVert->m_inEdges.size() > 0) distTree.deleteEdge(treeEndVert->m_inEdges[0]);
                        // Create the new edge
                        distTree.addEdge(treeStartVert, treeEndVert);
                        // Update location in prio queue
                        if(q.find(treeEndVert) != q.end())
                        {
                            q.erase(treeEndVert);
                            q.insert(treeEndVert);
                        }
                    }
                    // Only add the end vert if it wasn't visited yet.
                    if(!wasVisited(treeEndVert))
                    {
                        q.insert(treeEndVert);
                    }
                }
            }
            // Dist tree now contains the distances.
            auto* sinkVert = distTree.vertex(sink);
            // Follow to root
            auto* curr = sinkVert;
            std::vector<Edge*> output;
            while(curr != srcVert)
            {
                auto* origEnd = graph->vertex(curr->id());
                auto* pred = graph->vertex(curr->m_inEdges[0]->m_source->id());
                for(auto* e : origEnd->m_inEdges)
                {
                    if(e->m_source == pred)
                    {
                        output.push_back(e);
                        curr = distTree.vertex(e->m_source->id());
                    }
                    break;
                }
            }
            std::reverse(output.begin(), output.end());
            return output;
        }
    };
}
#endif