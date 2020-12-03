#include <LoopsAlgs/Frechet/WeakFrechetOnGraph.h>
#include "movetk/geom/CGALTraits.h"
using namespace LoopsLib;
LoopsAlgs::WeakFrechetOnGraph::Interval::Interval(NT min, NT max): min(min), max(max)
{
}

LoopsAlgs::WeakFrechetOnGraph::Interval::Interval()
{
}

bool LoopsAlgs::WeakFrechetOnGraph::Interval::isEmpty() const
{
    return max < min;
}

LoopsAlgs::WeakFrechetOnGraph::Interval LoopsAlgs::WeakFrechetOnGraph::Interval::intersection(const Interval& other) const
{
    return Interval(std::max(min, other.min), std::min(max, other.max));
}

LoopsAlgs::WeakFrechetOnGraph::Interval& LoopsAlgs::WeakFrechetOnGraph::Interval::intersectWith(const Interval& other)
{
    min = std::max(min, other.min);
    max = std::min(max, other.max);
    return *this;
}

LoopsAlgs::WeakFrechetOnGraph::Interval& LoopsAlgs::WeakFrechetOnGraph::Interval::scale(const NT& val)
{
    min *= val;
    max *= val;
    return *this;
}

LoopsAlgs::WeakFrechetOnGraph::Interval& LoopsAlgs::WeakFrechetOnGraph::Interval::offset(const NT& amount)
{
    min += amount;
    max += amount;
    return *this;
}

LoopsAlgs::WeakFrechetOnGraph::Interval& LoopsAlgs::WeakFrechetOnGraph::Interval::makePositive()
{
    if (max < min) std::swap(max, min);
    return *this;
}

LoopsAlgs::WeakFrechetOnGraph::LeftRightInclusion::LeftRightInclusion()
{
}

LoopsAlgs::WeakFrechetOnGraph::LeftRightInclusion::LeftRightInclusion(bool left, bool right): leftIncluded(left),
                                                                                     rightIncluded(right)
{
}

std::pair<LoopsAlgs::WeakFrechetOnGraph::Interval, LoopsAlgs::WeakFrechetOnGraph::LeftRightInclusion> LoopsAlgs::WeakFrechetOnGraph::
computeIntersectionInterval(const Segment& seg, const Point& p, const NT& eps)
{
    auto v0 = seg[0];
    auto v1 = seg[1];
    // Find projection
    auto dir = v1 - v0;
    auto len = std::sqrt(dir * dir); // TODO May use alternative formulation with squared distances

    auto pointVec = p - seg[0];
    // Projection of point on line through segment
    auto proj = dir * pointVec; // Multiplied by segment length still
    // Projected position
    auto projPos = dir * proj * (1.0 / (dir * dir));
    // Distance from point, squared.
    auto distPointSq = (projPos - pointVec) * (projPos - pointVec);
    // Compute interval on line through seg.
    const auto sqDiff = eps * eps - distPointSq;

    // Difference is smaller than zero: interval is empty.
    if (sqDiff < 0) return std::make_pair(Interval{}, LeftRightInclusion{}); //Empty

    auto diff = std::sqrt(sqDiff);
    Interval covered(proj / len - diff, proj / len + diff);
    // Covered interval out of range.
    if(covered.min > len || covered.max < 0) return std::make_pair(Interval{}, LeftRightInclusion{}); //Empty

    // Intersect with the segment
    auto intersCovered = covered.intersection(Interval(0, len));

    return std::make_pair(
        std::move(intersCovered.scale(1.0 / len)),
        LeftRightInclusion(covered.min <= 0, covered.max >= len)
    );
}

#define FRECHET_LOG_POINTERS(logExpr) if constexpr(LoopsAlgs::FrechetLogging::AssertCorrectPointers){std::cout << logExpr}

LoopsAlgs::WeakFrechetOnGraph::Data::Data(const std::vector<Point>& locations): locations(locations)
{
}

void LoopsAlgs::WeakFrechetOnGraph::Data::setup(DS::BaseGraph* graph)
{
    FDiWhiteIntervals.resize(graph->number_of_vertices(), std::vector<bool>{});

    ComponentMap.resize(graph->number_of_edges(), std::vector<Component>{});
    pathPointers.resize(graph->number_of_vertices(), std::vector<std::pair<DS::BaseGraph::Id_t, Interval>>{});
    for (int i = 0; i < graph->number_of_vertices(); ++i)
    {
        pathPointers[i].resize(polyline.size(), std::make_pair(0L, Interval{}));
    }
}

LoopsAlgs::WeakFrechetOnGraph::Data::Data(const std::vector<Point>& locations, DS::BaseGraph* graph):
    locations(locations)
{
    FDiWhiteIntervals.resize(graph->number_of_vertices(), std::vector<bool>{});
    ComponentMap.resize(graph->number_of_edges(), std::vector<Component>{});
    pathPointers.resize(graph->number_of_vertices(), std::vector<std::pair<DS::BaseGraph::Id_t, Interval>>{});
    for (int i = 0; i < graph->number_of_vertices(); ++i)
    {
        pathPointers[i].resize(polyline.size(), std::make_pair(0L, Interval{}));
    }
}

void LoopsAlgs::WeakFrechetOnGraph::computeFDi(Data& data)
{
    std::cout << "[WeakFrechetOnGraph] Computing FDi" << std::endl;
    data.IntervalInclusions.resize(m_graph->number_of_vertices(), std::vector<LeftRightInclusion>{});
    data.ConnComponentIds.resize(m_graph->number_of_vertices(), std::vector<int>{});
    data.ConnComponentIdToCell.resize(m_graph->number_of_vertices(), {});
    //Foreach vertex in the graph, compute the FD_i strip.
    for (int i = 0; i < m_graph->number_of_vertices(); ++i)
    {
        auto* v = m_graph->vertex(i);

        std::vector<bool>& intervals = data.FDiWhiteIntervals.at(i);
        intervals.resize(data.polyline.size(), false);
        std::vector<LeftRightInclusion>& lrInclusion = data.IntervalInclusions.at(i);
        lrInclusion.resize(data.polyline.size(), LeftRightInclusion{});

        for (int j = 0; j < data.polyline.size(); ++j)
        {
            // Computes the interval in [0,1] that is white.
            auto intervalResult = computeIntersectionInterval(data.polyline[j], data.locations[i], m_epsilon);
            if(!intervalResult.first.isEmpty())
            {
                intervals[j] = true;
            }
            lrInclusion[j] = intervalResult.second;
        }
    }
}

void LoopsAlgs::WeakFrechetOnGraph::computeComponents(Data& data)
{
    std::cout << "[WeakFrechetOnGraph] Computing components" << std::endl;
    DS::BaseGraph::Id_t totalComponentCount = 0;

    for(auto eId = 0; eId < m_graph->number_of_edges(); ++eId)
    {
        auto *e = m_graph->edge(eId);
        const auto v0 = e->m_source->id();
        const auto v1 = e->m_sink->id();
        auto seg = MakeSegment()(data.locations[v0], data.locations[v1]);

        // Count number of open cells at each vertex
        int v0OpenCount = 0;
        int v1OpenCount = 0;
        Component c{-1,-1};
        c.edgeSrc = eId;
        DS::BaseGraph::Id_t currentComponent = 0;
        for(int j = 0; j < data.polyline.size(); ++j)
        {
            // Set first index with non-empty element
            if((data.FDiWhiteIntervals[v0][j] || data.FDiWhiteIntervals[v1][j]) && c.startInterval < 0)
            {
                c.startInterval = j;
            }
            if (data.FDiWhiteIntervals[v0][j]) ++v0OpenCount;
            if (data.FDiWhiteIntervals[v1][j]) ++v1OpenCount;
            if( j == data.polyline.size()-1)
            {
                if (v0OpenCount > 0 && v1OpenCount > 0)
                {
                    c.endInterval = j;
                    c.componentIndex = currentComponent;
                    c.id = totalComponentCount;
                    data.ComponentMap[e->id()].push_back(c);

                    data.ComponentVertToEdge.push_back(eId);

                    ++totalComponentCount;
                    ++currentComponent;
                    c.startInterval = -1;
                }
                v0OpenCount = 0;
                v1OpenCount = 0;
            }
            // Create a component if the both vertex contain some open intervals
            else if(computeIntersectionInterval(seg, data.polyline[j+1][0],m_epsilon).first.isEmpty())
            {
                if(v0OpenCount > 0 && v1OpenCount > 0)
                {
                    c.endInterval = j;
                    c.id = totalComponentCount;
                    c.componentIndex = currentComponent;
                    data.ComponentMap[e->id()].push_back(c);

                    data.ComponentVertToEdge.push_back(eId);

                    ++totalComponentCount;
                    ++currentComponent;
                    c.startInterval = -1;
                }
                v0OpenCount = 0;
                v1OpenCount = 0;
            }
        }
    }

    std::cout << "[WeakFrechetOnGraph] Constructing components graph" << std::endl;
    // Build components graph. Add supersource and supersink
    data.ComponentsGraph.allocateVertices(totalComponentCount + 2);
    // O(n^3)?
    for(DS::BaseGraph::Id_t i = 0; i < m_graph->number_of_vertices(); ++i)
    {
        // Connect source and sinks
        auto* superSource = data.ComponentsGraph.vertex(totalComponentCount);
        auto* superSink = data.ComponentsGraph.vertex(totalComponentCount+1);
        if(data.FDiWhiteIntervals[i][0])
        {
            for(auto* e : m_graph->vertex(i)->m_outEdges)
            {
                if(data.ComponentMap[e->id()].size() > 0 && data.ComponentMap[e->id()][0].startInterval == 0)
                {
                    data.ComponentsGraph.addEdge(superSource->id(), data.ComponentMap[e->id()][0].id);
                }
            }
        }
        if(data.FDiWhiteIntervals[i].back())
        {
            for (auto* e : m_graph->vertex(i)->m_inEdges)
            {
                if (data.ComponentMap[e->id()].size() > 0 && data.ComponentMap[e->id()].back().endInterval == data.polyline.size()-1)
                {
                    data.ComponentsGraph.addEdge(data.ComponentMap[e->id()][0].id, superSink->id());
                }
            }
        }
    }
    std::cout << "[WeakFrechetOnGraph] \t Between component connections" << std::endl;
    // Connect all intermediate components
    for(DS::BaseGraph::Id_t i = 0; i < m_graph->number_of_edges(); ++i)
    {
        auto* e = m_graph->edge(i);
        for(const auto& c: data.ComponentMap[i])
        {
            for(auto* otherE : e->m_sink->m_outEdges)
            {
                for(const auto& otherC: data.ComponentMap[otherE->id()])
                {
                    // Checking interval is strictly smaller than interval: no other component can connect
                    if (c.endInterval < otherC.startInterval) break;
                    // Check if there is overlap.
                    if(!(c.endInterval < otherC.startInterval) && !(c.startInterval > otherC.endInterval))
                    {
                        // If so, check if there is actually a white interval in the overlap.
                        bool connect = false;
                        // Check for actual overlap
                        for(int k = std::max(c.startInterval, otherC.startInterval); k <= std::min(c.endInterval,otherC.endInterval); ++k)
                        {
                            if (!data.FDiWhiteIntervals[e->m_sink->id()][k])continue;

                            connect = true;
                            break;
                        }
                        if(connect)
                        {
                            data.ComponentsGraph.addEdge(c.id, otherC.id);
                        }
                    }
                }
            }
        }
    }
    std::cout << "[WeakFrechetOnGraph] -- Constructing components done" << std::endl;
}

template<typename T>
bool approxGreaterEqual(const T& first, const T& second, const T& thresh = 0.000000001)
{
    auto diff = std::abs(first - second);
    if (diff < thresh) return true;
    return first >= second;
}

#define FRECHET_LOG_DP(logCode) if constexpr(LoopsAlgs::WeakFrechetLogging::LogDP) { logCode; }


bool LoopsAlgs::WeakFrechetOnGraph::computeBastardFrechetPath(Data& data, std::vector<DS::BaseGraph::Edge*>& path)
{
    // Idea here is to just go forward in the graph, but allow 'up' 'down' motion along matched polyline.
    // Not a great way to measure things, but computable for now.
    std::cout << "[WeakFrechetOnGraph] Computing bastard Frechet" << std::endl;
    using Vert = DS::BaseGraph::Vertex;
    struct StackNode
    {
        Vert* src;
        decltype(src->m_outEdges.begin()) currEdge;
    };
    std::set<DS::BaseGraph::Id_t> seenVerts;

    std::stack<StackNode> processStack;
    // Source and sink that ARE NOT PART of the original graph.
    auto* superSource = data.ComponentsGraph.vertex(data.ComponentsGraph.number_of_vertices() - 2);
    auto* superSink = data.ComponentsGraph.vertex(data.ComponentsGraph.number_of_vertices() - 1);
    processStack.push(StackNode{ superSource, superSource->m_outEdges.begin() });
    seenVerts.insert(superSource->id());
    while(!processStack.empty())
    {
        auto el = processStack.top();
        if(el.src == superSink)
        {
            break;
        }
        if(el.currEdge == el.src->m_outEdges.end())
        {
            processStack.pop();
            // Increment previous edge iterator.
            ++processStack.top().currEdge;
            continue;
        }
        auto* e = *el.currEdge;
        if(seenVerts.find(e->m_sink->id()) == seenVerts.end())
        {
            processStack.push(StackNode{ e->m_sink, e->m_sink->m_outEdges.begin() });
            seenVerts.insert(e->m_sink->id());
        }
        else
        {
            ++processStack.top().currEdge;
        }
    }
    std::cout << "[WeakFrechetOnGraph] Computation done" << std::endl;
    // If the stack is empty, no path was found.
    if (processStack.empty()) return false;
    
    // Reconstruct the path from the stack
    // Note: potentially non-simple.
    processStack.pop(); //Top most (supersink) element does not contribute.

    // Note: the we need the original edges!
    while(processStack.size()!= 1)
    {
        auto* componentGraphVert = processStack.top().src;// Corresponds to edge in original
        //data.ComponentVertToEdge.push_back(eId);
        path.push_back(m_graph->edge(data.ComponentVertToEdge[componentGraphVert->id()]));
        processStack.pop();
    }
    // We reconstructed in reverse order, so fix that.
    std::reverse(path.begin(), path.end());
}

bool LoopsAlgs::WeakFrechetOnGraph::applyDynamicProgram(Data& data, DS::BaseGraph::Id_t& endVert, int& endInterval)
{
    std::cout << "[WeakFrechetOnGraph] Running search" << std::endl;

    struct StackNode
    {
        DS::BaseGraph::Vertex* vertex;
        decltype(m_graph->vertex(0)->m_outEdges.begin()) currEdgeIt; //The iterator to the edge to use next
        // Component that we came from.
        Component srcComponent;
        // Component to search in in the edge
        int component;
    };
    // Try all potential starting points
    for(DS::BaseGraph::Id_t i = 0; i < m_graph->number_of_vertices(); ++i)
    {
        // Not a potential starting point: continue
        if (!data.IntervalInclusions[i][0].leftIncluded) continue;

        // The DFS-like path that we currently have.
        std::stack<StackNode> currentPath;
        currentPath.push(StackNode{ m_graph->vertex(i), m_graph->vertex(i)->m_outEdges.begin(), Component{0,0},0 });

        bool foundEnd = false;
        // Keep on searching
        while(!currentPath.empty())
        {
            //We need to find out whether a feasible path exists.
            auto state = currentPath.top();
            if(state.currEdgeIt == state.vertex->m_outEdges.end())
            {
                // TODO Done? or are we going to try switching component?
                currentPath.pop();
                // Go to next edge of previous part.
                ++currentPath.top().currEdgeIt;
                continue;
            }
            auto* e = *state.currEdgeIt;
            // Find on the target edge a connected component, using the current element (cell) to check if it indeed exists.
            const auto& components = data.ComponentMap[e->id()];
            bool foundNewActiveComponent = false;
            for(int i = state.component; i < data.ComponentMap[e->id()].size(); ++i)
            {
                // No overlap
                if((components[i].startInterval >= state.srcComponent.endInterval) || (components[i].endInterval <= state.srcComponent.startInterval))
                {
                    continue;
                }
                for (auto cell = state.srcComponent.startInterval; cell < state.srcComponent.endInterval; ++cell)
                {
                    // Ignore cells on the source that are not open
                    if (!data.FDiWhiteIntervals[e->m_source->id()][cell]) continue;

                    // Push new edge to explore
                    currentPath.push(StackNode{ e->m_sink, e->m_sink->m_outEdges.begin(), components[i], 0 });
                    foundNewActiveComponent = true;
                    break;
                }
                if (foundNewActiveComponent)break;
            }
            
            // Go to the next edge.
            if(!foundNewActiveComponent)
            {
                ++currentPath.top().currEdgeIt;
            }

        }
    }

    return false;
}

void LoopsAlgs::WeakFrechetOnGraph::reconstructPath(Data& data, DS::BaseGraph::Id_t vId, int endInterval,
                                           std::vector<DS::BaseGraph::Id_t>& path)
{
    std::cout << "[WeakFrechetOnGraph] Reconstructing path" << std::endl;
    // Follow the pathpointers backwards until we reach L_k.
    // We are not interested in the matching yet, so just reconstruct the path.
    
    assert(data.IntervalInclusions[vId].back().rightIncluded);

    // Build in reverse order
    path.push_back(vId);
    int currentV = vId;
    int currentInter = endInterval;
    while (true)
    {
        std::cout << "[WeakFrechetOnGraph] \t Current vertex: " << currentV << std::endl;
        auto next = data.pathPointers[currentV][currentInter];
        path.push_back(next.first);
        // Find next interval
        currentV = next.first;
        auto nextInterIndex = -1;// data.IntervalIds[currentV][(int)next.second.min];
        currentInter = nextInterIndex;
        
        // Check if leftmost point is white: we are done.
        if (data.IntervalInclusions[currentV][0].leftIncluded)
        {
            break;
        }
    }
    // Orient the path in the proper direction
    std::reverse(path.begin(), path.end());
}

void LoopsAlgs::WeakFrechetOnGraph::pruneViaWeakConnectivity(Data& data)
{

}

bool LoopsAlgs::WeakFrechetOnGraph::hasPotentialSolution(Data& data)
{
    std::set<DS::BaseGraph::Id_t> seenVerts;
    auto wasSeen = [&seenVerts](DS::BaseGraph::Vertex* v)
    {
        return seenVerts.find(*v) != seenVerts.end();
    };
    // Run BFS on everything
    std::queue< DS::BaseGraph::Vertex*> nodeToProcess;
    // Push supersource
    nodeToProcess.push(data.ComponentsGraph.vertex(data.ComponentsGraph.number_of_vertices() - 2));
    auto* superSink = data.ComponentsGraph.vertex(data.ComponentsGraph.number_of_vertices() - 1);
    while(!nodeToProcess.empty())
    {
        auto* v = nodeToProcess.front();
        nodeToProcess.pop();
        for(auto* e : v->m_outEdges)
        {
            if (e->m_sink == superSink) return true;
            if(!wasSeen(e->m_sink))
            {
                seenVerts.insert(*e->m_sink);
                nodeToProcess.push(e->m_sink);
            }
        }
        for (auto* e : v->m_inEdges)
        {
            if (e->m_source == superSink) return true;
            if (!wasSeen(e->m_source))
            {
                seenVerts.insert(*e->m_source);
                nodeToProcess.push(e->m_source);
            }
        }
    }
    return false;
}

LoopsAlgs::WeakFrechetOnGraph::WeakFrechetOnGraph(DS::BaseGraph* graph): m_graph(graph)
{
}

void LoopsAlgs::WeakFrechetOnGraph::setTargetGraph(DS::BaseGraph* graph)
{
    m_graph = graph;
}

void LoopsAlgs::WeakFrechetOnGraph::compute(const std::vector<DS::BaseGraph::Id_t>& path, const std::vector<Point>& locations,
                                   double epsilon, std::vector<DS::BaseGraph::Id_t>& outputPath)
{

    std::cout << "[WeakFrechetOnGraph] Path size: " << path.size() << '\n';
    std::cout << "[WeakFrechetOnGraph] Path nodes: ";
    for(auto el: path)
    {
        std::cout << el << ' ';
    }
    std::cout << '\n';
    m_epsilon = epsilon;
    Data data(locations);
    // Construct polyline from points
    for (auto it = path.begin(), it2 = path.begin() + 1; it2 != path.end(); ++it, ++it2)
    {
        Segment s = MakeSegment()(locations[*it], locations[*it2]);
        auto v0 = s[0];
        auto v1 = s[1];
        data.polyline.push_back(s);
        v0 = data.polyline.back()[0];
        v1 = data.polyline.back()[1];
        std::cout << "Distance seg "<< (it-path.begin()) <<": " << *it << "-" << *it2 << ":" << std::sqrt(s.get().squared_length()) << std::endl;
    }
    data.setup(m_graph);


    // Compute 1D Frechet surface per vertex
    std::vector<std::vector<Interval>> vertexSurface;

    // Compute FDi elements and store in the data object
    computeFDi(data);

    computeComponents(data);
        

    // Run dynamic programming to find a feasible path.
    DS::BaseGraph::Id_t endVert = -1;
    int endInterval = -1;
    // Succesfully found a path
    std::vector<DS::BaseGraph::Edge*> outPath;
    if(computeBastardFrechetPath(data, outPath))
    //if (applyDynamicProgram(data, endVert, endInterval))
    {
        //reconstructPath(data, endVert, endInterval, outputPath);
        // Output path given as vertex indices
        outputPath.push_back(outPath[0]->m_source->id());
        for(auto* e : outPath)
        {
            outputPath.push_back(e->m_sink->id());
        }
    }
    else
    {
        std::cout << "No path found" << std::endl;
    }
}
