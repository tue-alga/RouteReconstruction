#include <LoopsLib/Algs/LongestPath/k_SGL.h>
#include <unordered_set>
#include <iostream>
using namespace LoopsLib;
using namespace LoopsLib::Algs;
using namespace LoopsLib::Algs::LongestPath;

k_SGL::k_SGL(DS::BaseGraph* graph): ILongestPath("kSGL", graph),
                                                                             m_k(3)
{
}

bool k_SGL::bruteForceBestKPath(int source, std::vector<Edge*>& bestPath, double maxWeight)
{
    struct State
    {
        Vertex* vertex = nullptr;
        bool discovered = false;
        // Iterators over out edges of vertex
        decltype(vertex->m_outEdges.begin()) currIt;
        decltype(vertex->m_outEdges.begin()) endIt;
        double weightThusfar = 0;;

        State(Vertex* vertex, double weight = 0):
            vertex(vertex),
            currIt(vertex->m_outEdges.begin()),
            endIt(vertex->m_outEdges.end()),
            weightThusfar(weight)
        {
        }
    };
    bestPath = std::vector<Edge*>(m_k, nullptr);
    double bestWeight = -std::numeric_limits<double>::max();
    std::vector<Edge*> currentPath(m_k, nullptr);
    std::stack<State> stack;
    // Search starting point
    stack.push(State(m_graph->vertex(source)));
    m_forbidden(source) = 1;

    while (!stack.empty())
    {
        // Always pop state
        auto state = stack.top();
        stack.pop();

        const auto& neighs = state.vertex->m_outEdges;

        // Where to place a newly found edge in the local best path
        const int pathIndexTarget = stack.size();
        // Are the nodes we are going to consider end nodes?
        const bool isEndLength = pathIndexTarget == m_k - 1;

        // The current element is considered the end of a local path
        if (isEndLength)
        {
            const double weightDiff = maxWeight - state.weightThusfar;
            // Find highest weight element
            auto maxElIt = std::max_element(state.currIt, state.endIt, [this, weightDiff](Edge* n0, Edge* n1)
            {
                if(m_weights[*n0] > weightDiff)
                {
                    if(m_weights[*n1] > weightDiff)
                    {
                        return *n0->m_sink < *n1->m_sink;
                    }
                    return true;
                }
                if(m_weights[*n1] > weightDiff)
                {
                    return false;
                }

                // Make forbidden nodes appear as low nodes.
                if (m_forbidden(*n1->m_sink))
                {
                    if (m_forbidden(*n0->m_sink)) return *n0->m_sink < *n1->m_sink;
                    return false;
                }
                if (*n1->m_sink == m_sink) return true;
                if (*n0->m_sink == m_sink) return false;
                return m_weights[*n0] < m_weights[*n1];
            });
            Edge* maxEdge = *maxElIt;
            // No edge is allowed, so continue search
            if (m_forbidden(*maxEdge->m_sink))
            {
                continue;
            }
            if(m_weights[*maxEdge] > weightDiff)
            {
                continue;
            }
            const double newWeight = state.weightThusfar + m_weights[*maxEdge];
            // Update best path
            if (bestWeight < newWeight && newWeight < maxWeight || (*maxEdge->m_sink == m_sink))
            {
                bestWeight = newWeight;
                // Overwrite best path
                std::copy_n(currentPath.begin(), m_k - 1, bestPath.begin());
                // Assign end
                bestPath.back() = maxEdge;
            }
        }
        else
        {
            // Find a usable child vertex to continue the search on
            auto it = std::find_if(state.currIt, state.endIt, [this](Edge* e)
            {
                return *e->m_sink == m_sink || !m_forbidden(*e->m_sink);
            });

            // Reached end of children
            if (it == state.endIt)
            {
                m_forbidden(*state.vertex) = 0;
            }
                // Found sink, so we will just stop searching
            else if ((*it)->m_sink->id() == m_sink)
            {
                std::copy_n(currentPath.begin(), pathIndexTarget, bestPath.begin());
                bestPath[pathIndexTarget] = *it;
                bestPath.resize(pathIndexTarget + 1);
                // Break out of the loop
                break;
            }
                // Found a new edge to move along
            else
            {
                currentPath[pathIndexTarget] = *it;
                // Set current iterator to just after the found element.
                state.currIt = it;
                std::advance(state.currIt, 1);

                stack.push(state);
                m_forbidden(*state.vertex) = 1;
                // Run on the next vertex
                stack.push(State((*it)->m_sink, state.weightThusfar + m_weights[(*it)->id()]));
                m_forbidden(*(*it)->m_sink) = 1;
            }
        }
    }

    // Return true if the best path is something meaningful
    return bestPath[0] != nullptr;
}

void k_SGL::applyKSGL(double maxWeight)
{
    m_forbidden.setConstant(m_graph->number_of_vertices(), 0);
    m_reachable.setConstant(m_graph->number_of_vertices(), 1);
    bool done = false;
    // The segments
    std::stack<KSegment> segs;
    int source = m_source;
    std::stack<int> sources;
    // Push a source and a new empty segment to fill.
    sources.push(source);
    segs.push(KSegment(&m_weights));
    segs.top().maxWeight = maxWeight;
    // Go on until we find a path or are back at the initial source
    // with no found paths.
    while (!sources.empty())
    {
        // Reset forbidden nodes
        for (auto edge : segs.top().edges)
        {
            if (edge != nullptr)
                m_forbidden(*edge->m_sink) = 0;
        }
        // Found new, good k segments
        if (bruteForceBestKPath(sources.top(), segs.top().edges, segs.top().maxWeight))
        {
            std::cout << "[k_SGL] Found new K segment with maxweight " << segs.top().maxWeight << " and own weight " << segs.top().weight() << std::endl;

            // Found sink: done
            if (segs.top().edges.back()->hasSink(m_sink)) break;

            // Update forbidden to be sure
            for (auto edge : segs.top().edges)
            {
                m_forbidden(*edge->m_sink) = 1;
            }

            // Push last node as next source
            sources.push(*segs.top().edges.back()->m_sink);
            // Push new empty segment to fill
            segs.push(KSegment(&m_weights));
        }
            // No path found (either not viable or not within max weight))
        else
        {
            std::cout << "[k_SGL] No segment found" << std::endl;
            // Remove source
            sources.pop();
            // Remove seg
            segs.pop();

            // We are through
            if (sources.size() == 0) break;

            // Apply searching on previous segment with smaller weight,
            // which hopefully gives a path.
            segs.top().maxWeight = segs.top().weight();
        }
    }
    //
    m_found = !segs.empty() && !segs.top().edges.empty() && segs.top().edges.back()->hasSink(m_sink);
    if (m_found)
    {
        // Reconstruct path from segments
        auto cnt = (segs.size() - 1) * 3 + segs.top().edges.size();
        // Setup nodes in best path
        m_best.resize(cnt, nullptr);
        // Add the segments. Note that they have to be popped
        // in reverse order.
        int targetLocation = m_best.size() - 1; //Target location of last element in curretn segment in the nodes array.
        while (!segs.empty())
        {
            auto el = segs.top();
            segs.pop();
            int start = targetLocation - (el.edges.size() - 1);
            for (int i = 0; i < el.edges.size(); ++i)
            {
                m_best[start + i] = el.edges[i];
            }
            targetLocation = start - 1;
        }
    }
}

k_SGL::KSegment::KSegment(LoopsLib::Algs::FieldType* weights): start(nullptr), m_weights(weights)
{
}

double k_SGL::KSegment::weight() const
{
    double w = 0;
    for (auto el : edges)
    {
        w += (*m_weights)[el->id()];
    }
    return w;
}

void k_SGL::setK(int k)
{
    m_k = k;
}

LoopsLib::Algs::BasisElement k_SGL::computeLongestPath(const LoopsLib::Algs::FieldType& field, NT maxWeight)
{
    m_weights = field;

    std::cout << "[LongestPath]\tApply k_SGL" << std::endl;
    // Apply the algorithm
    applyKSGL(maxWeight);
    //apply();

    std::cout << "[LongestPath]\tPath of length:" << m_best.size() << " and weight " << m_bestWeight << std::endl;

    if (m_best.empty())
    {
        return BasisElement{};
    }

    if(!m_graph->isSimplePath(m_best))
    {
        std::cout << "[k_SGL] Non simple path!" << std::endl;
    }

    // Convert back to vector
    m_bestWeight = 0;
    for (int i = 0; i < m_best.size(); i++)
    {
        m_bestWeight += m_weights[m_best[i]->id()];
    }
    Algs::BasisElement ret;
    std::transform(m_best.begin(), m_best.end(), std::back_inserter(ret), [](auto* e) {return e->id(); });
    return ret;

}

LoopsLib::Algs::BasisElement k_SGL::retry(const FieldType& field)
{
    return computeLongestPath(field, m_bestWeight);
}
