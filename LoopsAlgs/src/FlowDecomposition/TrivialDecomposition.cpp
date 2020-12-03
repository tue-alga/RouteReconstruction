#include <LoopsAlgs/FlowDecomposition/TrivialDecomposition.h>
#include <LoopsLib/Helpers/Iterators.h>
using namespace LoopsLib;
void LoopsAlgs::FlowDecomposition::TrivialDecomposition::generatePaths(DS::BaseGraph::Vertex* vertex)
{
    struct State
    {
        using It = decltype(vertex->m_outEdges.begin());
        It curr;
        It end;
    };
    std::stack<State> edgeStack;
    std::vector<DS::BaseGraph::Id_t> currPath;
    currPath.resize(m_pathSize, -1);

    edgeStack.push({ vertex->m_outEdges.begin(), vertex->m_outEdges.end() });
    while (!edgeStack.empty())
    {
        auto& state = edgeStack.top();
        if (state.curr == state.end)
        {
            edgeStack.pop();
            if (!edgeStack.empty())
            {
                ++edgeStack.top().curr;
            }
            continue;
        }
        if (edgeStack.size() == m_pathSize)
        {
            for (auto* e : Helpers::Iterators::Iterable(state.curr, state.end))
            {
                currPath[m_pathSize - 1] = e->id();
                basis().push_back(currPath);
            }
            edgeStack.pop();
            if (!edgeStack.empty())
            {
                ++edgeStack.top().curr;
            }
            continue;
        }
        auto* e = *state.curr;
        auto* v = e->m_sink;
        edgeStack.push(State{v->m_outEdges.begin(), v->m_outEdges.end()});
    }
}

void LoopsAlgs::FlowDecomposition::TrivialDecomposition::findNewBasisElements()
{
    for (auto* v : graph()->vertices())
    {
        generatePaths(v);
    }
}
