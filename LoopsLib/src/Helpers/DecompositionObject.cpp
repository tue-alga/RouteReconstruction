#include <LoopsLib/Helpers/DecompositionObject.h>
#include <Eigen/src/Core/util/ForwardDeclarations.h>
#include <LoopsLib/Helpers/RandomHelpers.h>
#include <LoopsLib/Math/Vector.h>
using namespace LoopsLib;

const Helpers::DecompositionObject::Point& Helpers::DecompositionObject::vertexLocation(DS::BaseGraph::Id_t vertex) const
{
    return m_graph->locations()[vertex];
}

const Helpers::DecompositionObject::Point& Helpers::DecompositionObject::vertexLocation(DS::BaseGraph::Vertex* vertex) const
{
    return vertexLocation(vertex->id());
}

std::size_t Helpers::DecompositionObject::basisSize() const
{
    return m_basis.size();
}

NT Helpers::DecompositionObject::fieldValue() const
{
    return Math::VectorWrapper<NT>(m_field).normSq();
}

std::vector<NT> Helpers::DecompositionObject::decompositionField() const
{
    std::vector<NT> ret;
    ret.assign(m_field.size(), 0);
    for (std::size_t i = 0; i < m_basis.size(); ++i)
    {
        const auto& path = m_basis[i];
        for (auto* e : path)
        {
            ret[e->id()] += m_decompositionCoeffs[i];
        }
    }
    return ret;
}

void Helpers::DecompositionObject::pickRandomAvailablePaths(std::size_t number)
{
    std::vector<std::size_t> ids;
    ids.resize(m_paths.size(), 0);
    std::iota(ids.begin(), ids.end(), 0);
    auto randEng = Helpers::RandomHelpers::getRandomEngine();
    std::shuffle(ids.begin(), ids.end(), randEng);

    m_availablePaths.clear();
    for(std::size_t i = 0; i < number; ++i)
    {
        m_availablePaths.insert(ids[i]);
    }
}

void Helpers::DecompositionObject::reconstructFieldFromPaths()
{
    m_field.assign(m_graph->number_of_edges(), 0);
    for (std::size_t i = 0; i < m_paths.size(); ++i)
    {
        for (auto* e : m_paths[i])
        {
            m_field[e->id()] += m_pathValues[i];
        }
    }
}

void Helpers::DecompositionObject::sourcesAndSinksFromPaths()
{
    m_sources.clear();
    m_sinks.clear();
    for (const auto& p : m_paths)
    {
        m_sources.insert(p[0]->m_source->id());
        m_sinks.insert(p.back()->m_sink->id());
    }
}

void Helpers::DecompositionObject::clearSources()
{
    m_sources.clear();
}

void Helpers::DecompositionObject::clearSinks()
{
    m_sinks.clear();
}

bool Helpers::DecompositionObject::isSameElement(std::size_t basisElement,
                                                 const std::vector<DS::BaseGraph::Edge*>& element)
{
    return isSameElement(m_basis[basisElement], element);
}

bool Helpers::DecompositionObject::isSameElement(const std::vector<DS::BaseGraph::Edge*>& e0,
                                                 const std::vector<DS::BaseGraph::Edge*>& e1)
{
    if (e0.size() != e1.size()) return false;
    for (std::size_t i = 0; i < e0.size(); ++i)
    {
        if (e0[i]->id() != e1[i]->id()) return false;
    }
    return true;
}

void Helpers::DecompositionObject::clear()
{
    m_basis.clear();
    m_decompositionCoeffs.clear();
    m_field.clear();
}

bool Helpers::DecompositionObject::isBasisElementSimple(int index, bool& isEmpty)
{
    isEmpty = false;
    std::set<DS::BaseGraph::Id_t> seenVerts;
    if(index >= m_paths.size())
    {
        isEmpty = true;
        return true;
    }
    seenVerts.insert(m_paths[index][0]->m_source->id());
    for(auto* e: m_paths[index])
    {
        if (seenVerts.find(e->m_sink->id()) != seenVerts.end()) return false;
        seenVerts.insert(e->m_sink->id());
    }
    return true;
}

Eigen::VectorXd Helpers::DecompositionObject::getEigenPath(DS::BaseGraph* graph, const std::vector<DS::BaseGraph::Edge*>& edges)
{
    Eigen::VectorXd ret;
    ret.setConstant(graph->number_of_edges(), 0);
    for (auto* e : edges)
    {
        ret(e->id()) += 1;
    }
    return ret;
}

Eigen::VectorXd Helpers::DecompositionObject::getEigenPath(DS::BaseGraph* graph,
                                                           const std::vector<DS::BaseGraph::Id_t>& vertices)
{
    Eigen::VectorXd ret;
    ret.setConstant(graph->number_of_edges(), 0);
    for (auto i = 1; i < vertices.size(); ++i)
    {
        auto* e = graph->vertex(vertices[i - 1])->findOutEdge(graph->vertex(vertices[i]));
        ret(e->id()) += 1.0;
    }
    return ret;
}

std::vector<DS::BaseGraph::Edge*> Helpers::DecompositionObject::toEdgePointers(
    const std::vector<DS::BaseGraph::Id_t>& vertices) const
{
    std::vector<DS::BaseGraph::Edge*> out;
    out.reserve(vertices.size() - 1);
    for (std::size_t i = 0; i < vertices.size() - 1; ++i)
    {
        out.push_back(m_graph->vertex(vertices[i])->findOutEdge(vertices[i + 1]));
    }
    return out;
}

void Helpers::DecompositionObject::extendBasis(const Basis_t& extraElements)
{
    m_basis.reserve(m_basis.size() + extraElements.size());
    m_basis.insert(m_basis.end(), extraElements.begin(), extraElements.end());
}

void Helpers::DecompositionObject::applyNNLSAndPrune(double pruneValue)
{
    applyNNLSForCoefficients();
    sortBasisByDecompCoeff();
    prune(pruneValue);
}

Eigen::VectorXi Helpers::DecompositionObject::verifySimplicity()
{
    Eigen::VectorXi result;
    result.setConstant(m_basis.size(), 1);
    //Find a start edge for the basis
    for (int r = 0; r < m_basis.size(); r++)
    {
        bool empty = false;
        if (!isBasisElementSimple(r, empty))
        {
            result(r) = 0;
        }
        else if (empty)
        {
            result(r) = 2;
        }
    }
    return result;
}

std::vector<NT> Helpers::DecompositionObject::residual() const
{
    std::vector<NT> ret(m_field.begin(), m_field.end());
    ret.resize(m_field.size(), 0);
    for(std::size_t  i =0; i < m_field.size(); ++i)
    {
        ret[i] = m_field[i];
    }
    for(std::size_t i = 0; i < m_basis.size(); ++i)
    {
        for(auto* e: m_basis[i])
        {
            ret[e->id()] -= m_decompositionCoeffs[i];
        }
    }

    return ret;
}

void Helpers::DecompositionObject::prune(double minimumDecompCoeffValue)
{
    if (m_decompositionCoeffs.size() == 0)
    {
        // Warn called on empty decomposition object.
        m_logger.warn("Pruning empty basis");
        return;
    }
    int r = 0;
    for (; r < m_decompositionCoeffs.size(); ++r)
    {
        if (m_decompositionCoeffs[r] < minimumDecompCoeffValue)
        {
            break;
        }
    }
    m_logger.info("Pruned basis from ", m_decompositionCoeffs.size(), " to ", r);
    m_basis.resize(r);
    m_decompositionCoeffs.resize(r);
}

void Helpers::DecompositionObject::applyNNLSForCoefficients()
{
    Algs::NNLS nnls;
    nnls.solve(m_basis, m_field, m_decompositionCoeffs, m_objectValueNNLS);
}

void Helpers::DecompositionObject::sortBasisByDecompCoeff()
{
    // Will contain the order of the elements (value at index i is the ID of the element to go to index i)
    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < m_decompositionCoeffs.size(); i++)
    {
        order.emplace_back(i);
    }
    std::sort(order.begin(), order.end(), [this](std::size_t i0, std::size_t i1)
    {
        return m_decompositionCoeffs[i0] > m_decompositionCoeffs[i1];
    });

    Basis_t newBasis;
    newBasis.reserve(m_basis.size());
    Coeffs_t newCoeffs;
    newCoeffs.reserve(m_basis.size());
    for(std::size_t i = 0; i < m_basis.size(); ++i)
    {
        newBasis.emplace_back(std::move(m_basis[order[i]]));
        newCoeffs.push_back(m_decompositionCoeffs[order[i]]);
    }
    m_basis = std::move(newBasis);
    m_decompositionCoeffs = newCoeffs;
}
