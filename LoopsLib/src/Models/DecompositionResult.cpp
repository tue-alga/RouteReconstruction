#include <LoopsLib/Models/DecompositionResult.h>
using namespace LoopsLib;

void Models::DecompositionResult::extendBasis(long long srcRepresentative, const std::vector<GraphPath_t>& newElements)
{
    // Reserve space 
    m_basisIds.reserve(m_basis.size());
    m_basisSrcRepresentatives.reserve(m_basis.size());
    m_basis.reserve(m_basis.size() + newElements.size());

    for (const auto& el : newElements)
    {
        m_basisIds.push_back(currentId);
        ++currentId;
        m_basisSrcRepresentatives.push_back(srcRepresentative);
        m_basis.push_back(el);
    }
}

void Models::DecompositionResult::updateRepresentativeIdsForNewElements(long long srcRepresentative)
{
    // Reserve space 
    m_basisIds.reserve(m_basis.size());
    m_basisSrcRepresentatives.reserve(m_basis.size());

    for (auto i = m_basisIds.size(); i < m_basis.size(); ++i)
    {
        m_basisIds.push_back(currentId);
        ++currentId;
        m_basisSrcRepresentatives.push_back(srcRepresentative);
    }
}

void Models::DecompositionResult::clear()
{
    m_timings.clear();
    m_objectValueNNLS = std::numeric_limits<double>::max();
    m_basis.clear();
    m_decompositionCoeffs.clear();
    m_decompositionPath = "";
    m_distortion = 0;
    currentId = 0;
}

std::size_t Models::DecompositionResult::basisSize() const
{
    return m_basis.size();
}

void Models::DecompositionResult::recomputeObjectiveValue()
{
    auto resid = residual();
    m_objectValueNNLS = std::inner_product(resid.begin(), resid.end(), resid.begin(), (NT)0);
}

void Models::DecompositionResult::pruneToTopWeighted(int number)
{
    if (m_basis.size() <= number) return;

    std::set<std::pair<NT, std::size_t>> weightToIndex;
    for(std::size_t i = 0; i < m_basis.size(); ++i)
    {
        if(weightToIndex.size() < number)
        {
            weightToIndex.insert(std::make_pair(m_decompositionCoeffs[i], i));
        }
        else
        {
            auto endEl = *weightToIndex.begin();
            if(m_decompositionCoeffs[i] > endEl.first)
            {
                weightToIndex.insert(std::make_pair(m_decompositionCoeffs[i], i));
                weightToIndex.erase(weightToIndex.begin());
            }
        }
    }
    std::size_t newIndex = 0;
    decltype(m_basis) newBasis;
    std::vector<NT> newCoeffs;
    for(const auto& el: weightToIndex)
    {
        newBasis.push_back(m_basis[el.second]);
        newCoeffs.push_back(el.first);
    }
    m_basis = newBasis;
    m_decompositionCoeffs = newCoeffs;
}

std::vector<NT> Models::DecompositionResult::decompositionField() const
{
    std::vector<NT> ret;
    ret.assign(m_relatedInstance->m_field->m_data.size(), 0);
    for (std::size_t i = 0; i < m_basis.size(); ++i)
    {
        const auto& path = m_basis[i];
        for (const auto& eId : path)
        {
            ret[eId] += m_decompositionCoeffs[i];
        }
    }
    return ret;
}

const std::vector<Models::DecompositionResult::BasisElement_t>& Models::DecompositionResult::paths() const
{
    if (!m_relatedInstance) throw std::runtime_error("No problem instance set!");
    return m_relatedInstance->m_field->m_paths;
}

const std::vector<NT>& Models::DecompositionResult::field() const
{
    if (!m_relatedInstance) throw std::runtime_error("No problem instance set!");
    return m_relatedInstance->m_field->m_data;
}


void Models::DecompositionResult::extendBasis(const Basis_t& extraElements)
{
    m_basis.reserve(m_basis.size() + extraElements.size());
    m_basis.insert(m_basis.end(), extraElements.begin(), extraElements.end());
}

void Models::DecompositionResult::applyNNLSAndPrune(double pruneValue)
{
    m_logger.info("Solving NNLS for basis of size ", basisSize());
    std::size_t totalEdges = 0;
    for (const auto& el : m_basis) totalEdges += el.size();
    m_logger.info("\tEdges in basis: ", totalEdges);
    applyNNLSForCoefficients();
    //m_logger.info("Sorting basis");
    //sortBasisByDecompCoeff();
    m_logger.info("Pruning");
    prune(pruneValue);
}

void Models::DecompositionResult::applyNNLSAndPruneBatched(double pruneValue, std::size_t batchSize)
{
    m_logger.info("Solving NNLS for basis of size ", basisSize());
    std::size_t totalEdges = 0;
    for (const auto& el : m_basis) totalEdges += el.size();
    m_logger.info("\tEdges in basis: ", totalEdges);
    Algs::NNLS nnls;
    nnls.solveBatched(m_basis, field(), batchSize, m_decompositionCoeffs, m_objectValueNNLS);

    //m_logger.info("Sorting basis");
    //sortBasisByDecompCoeff();
    m_logger.info("Pruning");
    prune(pruneValue);
}

bool Models::DecompositionResult::applyNnlsIncrementally(std::size_t batchSize, std::size_t retainSize,
    bool retainFraction)
{
    m_logger.info("Solving NNLS for basis of size ", basisSize());
    std::size_t totalEdges = 0;
    for (const auto& el : m_basis) totalEdges += el.size();
    m_logger.info("\tEdges in basis: ", totalEdges);
    Algs::NNLS nnls;
    std::vector<std::size_t> retainedIndices;
    if (!nnls.solveIncrementally(m_basis, field(), batchSize, retainSize, retainFraction, retainedIndices, m_decompositionCoeffs, m_objectValueNNLS)) {
        m_logger.error("NNLS: Failed to solve incrementally");
        return false;
    }

    decltype(m_basis) newBasis;
    std::transform(retainedIndices.begin(), retainedIndices.end(), std::back_inserter(newBasis), [this](const auto& ind) {return m_basis[ind]; });
    m_basis = newBasis;
    return true;
}

std::vector<NT> Models::DecompositionResult::residual() const
{
    if(field().empty())
    {
        throw std::runtime_error("Field is empty");
    }
    std::vector<NT> ret(field().begin(), field().end());
    ret.resize(field().size(), 0);
    for (std::size_t i = 0; i < ret.size(); ++i)
    {
        ret[i] = field()[i];
    }
    if(!m_basis.empty() && m_basis.size() == m_decompositionCoeffs.size())
    {
        for (std::size_t i = 0; i < m_basis.size(); ++i)
        {
            for (const auto& eId : m_basis[i])
            {
                ret[eId] -= m_decompositionCoeffs[i];
            }
        }
    }
    
    return ret;
}

void Models::DecompositionResult::prune(double minimumDecompCoeffValue)
{
    if (m_decompositionCoeffs.size() == 0)
    {
        // Warn called on empty decomposition object.
        m_logger.warn("Pruning empty basis");
        return;
    }
    std::size_t r = 0;
    std::size_t pruneCount = 0;
    bool encounteredPrunable = false;
    for (std::size_t i= 0; i < m_decompositionCoeffs.size(); ++i)
    {
        if (m_decompositionCoeffs[i] < minimumDecompCoeffValue)
        {
            if (!encounteredPrunable) {
                encounteredPrunable = true;
                r = i;
            }
            ++pruneCount;
        }
        else if(encounteredPrunable)
        {
            m_decompositionCoeffs[r] = std::move(m_decompositionCoeffs[i]);
            m_basis[r] = std::move(m_basis[i]);
            ++r;
        }
    }
    m_logger.info("Pruned basis from ", m_decompositionCoeffs.size(), " to ", m_decompositionCoeffs.size()-pruneCount);
    if(encounteredPrunable)
    {
        m_basis.resize(r);
        m_decompositionCoeffs.resize(r);
    }
}

void Models::DecompositionResult::applyNNLSForCoefficients()
{
    Algs::NNLS nnls;
    nnls.solve(m_basis, field(), m_decompositionCoeffs, m_objectValueNNLS);
}

void Models::DecompositionResult::sortBasisByDecompCoeff()
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
    for (std::size_t i = 0; i < m_basis.size(); ++i)
    {
        newBasis.emplace_back(std::move(m_basis[order[i]]));
        newCoeffs.push_back(m_decompositionCoeffs[order[i]]);
    }
    m_basis = std::move(newBasis);
    m_decompositionCoeffs = newCoeffs;
}
