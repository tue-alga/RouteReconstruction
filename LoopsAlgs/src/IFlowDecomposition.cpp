#include <LoopsAlgs/IFlowDecomposition.h>

std::ostream& LoopsAlgs::FlowDecomposition::IFlowDecomposition::log()
{
    return std::cout << "[" << m_name << "] ";
}

LoopsAlgs::FlowDecomposition::IFlowDecomposition::Basis_t& LoopsAlgs::FlowDecomposition::IFlowDecomposition::basis()
{
    return m_decompObj->m_basis;
}

std::size_t LoopsAlgs::FlowDecomposition::IFlowDecomposition::numberOfRepresentatives() const
{
    return m_decompObj->m_relatedInstance->m_representatives.size();
}

const LoopsLib::Models::ProblemInstance::Trajectory& LoopsAlgs::FlowDecomposition::IFlowDecomposition::representative(
    std::size_t index) const
{
    return m_decompObj->m_relatedInstance->m_representatives[index];
}

LoopsAlgs::FlowDecomposition::IFlowDecomposition::DecompositionCoeffs_t& LoopsAlgs::FlowDecomposition::
IFlowDecomposition::coefficients()
{
    return m_decompObj->m_decompositionCoeffs;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::extendBasis(const Basis_t& extra)
{
    m_decompObj->extendBasis(extra);
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::pruneBasis(double minValue)
{
    m_decompObj->sortBasisByDecompCoeff();
    m_decompObj->prune(minValue);
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::solveNNLS()
{
    m_decompObj->applyNNLSForCoefficients();
}

LoopsAlgs::FlowDecomposition::IFlowDecomposition::DecompositionCoeffs_t& LoopsAlgs::FlowDecomposition::
IFlowDecomposition::field()
{
    return m_decompObj->m_relatedInstance->m_field->m_data;
}

LoopsLib::DS::EmbeddedGraph* LoopsAlgs::FlowDecomposition::IFlowDecomposition::graph()
{
    return m_decompObj->m_relatedInstance->m_graph;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::init()
{
    reset();
}

LoopsAlgs::FlowDecomposition::IFlowDecomposition::IFlowDecomposition(const std::string& name,
                                                                     DecompositionResult_t* decompObj):
    m_decompObj(decompObj),
    m_name(name)
{
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::reset()
{
    // Clear basis 
    basis().clear();
    coefficients().clear();
    m_currentIteration = 0;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::setDecompositionObject(DecompositionResult_t* decompObj)
{
    m_decompObj = decompObj;
    reset();
}

LoopsAlgs::FlowDecomposition::IFlowDecomposition::DecompositionResult_t* LoopsAlgs::FlowDecomposition::
IFlowDecomposition::decompositionObject() const
{
    return m_decompObj;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::setNumberOfIterations(int numberOfIts)
{
    m_numberOfIterations = numberOfIts;
}

int LoopsAlgs::FlowDecomposition::IFlowDecomposition::numberOfIterations() const
{
    return m_numberOfIterations;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::setPruneValue(double value)
{
    m_pruneValue = value;
}

double LoopsAlgs::FlowDecomposition::IFlowDecomposition::pruneValue() const
{
    return m_pruneValue;
}

std::string LoopsAlgs::FlowDecomposition::IFlowDecomposition::logDir() const
{
    return m_logDir;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::setLogDir(std::string logDir)
{
    m_logDir = logDir;
}

std::string LoopsAlgs::FlowDecomposition::IFlowDecomposition::logPrefix() const
{
    return m_logPrefix;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::setLogPrefix(std::string logDir)
{
    m_logPrefix = logDir;
}

std::string LoopsAlgs::FlowDecomposition::IFlowDecomposition::logFilePrefix() const
{
    return m_logFilePrefix;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::setLogFilePrefix(std::string logDir)
{
    m_logFilePrefix = logDir;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::nextStep()
{
    this->findNewBasisElements();
    ++m_currentIteration;
}

bool LoopsAlgs::FlowDecomposition::IFlowDecomposition::isDone()
{
    return m_currentIteration == m_numberOfIterations;
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::setName(const std::string& value)
{
    m_name = value;
}

std::string LoopsAlgs::FlowDecomposition::IFlowDecomposition::name() const
{
    return m_name;
}

LoopsAlgs::FlowDecomposition::IFlowDecomposition::~IFlowDecomposition()
{
}

void LoopsAlgs::FlowDecomposition::IFlowDecomposition::decompose()
{
    init();
    while (!isDone())
    {
        nextStep();
        m_decompObj->applyNNLSAndPrune(m_pruneValue);
    }
}
