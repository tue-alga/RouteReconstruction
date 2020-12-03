#include <LoopsLib/Models/ReconstructedTrajectory.h>

LoopsLib::Models::ReconstructedTrajectory::ReconstructedTrajectory(long long id): m_id(id)
{
}

LoopsLib::Models::ReconstructedTrajectory::ReconstructedTrajectory(const std::vector<DS::BaseGraph::EdgeIndex>& path,
                                                                   std::size_t sourceRepresentative, long long id):
    m_edges(path),
    m_sourceRepresentative(sourceRepresentative),
    m_id(id)
{
}

std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>::const_iterator LoopsLib::Models::ReconstructedTrajectory::begin() const
{
    return m_edges.begin();
}

std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>::const_iterator LoopsLib::Models::ReconstructedTrajectory::end() const
{
    return m_edges.end();
}

void LoopsLib::Models::ReconstructedTrajectory::setId(long long id)
{
    m_id = id;
}

void LoopsLib::Models::ReconstructedTrajectory::setSourceRepresentative(std::size_t sourceRepresentative)
{
    m_sourceRepresentative = sourceRepresentative;
}

std::size_t LoopsLib::Models::ReconstructedTrajectory::sourceRepresentative() const
{
    return m_sourceRepresentative;
}

const std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>& LoopsLib::Models::ReconstructedTrajectory::path() const
{
    return m_edges;
}

std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>& LoopsLib::Models::ReconstructedTrajectory::path()
{
    return m_edges;
}

long long LoopsLib::Models::ReconstructedTrajectory::id() const
{
    return m_id;
}
