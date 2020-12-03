#include <LoopsLib/Models/ProblemInstance.h>
#include <Eigen/src/Core/util/ForwardDeclarations.h>
#include <LoopsLib/Helpers/RandomHelpers.h>
using namespace LoopsLib;
using namespace LoopsLib::Models;

bool ProblemInstance::TrajectorySource::hasPathSource() const
{
    return m_pathSource >= 0;
}

std::string ProblemInstance::TrajectorySource::desc() const
{
    return hasPathSource() ? "Path " + std::to_string(m_pathSource) : m_trajectoryId;
}

void ProblemInstance::setRepresentativesFromPaths(const std::set<std::size_t>& paths)
{
    m_representativeSources.clear();
    m_representatives.clear();
    m_representatives.reserve(paths.size());
    m_representativeSources.reserve(paths.size());
    for (const auto& pId : paths)
    {
        m_representativeSources.push_back({"", (long long)pId});
        m_representatives.push_back({});
        DS::edgePathToLocations(*m_graph, m_field->m_paths[pId], m_representatives.back());
    }
}
void ProblemInstance::setRepresentativesFromPaths(const std::vector<std::size_t>& paths)
{
    m_representativeSources.clear();
    m_representatives.clear();
    m_representatives.reserve(paths.size());
    m_representativeSources.reserve(paths.size());
    for (const auto& pId : paths)
    {
        m_representativeSources.push_back({ "", (long long)pId });
        m_representatives.push_back({});
        DS::edgePathToLocations(*m_graph, m_field->m_paths[pId], m_representatives.back());
    }
}

void ProblemInstance::setRepresentativesFromPaths(const std::set<long long>& paths)
{
    m_representativeSources.clear();
    m_representatives.clear();
    m_representatives.reserve(paths.size());
    m_representativeSources.reserve(paths.size());
    for (const auto& pId : paths)
    {
        m_representativeSources.push_back({ "", pId });
        m_representatives.push_back({});
        DS::edgePathToLocations(*m_graph, m_field->m_paths[pId], m_representatives.back());
    }
}

void ProblemInstance::setRepresentativesFromPaths(const std::vector<long long>& paths)
{
    m_representativeSources.clear();
    m_representatives.clear();
    m_representatives.reserve(paths.size());
    m_representativeSources.reserve(paths.size());
    for (const auto& pId : paths)
    {
        m_representativeSources.push_back({ "", pId });
        
        m_representatives.push_back({});
        DS::edgePathToLocations(*m_graph, m_field->m_paths[pId], m_representatives.back());
    }
}

void ProblemInstance::addRepresentativesFromPaths(const std::set<std::size_t>& paths)
{
    m_representatives.reserve(m_representatives.size() + paths.size());
    m_representativeSources.reserve(m_representatives.size() + paths.size());
    for (const auto& pId : paths)
    {
        m_representativeSources.push_back({"", (long long)pId});
        m_representatives.push_back({});
        DS::edgePathToLocations(*m_graph, m_field->m_paths[pId], m_representatives.back());
    }
}

void ProblemInstance::addRepresentativeFromPath(const std::size_t& pId)
{
    m_representativeSources.push_back({"", (long long)pId});
    m_representatives.push_back({});
    DS::edgePathToLocations(*m_graph, m_field->m_paths[pId], m_representatives.back());
}

void ProblemInstance::setRepresentativesFromTrajectories(const std::vector<Trajectory>& trajectories,
                                                         const std::vector<std::string>& ids)
{
    assert(trajectories.size() == ids.size());

    m_representativeSources.clear();
    m_representatives.clear();
    m_representatives.reserve(trajectories.size());
    m_representativeSources.reserve(trajectories.size());
    for (auto i = 0; i < trajectories.size(); ++i)
    {
        m_representativeSources.push_back({ids[i], -1});
        m_representatives.push_back(trajectories[i]);
    }
    assert(m_representatives.size() == m_representativeSources.size());
}

void ProblemInstance::setRepresentativesFromTrajectories(const LoopsLib::MovetkGeometryKernel::TrajectorySet& trajSet,
    const std::set<long long>& indices, bool convertCrs)
{
    m_representativeSources.clear();
    m_representatives.clear();
    m_representatives.reserve(indices.size());
    m_representativeSources.reserve(indices.size());
    for (auto i = 0; i < indices.size(); ++i)
    {
        m_representativeSources.push_back({ trajSet.ids[i], -1 });
        if(convertCrs)
        {
            LoopsLib::MovetkGeometryKernel::Trajectory traj = trajSet.trajectories[i];
            LoopsLib::MovetkGeometryKernel().convertCRS(trajSet.m_ref, m_graph->spatialRef(), traj);
            m_representatives.push_back(traj);
        }
        else
        {
            m_representatives.push_back(trajSet.trajectories[i]);
        }
    }
}

void ProblemInstance::setRepresentativesFromTrajectories(const LoopsLib::MovetkGeometryKernel::TrajectorySet& trajSet,
    const std::vector<long long>& indices, bool convertCrs)
{
    m_representativeSources.clear();
    m_representatives.clear();
    m_representatives.reserve(indices.size());
    m_representativeSources.reserve(indices.size());
    for (auto i = 0; i < indices.size(); ++i)
    {
        m_representativeSources.push_back({ trajSet.ids[i], -1 });
        if(convertCrs)
        {
            LoopsLib::MovetkGeometryKernel::Trajectory traj = trajSet.trajectories[i];
            LoopsLib::MovetkGeometryKernel().convertCRS(trajSet.m_ref, m_graph->spatialRef(), traj);
            m_representatives.push_back(traj);
        }
        else
        {
            m_representatives.push_back(trajSet.trajectories[i]);
        }
    }
}

void ProblemInstance::addRepresentativesFromTrajectories(const std::vector<Trajectory>& trajectories,
                                                         const std::vector<std::string>& ids)
{
    assert(trajectories.size() == ids.size());

    m_representatives.reserve(m_representatives.size() + trajectories.size());
    m_representativeSources.reserve(m_representatives.size() + trajectories.size());
    for (auto i = 0; i < trajectories.size(); ++i)
    {
        m_representativeSources.push_back({ids[i], -1});
        m_representatives.push_back(trajectories[i]);
    }
    assert(m_representatives.size() == m_representativeSources.size());
}

void ProblemInstance::addRepresentativeFromTrajectory(const Trajectory& trajectory, const std::string& id)
{
    m_representativeSources.push_back({id, -1});
    m_representatives.push_back(trajectory);
}

const ProblemInstance::Point& ProblemInstance::vertexLocation(DS::BaseGraph::Id_t vertex) const
{
    return m_graph->locations()[vertex];
}

const ProblemInstance::Point& ProblemInstance::vertexLocation(DS::BaseGraph::Vertex* vertex) const
{
    return vertexLocation(vertex->id());
}


void ProblemInstance::pickRandomAvailablePaths(std::size_t number)
{
    std::vector<std::size_t> ids;
    ids.resize(m_field->m_paths.size(), 0);
    std::iota(ids.begin(), ids.end(), 0);
    auto randEng = Helpers::RandomHelpers::getRandomEngine();
    std::shuffle(ids.begin(), ids.end(), randEng);

    setRepresentativesFromPaths(ids);
}

std::string ProblemInstance::representativeDescription(size_t index)
{
    if (index >= m_representativeSources.size()) return "";
    const auto& src = m_representativeSources[index];
    return src.hasPathSource() ? "Path: " + std::to_string(src.m_pathSource) : src.m_trajectoryId;
}

Eigen::VectorXd ProblemInstance::getEigenPath(DS::BaseGraph* graph, const std::vector<DS::BaseGraph::Edge*>& edges)
{
    Eigen::VectorXd ret;
    ret.setConstant(graph->number_of_edges(), 0);
    for (auto* e : edges)
    {
        ret(e->id()) += 1;
    }
    return ret;
}

Eigen::VectorXd ProblemInstance::getEigenPath(DS::BaseGraph* graph,
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

std::vector<LoopsLib::DS::BaseGraph::Edge*> ProblemInstance::toEdgePointers(
    const std::vector<LoopsLib::DS::BaseGraph::Id_t>& vertices) const
{
    std::vector<DS::BaseGraph::Edge*> out;
    out.reserve(vertices.size() - 1);
    for (std::size_t i = 0; i < vertices.size() - 1; ++i)
    {
        out.push_back(m_graph->vertex(vertices[i])->findOutEdge(vertices[i + 1]));
    }
    return out;
}
