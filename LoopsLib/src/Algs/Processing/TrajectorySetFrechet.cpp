#include <LoopsLib/Algs/Processing/TrajectorySetFrechet.h>
#include <LoopsLib/DS/EmbeddedGraph.h>
#include "movetk/metric/Norm.h"
using namespace LoopsLib;
using namespace LoopsLib::Algs;
using namespace LoopsLib::Algs::Processing;

LoopsLib::NT TrajectorySetFrechet::Norm::operator()(const MovetkGeometryKernel::MovetkPoint& p0,
                                                             const MovetkGeometryKernel::MovetkPoint& p1)
{
    const auto xDiff = p0.get().x() - p1.get().x();
    const auto yDiff = p0.get().y() - p1.get().y();
    return std::sqrt(xDiff * xDiff + yDiff * yDiff);
    // Latest StrongFrechet requires squared distance
    //return xDiff * xDiff + yDiff * yDiff;
}

NT TrajectorySetFrechet::tolerance() const
{
    return m_tolerance;
}

void TrajectorySetFrechet::setTolerance(NT tolerance)
{
    m_tolerance = tolerance;
}

// 
using SFD_local = movetk_algorithms::StrongFrechetDistance<MovetkGeometryKernel, TrajectorySetFrechet::Norm>;
using SFD_tested = movetk_algorithms::StrongFrechet<MovetkGeometryKernel, movetk_support::squared_distance_d<MovetkGeometryKernel, movetk_support::FiniteNorm<MovetkGeometryKernel, 2>>>;
using SFD = SFD_tested;

void TrajectorySetFrechet::toMovetkTrajectory(const BasisElement& trajectory,
    DS::EmbeddedGraph* graph,
                                                                 std::vector<Point>& movetkTrajectory)
{
    // Convert to Movetk trajectory
    std::transform(trajectory.begin(), trajectory.end(), std::back_inserter(movetkTrajectory),
                   [graph](const auto& eId)
                   {
                       return graph->locations()[graph->edge(eId)->m_source->id()];
                   });
    movetkTrajectory.push_back(graph->locations()[graph->edge(trajectory.back())->m_sink->id()]);
}

void TrajectorySetFrechet::minFrechetValues(DS::EmbeddedGraph* graph,
                                                               const std::vector<BasisElement>& trajectorySet,
                                                               const std::vector<BasisElement>& compareAgainst,
    NT upperBound,
                                                               std::vector<NT>& outputValues)
{
    std::vector<std::size_t> dummy;
    minFrechetValues(graph, trajectorySet, compareAgainst, upperBound, outputValues, dummy);
}

void TrajectorySetFrechet::minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<BasisElement>& trajectorySet,
    const std::vector<std::vector<Point>>& compareAgainst, NT upperBound, std::vector<NT>& outputValues)
{
    std::vector<std::size_t> dummy;

    // Somewhat unfortunate for larger sets
    std::vector<std::vector<Point>> trajectorySetMtkVector;
    trajectorySetMtkVector.reserve(trajectorySet.size());
    for (const auto& el : trajectorySet)
    {
        trajectorySetMtkVector.push_back({});
        toMovetkTrajectory(el, graph, trajectorySetMtkVector.back());
    }
    minFrechetValues(graph, trajectorySetMtkVector, compareAgainst, upperBound, outputValues, dummy);
}

void TrajectorySetFrechet::minFrechetValues(DS::EmbeddedGraph* graph,
    const std::vector<std::vector<Point>>& trajectorySet, const std::vector<BasisElement>& compareAgainst,
    NT upperBound, std::vector<NT>& outputValues)
{
    std::vector<std::size_t> dummy;

    // Somewhat unfortunate for larger sets
    std::vector<std::vector<Point>> compareAgainstMtk;
    compareAgainstMtk.reserve(compareAgainst.size());
    for(const auto& el: compareAgainst)
    {
        compareAgainstMtk.push_back({});
        toMovetkTrajectory(el, graph, compareAgainstMtk.back());
    }
    minFrechetValues(graph, trajectorySet, compareAgainstMtk, upperBound, outputValues, dummy);
}

void TrajectorySetFrechet::minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<std::vector<Point>>& trajectorySet,
    const std::vector< std::vector<Point>>& compareAgainst, NT upperBound, std::vector<NT>& outputValues,
    std::vector<std::size_t>& setIndices)
{
    outputValues.reserve(trajectorySet.size());
    setIndices.reserve(trajectorySet.size());

    SFD frechetDist;
    using FrMode = decltype(frechetDist.mode());
    frechetDist.setMode(FrMode::BisectionSearch);
    frechetDist.setTolerance(m_tolerance);

    std::size_t currTraj = 0, currCompar = 0;
    int percIncr = 5;
    int perc = percIncr;
    for (const auto& trajectory : trajectorySet)
    {
        //std::cout << "Cur traj: " << currTraj << std::endl;
        if (trajectory.empty())
        {
            continue;
        }
        NT minFrechetValue = upperBound;

        // Index of best trajectory
        std::size_t bestTrajectory = 0;
        for (auto i = 0; i < compareAgainst.size(); ++i)
        {
            //std::cout << "Compare with " << i << std::endl;
            const auto& compareTrajectory = compareAgainst[i];
            if (compareTrajectory.empty())
            {
                continue;
            }

            NT newFrechetValue;
            frechetDist.setUpperbound(minFrechetValue);
            auto result = frechetDist(trajectory.begin(), trajectory.end(), compareTrajectory.begin(),
                compareTrajectory.end(), newFrechetValue);
            if (result) {
                minFrechetValue = newFrechetValue;
                //std::cout << "New val: " << minFrechetValue << " at " << i << std::endl;
                bestTrajectory = i;
            }
            else
            {
                //std::cout << "Worse val" << std::endl;
            }
        }
        if ((currCompar + 1) * 100 / trajectorySet.size() > perc)
        {
            auto end = std::chrono::system_clock::now();

            const std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            auto* tm = std::localtime(&end_time);
            std::cout << "[" << std::put_time(tm,"%a %b %d, %T")  << "]:" << "Traj: " << (currTraj + 1) << "/" << trajectorySet.size() << ", at " << perc << "%" << std::endl;
            perc += percIncr;
        }
        ++currCompar;
        outputValues.push_back(minFrechetValue);
        setIndices.push_back(bestTrajectory);
        ++currTraj;
    }
}

void TrajectorySetFrechet::minFrechetValues(DS::EmbeddedGraph* graph, const std::vector<BasisElement>& trajectorySet,
    const std::vector<BasisElement>& compareAgainst, NT upperBound, std::vector<NT>& outputValues,
    std::vector<std::size_t>& setIndices)
{
    outputValues.reserve(trajectorySet.size());
    setIndices.reserve(trajectorySet.size());

    SFD frechetDist;
    using FrMode = decltype(frechetDist.mode());
    frechetDist.setMode(FrMode::BisectionSearch);
    frechetDist.setTolerance(m_tolerance);

    std::size_t currTraj = 0, currCompar = 0;
    int percIncr = 5;
    int perc = percIncr;
    for (const auto& trajectory : trajectorySet)
    {
        if (trajectory.empty())
        {
            continue;
        }
        NT minFrechetValue = upperBound;
        // Convert to Movetk trajectory
        std::vector<Point> movetkTrajectory;
        toMovetkTrajectory(trajectory, graph, movetkTrajectory);

        std::vector<Point> movetkCompareTrajectory;
        // Index of best trajectory
        std::size_t bestTrajectory = 0;
        for (auto i = 0; i < compareAgainst.size(); ++i)
        {
            const auto& compareTrajectory = compareAgainst[i];
            if (compareTrajectory.empty())
            {
                continue;
            }
            movetkCompareTrajectory.clear();
            toMovetkTrajectory(compareTrajectory, graph, movetkCompareTrajectory);

            NT newFrechetValue;
            frechetDist.setUpperbound(minFrechetValue);
            auto result = frechetDist(movetkTrajectory.begin(), movetkTrajectory.end(), movetkCompareTrajectory.begin(),
                movetkCompareTrajectory.end(), newFrechetValue);
            if (result) {
                minFrechetValue = newFrechetValue;
                bestTrajectory = i;
            }
        }
        if ((currCompar + 1) * 100 / trajectorySet.size() > perc)
        {
            auto end = std::chrono::system_clock::now();

            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            std::string timeString = std::ctime(&end_time);
            std::cout << "["<< timeString <<"]:" << "Traj: " << (currTraj + 1) << "/" << trajectorySet.size() << ", at " << perc << "%" << std::endl;
            perc += percIncr;
        }
        ++currCompar;
        outputValues.push_back(minFrechetValue);
        setIndices.push_back(bestTrajectory);
        ++currTraj;
    }
}

void TrajectorySetFrechet::frechetTable(DS::EmbeddedGraph* graph,
                                                           const std::vector<BasisElement>& trajectorySet,
                                                           const std::vector<BasisElement>& compareAgainst,
                                                           std::vector<std::vector<NT>>& outputValues)
{
    outputValues.resize(trajectorySet.size(), std::vector<NT>{});

    SFD frechetDist;
    frechetDist.setTolerance(m_tolerance);
    frechetDist.setUpperbound(1000);

    for (std::size_t i = 0; i < trajectorySet.size(); ++i)
    {
        const auto& trajectory = trajectorySet[i];
        // Convert to Movetk trajectory
        std::vector<Point> movetkTrajectory;
        toMovetkTrajectory(trajectory, graph, movetkTrajectory);

        std::vector<Point> movetkCompareTrajectory;
        for (const auto& compareTrajectory : compareAgainst)
        {
            movetkCompareTrajectory.clear();
            toMovetkTrajectory(compareTrajectory, graph, movetkCompareTrajectory);
            LoopsLib::NT val;
            
            auto result = frechetDist(movetkTrajectory.begin(), movetkTrajectory.end(), movetkCompareTrajectory.begin(),
                                      movetkCompareTrajectory.end(),val);
            outputValues[i].push_back(val);
        }
    }
}
