#include <LoopsAlgs/MapMatching/StrongFrechetMapMatching.h>
#include <iostream>

void LoopsAlgs::MapMatching::StrongFrechetMapMatching::apply(const Frechet::Trajectory& trajectory,
                                                             Frechet::NT lowerBound, Frechet::NT precision,
                                                             std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>&
                                                             mapmatchedPath,
                                                             Frechet::NT& epsilon)
{
    using NT = Frechet::NT;
    NT currentLower = 0;
    NT currentUpper = lowerBound;
    NT lastValue = 0;
    bool lastSuccess = false;

    int exponentialIt = 0;

    auto logger = LoopsLib::Helpers::logFactory(this);

    // Exponential search
    while(true)
    {
        Frechet::FrechetOnGraph frechetMapMatch(m_graph);
        mapmatchedPath.clear();
        logger.info("Computing exponential step with ", currentUpper);
        frechetMapMatch.compute(trajectory, currentUpper, mapmatchedPath);
        if (!mapmatchedPath.empty())
        {
            if(exponentialIt != 0)
            {
                currentLower = 0.5 * currentUpper;
            }
            logger.info("Found lower and upper range ",currentLower,",", currentUpper);
            break;
        }
        currentUpper *= 2.0;
        ++exponentialIt;
    }

    while (currentUpper - currentLower > precision)
    {
        lastValue = 0.5 * (currentLower + currentUpper);
        logger.info("Binary searching with ", lastValue);
        Frechet::FrechetOnGraph frechetMapMatch(m_graph);
        mapmatchedPath.clear();
        frechetMapMatch.compute(trajectory, lastValue, mapmatchedPath);
        if (!mapmatchedPath.empty())
        {
            currentUpper = lastValue;
            lastSuccess = true;
        }
        else
        {
            currentLower = lastValue;
            lastSuccess = false;
        }
    }

    epsilon = currentUpper;

    // Last was not reachable, so recompute the mapmatched path with the known best epsilon
    if (!lastSuccess)
    {
        Frechet::FrechetOnGraph frechetMapMatch(m_graph);
        mapmatchedPath.clear();
        frechetMapMatch.compute(trajectory, epsilon, mapmatchedPath);
        assert(!mapmatchedPath.empty());
    }
}

bool LoopsAlgs::MapMatching::StrongFrechetMapMatching::applyFixed(const Frechet::Trajectory& trajectory, Frechet::NT epsilon,
    std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>& mapmatchedPath)
{
    using NT = Frechet::NT;

    auto logger = LoopsLib::Helpers::logFactory(this);
    Frechet::FrechetOnGraph frechetMapMatch(m_graph);
    mapmatchedPath.clear();
    frechetMapMatch.compute(trajectory, epsilon, mapmatchedPath);
    return !mapmatchedPath.empty();
}
