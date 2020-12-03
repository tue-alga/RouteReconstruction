#include <LoopsAlgs/Trajectories/Coverage.h>
#include <movetk/algo/Similarity.h>
#include <ilcplex/ilocplexi.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
struct Norm
{
    using Point = LoopsLib::MovetkGeometryKernel::MovetkPoint;
    using NT = LoopsLib::MovetkGeometryKernel::NT;
    NT operator()(const Point& p0, const Point& p1) const
    {
        return (p0 - p1).length();
    }
};

void LoopsAlgs::Trajectories::Coverage::computeFrechetTable(
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& centers,
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& coverSet, const std::string& cachePath,
    std::vector<std::vector<LoopsLib::NT>>& table) const
{
    if(!cachePath.empty())
    {
        std::ifstream readStream(cachePath);
        if(readStream.is_open())
        {
            // Read table
            boost::archive::binary_iarchive loadArch(readStream);
            loadArch >> table;
            return;
        }
    }
    const bool convertCrs = !centers.m_ref.IsSame(&coverSet.m_ref);

    std::cout << "[Coverage] Computing Frechet table" << std::endl;
    movetk_algorithms::StrongFrechetDistance<LoopsLib::MovetkGeometryKernel, Norm> frechet;
    using FrMode = decltype(frechet.mode());
    frechet.setMode(FrMode::DoubleAndSearch);
    frechet.setTolerance(m_frechetTolerance);
    table.resize(centers.size(), std::vector<LoopsLib::NT>{});
    for(int i = 0; i < centers.size(); ++i)
    {
        for(int j = 0; j < coverSet.size(); ++j)
        {
            LoopsLib::NT out = 0.0;
            bool success = false;
            if(convertCrs)
            {
                LoopsLib::MovetkGeometryKernel::Trajectory converted = centers.trajectories[i];
                LoopsLib::MovetkGeometryKernel().convertCRS(centers.m_ref, coverSet.m_ref, converted);
                success = frechet(converted.cbegin(), converted.cend(),
                    coverSet.trajectories[j].begin(), coverSet.trajectories[j].end(), out);
            }
            else
            {
                success = frechet(centers.trajectories[i].begin(), centers.trajectories[i].end(),
                    coverSet.trajectories[j].begin(), coverSet.trajectories[j].end(), out);
            }
            
            if (success)
                table[i].push_back(out);
            else
                throw std::runtime_error("Frechet distance failed...");
            std::cout << "[Coverage] Computed " << (i + 1) << "/" << (centers.size()) << ", sub " << (j + 1) << "/" << coverSet.size() << ":" << out << std::endl;
        }
    }
    if(!cachePath.empty())
    {
        std::ofstream writeStream(cachePath);
        if (writeStream.is_open())
        {
            // Read table
            boost::archive::binary_oarchive loadArch(writeStream);
            loadArch << table;
        }
    }
}

LoopsAlgs::Trajectories::Coverage::Mode LoopsAlgs::Trajectories::Coverage::mode() const
{
    return m_mode;
}

void LoopsAlgs::Trajectories::Coverage::setMode(Mode mode)
{
    m_mode = mode;
}

void LoopsAlgs::Trajectories::Coverage::setFrechetTolerance(const LoopsLib::NT& frechetTolerance)
{
    m_frechetTolerance = frechetTolerance;
}

LoopsLib::NT LoopsAlgs::Trajectories::Coverage::frechetTolerance() const
{
    return m_frechetTolerance;
}

void LoopsAlgs::Trajectories::Coverage::setFractionPrecision(const LoopsLib::NT& fractionPrecision)
{
    m_fractionPrecision = fractionPrecision;
}

LoopsLib::NT LoopsAlgs::Trajectories::Coverage::fractionPrecision() const
{
    return m_fractionPrecision;
}


bool LoopsAlgs::Trajectories::Coverage::compute(
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& centers, 
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& coverSet,
    const std::string& cachePath,
    LoopsLib::NT epsilon, LoopsLib::NT& fraction) const
{
    // Compute table of connections that are less than epsilon
    std::vector<std::vector<bool>> frechetAccessTable;
    std::vector<std::vector<LoopsLib::NT>> frechetDistTable;

    computeFrechetTable(centers, coverSet, cachePath, frechetDistTable);
    frechetAccessTable.resize(frechetDistTable.size(), std::vector<bool>{});


    std::cout << "[Coverage] Solving integer problem" << std::endl;
    // Compute exact minimum coverage for given epsilon
    IloEnv env;
    IloModel model(env);

    // Setup a bool variable per trajectory element
    IloBoolVarArray arr(env);
    IloExpr sumVar(env);
    for (auto i = 0; i < centers.size(); ++i)
    {
        IloBoolVar var(env);
        arr.add(var);
        sumVar += var;
    }
    // Add total count constraint
    model.add(IloMinimize(env, sumVar));

    // Add cover constraints
    for (auto j = 0; j < coverSet.size(); ++j)
    {
        // Construct an expression such that either the trajectory itself is picked
        // or it is covered by one of the trajectories that are <= epsilon away.
        IloExpr localTotal(env);
        for (int i = 0; i < centers.size(); ++j)
        {
            if (frechetDistTable[i][j] <= epsilon) localTotal += arr[i];
        }
        model.add(localTotal >= 1);
    }
    std::cout << "[Coverage] Model built, applying solve" << std::endl;
    // Solve the model
    IloCplex cplex(model);
    //cplex.setParam(IloCplex::Param::Threads, m_numberOfThreads);
    // Disable output
    //if (m_disableOutput) cplex.setOut(env.getNullStream());
    bool success = false;
    try
    {
         success = cplex.solve();
        if (!success)
        {
            std::cout << "[Coverage] No feasible solution" << std::endl;
        }
        else
        {
            fraction = cplex.getObjValue() / (LoopsLib::NT)centers.size();
        }
    }
    catch (IloException& e)
    {
        std::cout << "[Coverage] ILOException: " << e.getMessage() << std::endl;
    }
    env.end();
    std::cout << "[Coverage] Done" << std::endl;
    return success;
}

bool LoopsAlgs::Trajectories::Coverage::computeUpperBounded(
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& centers,
    const LoopsLib::MovetkGeometryKernel::TrajectorySet& coverSet, const std::string& cachePath, NT epsilon,
    NT& fraction) const
{
    // Compute table of connections that are less than epsilon
    std::map<std::size_t,std::vector<std::pair<std::size_t,LoopsLib::NT>>> frechetDistTable;

    auto pushEpsilon = [&frechetDistTable](std::size_t covered, std::size_t center, NT epsilon)
    {
        if (frechetDistTable.find(covered) == frechetDistTable.end()) frechetDistTable[covered] = {};
        frechetDistTable[covered].push_back(std::make_pair(center, epsilon));
    };

    // Do we need to convert reference systems
    const bool convertCrs = !centers.m_ref.IsSame(&coverSet.m_ref);

    movetk_algorithms::StrongFrechetDistance<LoopsLib::MovetkGeometryKernel, Norm> frechet;
    using FrMode = decltype(frechet.mode());
    frechet.setMode(FrMode::BisectionSearch);
    frechet.setTolerance(m_frechetTolerance);
    frechet.setUpperbound(epsilon);
    std::set<std::size_t> centersWithConnection;
    for(int i = 0; i < coverSet.size(); ++i)
    {
        NT out = 0.0;
        bool hasDist = false;
        for(int j = 0; j < centers.size(); ++j)
        {
            bool success = false;
            if (convertCrs)
            {
                LoopsLib::MovetkGeometryKernel::Trajectory converted = centers.trajectories[j];
                LoopsLib::MovetkGeometryKernel().convertCRS(centers.m_ref, coverSet.m_ref, converted);
                success = frechet(converted.cbegin(), converted.cend(),
                    coverSet.trajectories[i].begin(), coverSet.trajectories[i].end(), out);
            }
            else
            {
                success = frechet(centers.trajectories[j].begin(), centers.trajectories[j].end(),
                    coverSet.trajectories[i].begin(), coverSet.trajectories[i].end(), out);
            }
            if (!success) continue;
            std::cout << "[Coverage] Computed " << (i + 1) << "/" << (centers.size()) << ", sub " << (j + 1) << "/" << coverSet.size() << ":" << out << std::endl;
            pushEpsilon(i, j, out);
            centersWithConnection.insert(j);
            hasDist = true;
        }
        if(!hasDist)
        {
            std::cout << "[Coverage] Could not cover " << i << " within epsilon " << epsilon << std::endl;
            return false;
        }
    }

    std::cout << "[Coverage] Solving integer problem" << std::endl;
    // Compute exact minimum coverage for given epsilon
    IloEnv env;
    IloModel model(env);

    std::map<std::size_t, IloBoolVar> centersMap;

    // Setup a bool variable per trajectory element
    IloBoolVarArray arr(env);
    IloExpr sumVar(env);
    for (const auto& center: centersWithConnection)
    {
        IloBoolVar var(env);
        arr.add(var);
        sumVar += var;
        centersMap[center] = var;
    }
    // Add total count constraint
    model.add(IloMinimize(env, sumVar));

    // Add cover constraints
    for (auto j = 0; j < coverSet.size(); ++j)
    {
        // Construct an expression such that either the trajectory itself is picked
        // or it is covered by one of the trajectories that are <= epsilon away.
        IloExpr localTotal(env);
        for (const auto& conn : frechetDistTable[j])
        {
            localTotal += centersMap[conn.first];
        }
        model.add(localTotal >= 1);
    }
    std::cout << "[Coverage] Model built, applying solve" << std::endl;
    // Solve the model
    IloCplex cplex(model);
    //cplex.setParam(IloCplex::Param::Threads, m_numberOfThreads);
    // Disable output
    //if (m_disableOutput) cplex.setOut(env.getNullStream());
    bool success = false;
    try
    {
        success = cplex.solve();
        if (!success)
        {
            std::cout << "[Coverage] No feasible solution" << std::endl;
        }
        else
        {
            fraction = cplex.getObjValue() / (LoopsLib::NT)centers.size();
        }
    }
    catch (IloException& e)
    {
        std::cout << "[Coverage] ILOException: " << e.getMessage() << std::endl;
    }
    env.end();
    std::cout << "[Coverage] Done" << std::endl;
    return success;
}
