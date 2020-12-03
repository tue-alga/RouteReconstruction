#include <LoopsLib/Algs/NNLS.h>
#include "LoopsLib/Helpers/Timer.h"
using namespace LoopsLib;
using namespace LoopsLib::Algs;

class NNLSModel
{
    IloEnv m_env;
    IloModel m_model;
    IloNumVarArray m_coeffVars;
    IloExprArray m_fieldExpressions;
    const std::vector<NT>& m_field;
    std::size_t m_constraintComplexity = 0;
public:
    ~NNLSModel()
    {
        m_model.getEnv().end();
    }
    NNLSModel(const std::vector<NT>& field):
        m_model(m_env),
        m_coeffVars(m_env),
        m_field(field),
        m_fieldExpressions(m_env, field.size())
    {
        for (std::size_t i = 0; i < m_field.size(); ++i)
        {
            m_fieldExpressions[i] = IloExpr(m_env);
        }
    }

    std::size_t basisElementCount() const
    {
        return m_coeffVars.getSize();
    }
    std::size_t constraintComplexity() const
    {
        return m_constraintComplexity;
    }

    std::size_t addBasisElement(const std::vector<DS::BaseGraph::Id_t>& element)
    {
        IloNumVar coeff(m_env);
        m_coeffVars.add(coeff);
        m_model.add(coeff >= 0);
        for(const auto& el: element)
        {
            m_fieldExpressions[el] += coeff;
            ++m_constraintComplexity;
        }
        return m_coeffVars.getSize() - 1;
    }
    void addLeastSquaresGoal()
    {
        IloExprArray arr(m_env,m_fieldExpressions.getSize());
        for(auto i = 0; i < m_field.size(); ++i)
        {
            arr[i] = IloSquare(m_field[i] - m_fieldExpressions[i]);
        }
        m_model.add(IloMinimize(m_env, IloSum(arr)));
    }
    bool solve(bool disableOutput, std::vector<NT>& output, NT& objectiveValue, int numThreads=8)
    {
        auto env = m_model.getEnv();
        IloCplex cplex(m_model);
        cplex.setParam(IloCplex::Param::Threads, numThreads);
        // Disable output
        if (disableOutput) cplex.setOut(env.getNullStream());
        try
        {
            output.clear();
            output.assign(m_coeffVars.getSize(), 0);
            bool success = cplex.solve();
            if (!success)
            {
                std::cout << "[NNLS] Cplex solve failed" << std::endl;
                return false;
            }
            for (int i = 0; i < m_coeffVars.getSize(); i++)
            {
                output[i] = cplex.getValue(m_coeffVars[i]);
            }
            objectiveValue = cplex.getObjValue();
        }
        catch (IloException& e)
        {
            std::cout << "[NNLS] ILOException: " << e.getMessage() << std::endl;
            cplex.end();
            return false;
        }
        cplex.end();
        return true;
    }
};

bool NNLS::solveModel(IloNumVarArray& coeffVars, IloModel& model, std::vector<NT>& output, NT& objectiveResult)
{
    auto env = model.getEnv();
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::Threads, m_numberOfThreads);
    if(m_useCoeffsAsStart && coeffVars.getSize() == output.size())
    {
        cplex.setParam(IloCplex::Param::Advance, 1);
        IloNumArray arr(env);
        for(const auto& el: output)
        {
            arr.add(el);
        }
        // Initialize with start
        cplex.setStart(arr, {}, coeffVars, {}, {}, {});
    }
    // Disable output
    if (m_disableOutput) cplex.setOut(env.getNullStream());
    try
    {
        output.clear();
        output.assign(coeffVars.getSize(), 0);
        bool success = cplex.solve();
        if (!success)
        {
            std::cout << "[NNLS] Cplex solve failed" << std::endl;
            return false;
        }
        for (int i = 0; i < coeffVars.getSize(); i++)
        {
            output[i] = cplex.getValue(coeffVars[i]);
        }
        objectiveResult = cplex.getObjValue();
    }
    catch (IloException& e)
    {
        std::cout << "[NNLS] ILOException: " << e.getMessage() << std::endl;
        cplex.end();
        env.end();
        return false;
    }
    cplex.end();
    env.end();
    return true;
}

bool NNLS::solveModel(IloNumVarArray& coeffVars, IloExprArray& fieldExpressions, const std::vector<NT>& field, IloModel& model,
    std::vector<NT>& output, NT& objectiveResult)
{
    for (std::size_t i = 0; i < field.size(); ++i)
    {
        fieldExpressions[i] = IloSquare(fieldExpressions[i] - field[i]);
    }

    // minimize squares
    model.add(IloMinimize(model.getEnv(), IloSum(fieldExpressions)));

    return solveModel(coeffVars, model, output, objectiveResult);
}

void NNLS::setup(long long numberOfEdges, int basisSize, IloEnv& env, IloModel& model, IloNumVarArray& coeffVar,
                 IloExprArray& fieldExpressions)
{
    model = IloModel(env);

    // Coefficients for the bases
    coeffVar = IloNumVarArray(env);
    for (int i = 0; i < basisSize; i++)
    {
        IloNumVar var(env);
        coeffVar.add(var);
        model.add(var >= 0);
    }
    fieldExpressions = IloExprArray(env, numberOfEdges);
    for (std::size_t i = 0; i < numberOfEdges; ++i)
    {
        fieldExpressions[i] = IloExpr(env);
    }
}

NNLS::~NNLS()
{
    //m_env.end();
}

NNLS::NNLS() : m_stream(&std::cout)
{
}

void NNLS::setOutStream(std::ostream& stream)
{
    m_stream = &stream;
}

void NNLS::setDisableOutput(bool value)
{
    m_disableOutput = value;
}

bool NNLS::useCoeffsAsStart() const
{
    return m_useCoeffsAsStart;
}

void NNLS::setUseCoeffsAsStart(bool val)
{
    m_useCoeffsAsStart = val;
}

void NNLS::setMaxGreedySteps(int steps)
{
    m_maxGreedySteps = steps;
}

bool NNLS::solve(const std::vector<std::vector<DS::BaseGraph::Edge*>>& basis, const std::vector<NT>& field,
                       std::vector<NT>& output, NT& objectiveResult)
{
    IloEnv m_env;
    IloModel model(m_env);
    objectiveResult = std::numeric_limits<double>::max();

    // Coefficients for the bases
    IloNumVarArray coefficientVars(m_env);
    const auto matRows = basis.size();
    for (int i = 0; i < matRows; i++)
    {
        IloNumVar var(m_env);
        coefficientVars.add(var);
        model.add(var >= 0);
    }
    IloExprArray fieldExpressions(m_env, field.size());
    for (std::size_t i = 0; i < field.size(); ++i)
    {
        fieldExpressions[i] = IloExpr(m_env);
    }


    // Construct field objective
    for (int i = 0; i < basis.size(); i++)
    {
        for (auto* e : basis[i])
        {
            fieldExpressions[e->id()] += coefficientVars[i];
        }
    }
    for (std::size_t i = 0; i < field.size(); ++i)
    {
        fieldExpressions[i] = IloSquare(fieldExpressions[i] - field[i]);
    }

    // minimize squares
    model.add(IloMinimize(m_env, IloSum(fieldExpressions)));

    return solveModel(coefficientVars, model, output, objectiveResult);
}

bool NNLS::solve(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field,
    std::vector<NT>& output, NT& objectiveResult)
{
    IloEnv m_env;
    IloModel model(m_env);
    objectiveResult = std::numeric_limits<double>::max();

    // Coefficients for the bases
    IloNumVarArray coefficientVars(m_env);
    const auto matRows = basis.size();
    for (int i = 0; i < matRows; i++)
    {
        IloNumVar var(m_env);
        coefficientVars.add(var);
        model.add(var >= 0);
    }
    IloExprArray fieldExpressions(m_env, field.size());
    for (std::size_t i = 0; i < field.size(); ++i)
    {
        fieldExpressions[i] = IloExpr(m_env);
    }

    // Get the expansion expression
    IloExpr expr(m_env);


    std::cout << "[NNLS] Building field expressions" << std::endl;
    // Construct field objective
    for (int i = 0; i < basis.size(); i++)
    {
        for (const auto& eId : basis[i])
        {
            fieldExpressions[eId] += coefficientVars[i];
        }
    }
    for (std::size_t i = 0; i < field.size(); ++i)
    {
        fieldExpressions[i] = IloSquare(fieldExpressions[i] - field[i]);
    }

    // minimize squares
    model.add(IloMinimize(m_env, IloSum(fieldExpressions)));


    std::cout << "[NNLS] Model built. Starting run" << std::endl;

    return solveModel(coefficientVars, model, output, objectiveResult);
}

bool NNLS::solveBatched(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field,
    std::size_t batchSize, std::vector<NT>& output, NT& objectiveResult)
{
    IloEnv m_env;
    IloModel model(m_env);
    objectiveResult = std::numeric_limits<double>::max();
    // Coefficients for the bases. Incrementally add variables for the batches
    IloNumVarArray coefficientVars(m_env);

    const char* objectiveName = "OBJECTIVE_EXPR";


    IloExprArray fieldExpressions(m_env, field.size());
    for (std::size_t i = 0; i < field.size(); ++i)
    {
        fieldExpressions[i] = IloExpr(m_env);
    }
    IloExprArray sqDiffsExpressions(m_env, field.size());

    IloExtractable objective;

    IloCplex cplex(model);

    std::size_t constraintsCount = 0;

    auto iterations = (basis.size() + batchSize) / batchSize;
    for(auto i = 0; i < iterations; ++i)
    {
        // Add remainder
        auto bound = batchSize;
        if(i == iterations-1) bound = basis.size() - i * batchSize;

        // Add new vars
        for (auto j = 0; j < bound; ++j)
        {
            IloNumVar var(m_env);
            coefficientVars.add(var);
            model.add(var >= 0);
        }
        // Add new edge flow coefficients
        for (int j = i * batchSize; j < i * batchSize + bound; ++j)
        {
            for (const auto& eId : basis[j])
            {
                fieldExpressions[eId] += coefficientVars[j];
                ++constraintsCount;
            }
        }
        std::cout << "[NNLS] Starting batch solve " << (i + 1) << "/" << iterations << ", batch size " << batchSize << ", constraints complexity: ~" << constraintsCount << std::endl;
        // Update square differences
        for (std::size_t j = 0; j < field.size(); ++j)
        {
            sqDiffsExpressions[j] = IloSquare(field[j] - fieldExpressions[j]);
        }
        // Remove old minimization goal
        if(i != 0)
        {
            model.remove(objective);
        }
        // Add new minimization goal
        objective = model.add(IloMinimize(m_env, IloSum(sqDiffsExpressions), objectiveName));

        cplex.setParam(IloCplex::Param::Threads, m_numberOfThreads);
        // Start with solution to previous problem
        if (i != 0)
        {
            cplex.setParam(IloCplex::Param::Advance, 1);
            IloNumArray arr(m_env);
            for (auto j = 0; j < i * batchSize; ++j)
            {
                arr.add(output[j]);
            }
            // Initialize with start
            cplex.setStart(arr, {}, coefficientVars, {}, {}, {});
        }
        // Disable output
        if (m_disableOutput) cplex.setOut(m_env.getNullStream());
        try
        {
            output.clear();
            output.assign(coefficientVars.getSize(), 0);
            bool success = cplex.solve();
            if (!success)
            {
                std::cout << "[NNLS] Cplex solve failed" << std::endl;
                return false;
            }
            for (int i = 0; i < coefficientVars.getSize(); i++)
            {
                output[i] = cplex.getValue(coefficientVars[i]);
            }
            objectiveResult = cplex.getObjValue();
        }
        catch (IloException& e)
        {
            std::cout << "[NNLS] ILOException: " << e.getMessage() << std::endl;
            cplex.end();
            m_env.end();
            return false;
        }
    }
    cplex.end();
    m_env.end();

    //// Filter on prune value
    //// TODO: do this intermediately? May result in worse decomposition though
    //for(auto i = 0; i < output.size(); ++i)
    //{
    //    if(output[i] > pruneValue)
    //    {
    //        retainSet.push_back(i);
    //    }
    //}
    //std::vector<LoopsLib::NT> outputFiltered;
    //for(auto el : retainSet)
    //{
    //    outputFiltered.push_back(output[el]);
    //}
    //output = outputFiltered;

    return true;
}

bool NNLS::solveIncrementally(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field,
    std::size_t batchSize, std::size_t retainSize, bool retainFraction, std::vector<std::size_t>& retainedIndices,
    std::vector<NT>& output, NT& objectiveResult)
{
    std::size_t batches = basis.size() / batchSize;
    if (basis.size() % batchSize > 0) ++batches;

    const auto batchFraction = retainSize / batches;
    auto rest = retainSize - batches * batchFraction;

    if (!m_disableOutput) std::cout << "[NNLS] Batch solving basis of size " << basis.size() << " in " << batches  << " steps, retain per step: " << batchFraction << std::endl;

    // To retain during iteration
    std::size_t toRetain = 0;

    for(auto i = 0; i < batches; ++i)
    {
        if (!m_disableOutput) std::cout << "[NNLS] Starting batch " << (i + 1) << "/" << batches << std::endl;

        std::vector<NT> newCoeffs;
        NNLSModel model(field);
        for(const auto& el: retainedIndices)
        {
            model.addBasisElement(basis[el]);
        }
        auto batchStart = batchSize * i;
        // Quit if we are out of bounds
        if (batchStart >= basis.size()) break;

        // Add batch elements
        for(auto j = i * batchSize; j < std::min(basis.size(), (i+1)*batchSize); ++j)
        {
            model.addBasisElement(basis[j]);
            retainedIndices.push_back(j);
        }
        // Add the goal
        model.addLeastSquaresGoal();
        // If we fail, return fail
        if (!model.solve(m_disableOutput, newCoeffs, objectiveResult, m_numberOfThreads)) {
            if (!m_disableOutput) std::cout << "[NNLS] Subsolve failed for basis size " << model.basisElementCount() << ", constraint complexityy " <<  model.constraintComplexity() << " failed" << std::endl;
            return false;
        }
        // Sort by best coeffs
        if (!m_disableOutput) std::cout << "[NNLS] Sorting solution" << std::endl;

        std::vector<std::size_t> indices(retainedIndices.size(), 0);
        std::iota(indices.begin(), indices.end(), 0);
        // Sort by highest coefficient
        std::sort(indices.begin(), indices.end(), [newCoeffs](const auto& i0, const auto& i1) {return newCoeffs[i0] > newCoeffs[i1]; });
        // Take the top
        if(retainFraction)
        {
            toRetain += batchFraction;
            if (rest > 0)
            {
                ++toRetain;
                rest--;
            }
        }
        else
        {
            toRetain = retainSize;
        }
        if (!m_disableOutput) std::cout << "[NNLS] Retaining: " << retainSize << std::endl;
        // Retain best
        indices.resize(std::min(toRetain, indices.size()));
        std::vector<std::size_t> newIndices;
        std::transform(indices.begin(), indices.end(), std::back_inserter(newIndices), [&retainedIndices](const auto& ind) {return retainedIndices[ind]; });
        retainedIndices = newIndices;
        // Update coefficients
        auto ti = LoopsLib::Helpers::Iterators::transform_iterator(newIndices, [newCoeffs](const std::size_t& ind) {return newCoeffs[ind]; });
        output.assign(ti, ti.associatedEnd());
        if (!m_disableOutput) std::cout << "[NNLS] End of batch basis size: " << retainedIndices.size() << std::endl;
    }
    if (!m_disableOutput) std::cout << "[NNLS] End of batch basis size: " << retainedIndices.size() << std::endl;
    return true;
}

void NNLS::greedyRefine(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis, const std::vector<NT>& field,
    std::vector<NT>& coefficients, NT& objectiveValue)
{
    if(coefficients.size() != basis.size())
    {
        throw std::runtime_error("Coeffficient and basis incongruency:coeffs=" + std::to_string(coefficients.size()) + ",basis=" + std::to_string(basis.size()));
    }
    if(coefficients.empty())
    {
        std::cout << "[NNLS] No coefficients and basis!" << std::endl;
    }
    auto minEl = std::min_element(coefficients.begin(), coefficients.end());
    auto maxEl = std::max_element(coefficients.begin(), coefficients.end());
    std::cout << "[NNLS] Basis size " << basis.size() << std::endl;
    std::cout << "[NNLS] Initial min-max pf coefficients: " << *minEl << "," << *maxEl << std::endl;

    if(field.size() == 0)
    {
        throw std::runtime_error("Empty field");
    }
    for(const auto& el: field)
    {
        if (std::isnan(el)) throw std::runtime_error("Nan in field!");
        if (std::isinf(el)) throw std::runtime_error("Inf in field!");
    }
    // Copy field
    std::vector<NT> residual(field.begin(), field.end());
    if(residual.size() != field.size())
    {
        throw std::runtime_error("Residual not the same size as field: resid=" + std::to_string(residual.size()) + ",field=" + std::to_string(field.size()));
    }
    std::cout << "[NNLS] Constructing residual" << std::endl;
    // Subtract basis elements first
    for(auto i =0; i < basis.size(); ++i)
    {
        const NT basisCoeff = coefficients[i];
        if (std::isnan(basisCoeff)) throw std::runtime_error("Nan coefficient");
        for(const auto& el: basis[i])
        {
            if(el >= residual.size() || el < 0)
            {
                throw std::runtime_error("Basis el " + std::to_string(el) + " out of bounds,resid=" + std::to_string(residual.size()));
            }
            residual[el] -= basisCoeff;
        }
    }
    const NT initialCost = std::accumulate(residual.begin(), residual.end(), (NT)0.0, [](const NT& acc, const NT& val)
    {
        if (std::isnan(val)) throw std::runtime_error("Nan in residual");
        return acc + val * val;
    });
    std::cout << "[NNLS] Initial cost: " << initialCost << std::endl;
    std::size_t refineCount = 0;
    LoopsLib::Helpers::Timer timer;
    timer.start();

    for(auto step = 0; step < m_maxGreedySteps; ++step)
    {
        // Refine step
        for (auto i = 0; i < basis.size(); ++i)
        {
            // Determine whether we can reduce the cost by adding more of this basis element
            std::map<DS::BaseGraph::Id_t, LoopsLib::NT> multiplicities;
            LoopsLib::NT weightedResidual = 0.0;
            for (auto el : basis[i])
            {
                if (multiplicities.find(el) == multiplicities.end()) multiplicities[el] = 0;
                multiplicities[el] += 1.0;

            }
            LoopsLib::NT sqMultiplicity = 0.0;
            for (auto pair : multiplicities)
            {
                // Sum of multiplicity of edge times residual weight. Add in coefficient 
                weightedResidual += residual[pair.first] * pair.second;
                sqMultiplicity += pair.second*pair.second;
            }
            LoopsLib::NT coeff = weightedResidual / sqMultiplicity;

            // Enforce non-negativity.
            if (coefficients[i] + coeff < 0) continue;

            // Can improve
            if (coeff*coeff * sqMultiplicity - 2.0 * coeff * weightedResidual < 0)
            {
                // Add to coeff
                coefficients[i] = coeff;
                // Subtract from residual
                for (const auto& el : basis[i])
                {
                    residual[el] -= coeff;
                }
                ++refineCount;
            }
        }
        std::cout << "[NNLS] Refined cost after step " << step << ": " << std::accumulate(residual.begin(), residual.end(), (NT)0.0, [](const auto& acc, const auto& val)
        {
            return acc + val * val;
        }) << std::endl;

        // Stop after fixed time
        if (timer.elapsedMs() * 1e-3 > m_maxGreedyTime) break;
    }
    
    std::cout << "[NNLS] Refined " << refineCount << "elements" << std::endl;
    objectiveValue = std::accumulate(residual.begin(), residual.end(), (NT)0.0, [](const auto& acc, const auto& val)
    {
        return acc + val * val;
    });
    std::cout << "[NNLS] Greedily improved cost from " << initialCost << " to " << objectiveValue << ", diff: " << (initialCost - objectiveValue) << std::endl;

}

bool NNLS::solve(const std::vector<std::vector<DS::BaseGraph::Id_t>>& basis,
    const std::map<DS::BaseGraph::Id_t, NT>& field, std::vector<NT>& output, NT& objectiveResult)
{
    IloEnv m_env;
    IloModel model(m_env);
    objectiveResult = std::numeric_limits<double>::max();

    // Coefficients for the bases
    IloNumVarArray coefficientVars(m_env);
    const auto matRows = basis.size();
    // Non-negativity on basis
    for (int i = 0; i < matRows; i++)
    {
        IloNumVar var(m_env);
        coefficientVars.add(var);
        model.add(var >= 0);
    }
    // Add field expressions
    IloExprArray fieldExpressions(m_env, field.size());
    std::map<DS::BaseGraph::Id_t, int> fieldElToIndex;
    {
        std::size_t i = 0; 
        for (const auto& pair : field)
        {
            fieldExpressions[i] = IloExpr(m_env);
            fieldElToIndex[pair.first] = i;
            ++i;
        }
    }
    

    // Get the expansion expression
    IloExpr expr(m_env);

    // Construct field objective
    for (int i = 0; i < basis.size(); i++)
    {
        for (const auto& eId : basis[i])
        {
            if(fieldElToIndex.find(eId) != fieldElToIndex.end())
            {
                fieldExpressions[fieldElToIndex[eId]] += coefficientVars[i];
            }
            
        }
    }
    // Construct square difference elements
    for (const auto& pair : field)
    {
        const auto ind = fieldElToIndex[pair.first];
        fieldExpressions[ind] = IloSquare(fieldExpressions[ind] - pair.second);
    }

    // minimize squares
    model.add(IloMinimize(m_env, IloSum(fieldExpressions)));

    return solveModel(coefficientVars, model, output, objectiveResult);
}

bool NNLS::solve(const Eigen::MatrixXd& basis, const Eigen::VectorXd& field, Eigen::VectorXd& output,
                       double& objectiveResult)
{
    IloEnv m_env;
    IloModel model(m_env);
    objectiveResult = std::numeric_limits<double>::max();

    // Non-negativity contraint
    IloNumVarArray arr(m_env);
    const int matRows = basis.rows();
    for (int i = 0; i < matRows; i++)
    {
        IloNumVar var(m_env);
        arr.add(var);
        model.add(var >= 0);
    }
    // Get the expansion expression
    IloExpr expr(m_env);
    for (int i = 0; i < basis.cols(); i++)
    {
        IloExpr subExpr(m_env);
        for (int j = 0; j < matRows; j++)
        {
            subExpr += basis(j, i) * arr[j];
        }
        subExpr = IloSquare(subExpr - field(i));
        expr += subExpr;
    }

    // minimize squares
    model.add(IloMinimize(m_env, expr));


    IloCplex cplex(model);
    // Disable output
    //cplex.setOut(m_env.getNullStream());
    try
    {
        output.setConstant(matRows, 0);
        bool success = cplex.solve();
        if (!success)
        {
            std::cout << "[NNLS] Cplex solve failed" << std::endl;
            return false;
        }
        for (int i = 0; i < matRows; i++)
        {
            output(i) = cplex.getValue(arr[i]);
        }
        objectiveResult = cplex.getObjValue();
    }
    catch (IloException& e)
    {
        std::cout << "[NNLS] ILOExcpetion: " << e.getMessage() << std::endl;
        cplex.end();
        m_env.end();
        return false;
    }
    cplex.end();
    m_env.end();
    return true;
}
