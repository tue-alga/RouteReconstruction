#include <LoopsLib/Algs/LongestPath/LongestPathILP.h>
#include <LoopsLib/Helpers/CplexHelpers.h>

using namespace LoopsLib;
using namespace LoopsLib::Algs::LongestPath;

class LazyConstraintHandler : public IloCplex::LazyConstraintCallbackI
{
	DS::BaseGraph* m_graph;
	int m_source, m_sink;
	IloBoolVarArray& m_vars;
    bool m_verbose = true;
public:
	explicit LazyConstraintHandler(DS::BaseGraph* graph,
		int source,
		int sink,
		IloBoolVarArray& vars,
		const IloEnv& env) :
		LazyConstraintCallbackI(env),m_graph(graph), m_source(source), m_sink(sink), m_vars(vars){}

    void setVerbosity(bool value)
	{
        m_verbose = value;
	}

protected:

	/**
	 * \brief The main callback function
	 */
	void main() override
	{
		std::cout << "[LazyConstraint] Start, src/sink: " << m_source << '/' << m_sink << std::endl;
		// Determine whether or not we have a single connected component
        const auto* neighs = &m_graph->vertex(m_source)->m_outEdges;
		bool done = false;
		int pathCount = 0;
		std::set<int> path;
		while(!done)
		{
			for(auto e: *neighs)
			{
                auto eId = e->id();
				auto var = m_vars[eId];
				if(this->getValue(var) > 0.5)
				{
					path.insert(eId);
					++pathCount;
					if(e->m_sink->id() == m_sink)
					{
						done = true;
						break;
					}
                    neighs = &e->m_sink->m_outEdges;
					break;
				}
			}
		}
        if (m_verbose) {
            std::cout << "[LazyConstraint] Counting edges" << std::endl;
        }
		// Count total edges
		int edgeCount = 0;
		std::set<int> cycles;
		for(int i = 0; i < m_graph->number_of_edges(); ++i)
		{
			auto var = m_vars[i];
			if (this->getValue(var) > 0.5) {
				++edgeCount;
				// Insert into cycles.
				if(path.find(i) == path.end()) cycles.insert(i);
			}
		}
		if(edgeCount != pathCount)
		{
            if (m_verbose) {
                std::cout << "[LazyConstraint] EdgeCount vs pathCount: " << edgeCount << " , " << pathCount << std::endl;
                std::cout << "[LazyConstraint] Setting up constraint" << std::endl;
            }
			// Constrain solution to not include the separate path and the loops.
			IloExpr cyclesExpr(getEnv());
			//TODO reject combination...
			for(auto el : cycles)
			{
				cyclesExpr = cyclesExpr + m_vars[el];
			}
            if (m_verbose) {
                std::cout << "[LazyConstraint] Sum done" << std::endl;
            }
			IloConstraint constr;
			constr = cyclesExpr < cycles.size();
			this->add(constr);
            if (m_verbose) {
                std::cout << "[LazyConstraint] Added cycle constraint" << std::endl;
            }
		}
        if (m_verbose) {
            std::cout << "[LazyConstraint] End" << std::endl;
        }
	}

	CallbackI* duplicateCallback() const override
	{
		return (new(getEnv()) LazyConstraintHandler(*this));
	}
};



void Algs::LongestPath::LongestPathILP::setupSimpleConstraints(IloBoolVarArray& vars, IloModel& target) const
{
	for (int i = 0; i < m_graph->number_of_vertices(); i++)
	{
        auto* vert = m_graph->vertex(i);

		if (i == m_source)
		{
			IloExpr expr(target.getEnv());
            
			for (auto n : vert->m_outEdges)
			{
				expr += vars[*n];
			}
			target.add(expr == 1);

            // These can never be present 
			for (auto n : vert->m_inEdges)
			{
                target.add(vars[*n] == 0);
			}
		}
		else if (i == m_sink)
		{
			IloExpr expr(target.getEnv());
			for (auto n : vert->m_inEdges)
			{
				expr += vars[*n];
			}
			target.add(expr == 1);

            // Eliminate outgoing
			for (auto n : vert->m_outEdges)
			{
                target.add(vars[*n] == 0);
			}
		}
		else
		{
			IloExpr sumIn(target.getEnv());
			for (auto n : vert->m_inEdges)
			{
				sumIn += vars[*n];
			}
			IloExpr sumOut(target.getEnv());
			for (auto n : vert->m_outEdges)
			{
				sumOut += vars[*n];
			}
			target.add(sumIn <= 1);
			target.add(sumOut <= 1);
			target.add(sumIn == sumOut);
            // Remove double edges
            for(auto n: vert->m_inEdges)
            {
                auto* otherE = vert->connectedTo(n->m_source->id()) ? vert->findOutEdge(n->m_source) : nullptr;
                // Double edge: only one of both can be present
                if(otherE != nullptr)
                {
                    target.add(vars[*n] + vars[*otherE] <= 1);
                }
            }
		}
	}
}

void Algs::LongestPath::LongestPathILP::setupNewConstraints(IloIntVarArray& intVars, IloBoolVarArray& vars, IloModel& target) const
{
    // Create a bool variable and an int variable per edge.
    IloEnv env = target.getEnv();
    for(int i = 0; i < m_graph->number_of_edges(); ++i)
    {
        IloBoolVar bVar(target.getEnv());
        vars.add(bVar);
        IloIntVar iVar(target.getEnv());
        intVars.add(iVar);
        target.add(iVar >= 1);
        target.add(iVar <= static_cast<int>(2 * m_graph->number_of_edges()));
    }
    for(int i = 0; i < m_graph->number_of_vertices(); ++i)
    {
        // Ignore source and sink
        if (i == m_source || i == m_sink) continue;
        // Create constraint on edges
        IloExpr sumExprIn(env);
        IloExpr sumExprIn_I(env);
        auto* vert = m_graph->vertex(i);
        for(auto e : vert->m_inEdges)
        {
            sumExprIn += vars[*e];
            sumExprIn_I += intVars[*e];
        }
        target.add(sumExprIn <= 1);
        IloExpr sumExprOut(env);
        IloExpr sumExprOut_I(env);
        for (auto e : vert->m_outEdges)
        {
            sumExprOut += vars[*e];
            sumExprOut_I += intVars[*e];
        }
        target.add(sumExprOut <= 1);
        // Impose monotonous ordering
        target.add(2 * (sumExprOut_I - sumExprIn_I) == sumExprIn + sumExprOut);
    }
    // Add sourcde and sink constraint
    IloExpr sinkExpr(env);
    for (auto e : m_graph->vertex(m_sink)->m_inEdges)
    {
        sinkExpr += vars[*e];
    }
    target.add(sinkExpr == 1);
    IloExpr sourceExpr(env);
    for (auto e : m_graph->vertex(m_source)->m_outEdges)
    {
        sourceExpr += vars[*e];
    }
    target.add(sourceExpr == 1);
}

Algs::BasisElement Algs::LongestPath::LongestPathILP::computeLongestPath(const Algs::FieldType& field, NT maxWeight)
{
	IloEnv env;
	IloModel model(env);
	IloBoolVarArray vars(env);
    //IloBoolVarArray intVars(env);
	// Add a variable per edge
	for (int i = 0; i < m_graph->number_of_edges(); i++)
	{
		vars.add(IloBoolVar(env));
	}

	// Add simple path requirements
	this->setupSimpleConstraints(vars, model);
    //this->setupNewConstraints(intVars, vars, model);

	// Add optimization
	IloExpr expr(env);
	for (int i = 0; i < field.size(); i++)
	{
		expr += vars[i] * (double)(field[i]);
	}
	model.add(IloMaximize(env, expr));

    // Add max weight constraint
    model.add(expr < maxWeight);

	// Constraint handler for cycles
	LazyConstraintHandler lh(m_graph, m_source, m_sink, vars, env);
    lh.setVerbosity(false);

    IloCplex cplex(model);
	try
	{
		cplex.use(&lh);
        // Don't show CPLEX output
		cplex.setOut(env.getNullStream());
		bool success = cplex.solve();
		if (! success)
		{
			std::cout << "Solve unsuccesful" << std::endl;
			return BasisElement{};
		}
        m_bestWeight = 0;
		
        auto edgePath = Helpers::Cplex::pathFromCplex(cplex, vars, m_source, m_sink, *m_graph);
        for(auto* e : edgePath)
        {
            m_bestWeight += field[e->id()];
        }
        // Cleanup Cplex memory.
        cplex.end();
        env.end();
        BasisElement ret;
        std::transform(edgePath.begin(), edgePath.end(), std::back_inserter(ret), [](auto* e) {return e->id(); });
        return ret;
	}
	catch (IloException& e)
	{
		std::cout << "Something went wrong during CPLEX run:" << e.getMessage() <<  std::endl;
        cplex.end();
        env.end();
	}
	// 
	return {};
}

Algs::BasisElement Algs::LongestPath::LongestPathILP::retry(const FieldType& field)
{
    return computeLongestPath(field, m_bestWeight);
}

void Algs::LongestPath::LongestPathILP::clear()
{
    m_bestWeight = 0;
}
