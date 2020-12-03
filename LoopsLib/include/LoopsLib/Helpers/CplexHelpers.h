#ifndef HELPERS_CPLEXHELPERS
#define HELPERS_CPLEXHELPERS
#include <ilcplex/ilocplexi.h>
#include <Eigen/Eigen>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/GraphAlgs/DFS.h>
#include <LoopsLib/GraphAlgs/ShortestPath.h>
#include <LoopsLib/GraphAlgs/CycleFinder.h>
#include "Iterators.h"

namespace LoopsLib::Helpers::Cplex
{
    inline std::vector<DS::BaseGraph::Edge*> pathFromCplex(IloCplex& solver, IloBoolVarArray& edgeVarArray, DS::BaseGraph::Id_t src, DS::BaseGraph::Id_t sink, DS::BaseGraph& graph)
    {
        using Id = DS::BaseGraph::Id_t;
        using Edge = DS::BaseGraph::Edge;
        std::map<DS::BaseGraph::Id_t, DS::BaseGraph::Id_t> vertToIdx;
        std::map<DS::BaseGraph::Id_t, DS::BaseGraph::Id_t> idxToVert;
        std::vector<std::pair<Id, Id>> edges;
        
        for(std::size_t i = 0; i < edgeVarArray.getSize(); ++i)
        {
            if (solver.getValue(edgeVarArray[i]) > 0.5)
            {
                auto* e = graph.edge(i);
                edges.emplace_back(e->m_source->id(), e->m_sink->id());
                if(auto pair = vertToIdx.try_emplace(e->m_source->id(), vertToIdx.size()); pair.second)
                {
                    idxToVert[pair.first->second] = pair.first->first;
                }
                if (auto pair = vertToIdx.try_emplace(e->m_sink->id(), vertToIdx.size()); pair.second)
                {
                    idxToVert[pair.first->second] = pair.first->first;
                }
            }
        }
        // Build a graph
        DS::BaseGraph pathGraph(vertToIdx.size());
        for(auto p : edges)
        {
            pathGraph.addEdge(p.first, p.second);
        }

        GraphAlgs::LeastEdgesPath sp;
        std::vector<Edge*> mainBranch;
        sp.computeShortestPath(&pathGraph, vertToIdx[src], vertToIdx[sink], mainBranch);
        std::list<Edge*> pathList;
        for(auto* e : mainBranch)
        {
            pathList.push_back(graph.vertex(idxToVert[e->m_source->id()])->findOutEdge(idxToVert[e->m_sink->id()]));
        }
        std::map<Id, decltype(pathList)::const_iterator> vertItMap;
        for(auto it = pathList.begin(); it != pathList.end(); ++it)
        {
            vertItMap[(*it)->m_source->id()] = it;
        }
        // Delete main path
        for(auto* e: pathList)
        {
            pathGraph.deleteEdge(e);
        }
        // Find cycles
        GraphAlgs::CycleFinder cycleFind;
        while(true)
        {
            if (pathGraph.number_of_edges() == 0) break;

            // Find a new cycle
            std::vector<DS::BaseGraph::Edge*> cycle;
            cycleFind.findCycle(&pathGraph, cycle);
            if (cycle.empty()) break;

            // Reconstruct cycle in original graph.
            std::vector<DS::BaseGraph::Edge*> origCycle;
            std::transform(cycle.begin(), cycle.end(), std::back_inserter(origCycle), [&graph,&idxToVert](DS::BaseGraph::Edge* e)
            {
                return graph.vertex(idxToVert[e->m_source->id()])->findOutEdge(idxToVert[e->m_sink->id()]);
            });

            // Add with correct IDs to path.
            for(auto it = origCycle.begin(); it != origCycle.end(); ++it)
            {
                auto* e = *it;
                if(vertItMap.find(e->m_source->id()) != vertItMap.end())
                {
                    LoopsLib::Helpers::Iterators::circular_it start(cycle, it);
                    LoopsLib::Helpers::Iterators::circular_it end(cycle,it,1);
                    pathList.insert(vertItMap[e->m_source->id()],start, end);

                    // Delete cycle from pathGraph
                    for(auto* e : cycle)
                    {
                        pathGraph.deleteEdge(e);
                    }
                    break;
                }
            }
        }
        mainBranch.clear();
        mainBranch.assign(pathList.begin(), pathList.end());
        
        return mainBranch;
    }

    template<typename IloArrayT>
    class MatrixView
    {
        // The array of variables. Assumed to be column-major
        IloArrayT& m_array;
        // Number of rows in the matrix
        int m_numRows;
    public:
        MatrixView(IloArrayT& array, int numRows) :m_array(array), m_numRows(numRows) {}

        /**
         * \brief Returns the element at the given location in the matrix
         * \param r The row number 
         * \param c The column number
         * \return The variable
         */
        auto operator()(int r, int c)
        {
            return m_array[c*m_numRows + r];
        }
        /**
         * \brief Perform a matrix multiplication, using the other array
         * as a column vector
         * \tparam OtherArrayT Type of the array
         * \param arr The array
         * \return The expression for the multiplication
         */
        template<typename OtherArrayT>
        auto operator*(const OtherArrayT& arr)
        {
            IloExpr expr(m_array.getEnv());
            for (int i = 0; i < m_numRows; ++i)
            {
                for (int j = 0; j < arr.getSize(); ++j)
                {
                    expr += this->operator()(i, j) * arr[j];
                }
            }
            return expr;
        }
        template<typename OtherArrayT>
        auto sumOfSquares(const OtherArrayT& valueVars, const Eigen::VectorXd& subtractCoeffs)
        {
            IloExpr expr(m_array.getEnv());
            for (int i = 0; i < m_numRows; ++i)
            {
                IloExpr localSum(m_array.getEnv());
                for (int j = 0; j < valueVars.getSize(); ++j)
                {
                    localSum += this->operator()(i, j) * valueVars[j];
                }
                expr += (localSum - subtractCoeffs(i))*(localSum - subtractCoeffs(i));
            }
            return expr;
        }
        template<typename OtherArrayT>
        auto sumOfSquares(const OtherArrayT& valueVars, const std::vector<MovetkGeometryKernel::NT>& subtractCoeffs)
        {
            IloExpr expr(m_array.getEnv());
            for (int i = 0; i < m_numRows; ++i)
            {
                IloExpr localSum(m_array.getEnv());
                for (int j = 0; j < valueVars.getSize(); ++j)
                {
                    localSum += this->operator()(i, j) * valueVars[j];
                }
                expr += (localSum - subtractCoeffs[i])*(localSum - subtractCoeffs[i]);
            }
            return expr;
        }
        int columnCount() const
        {
            return m_array.size() / m_numRows;
        }
    };
}
#endif