#include <LoopsLib/DS/Graph.h>
#include <sstream>
//#include <helpers.hpp>
using namespace DS;
//Heuristics

DiGraph::DiGraph(node_t numNodes) :
		m_n(numNodes),
		m_outgraph(numNodes),
		m_ingraph(numNodes),
		m_outDegrees(numNodes,0),
		m_inDegrees(numNodes, 0)
{
}

int DiGraph::add_edge(node_t from, node_t to)
{
	int id = m_edgeCount;
	++m_edgeCount;
	m_outgraph[from].emplace_back(to, 1.0, id);
	m_ingraph[to].emplace_back(from, 1.0, id);
	m_outDegrees[from]++;
	m_inDegrees[to]++;
	return id;
}

int DiGraph::add_edge(node_t from, node_t to, int id)
{
	++m_edgeCount;
	m_outgraph[from].emplace_back(to, 1.0, id);
	m_ingraph[to].emplace_back(from, 1.0, id);
	m_outDegrees[from]++;
	m_inDegrees[to]++;
	return id;
}


template <class Container, class T>
bool is_in(const Container& C, T val)
{
	return std::find(C.begin(), C.end(), val) != C.end();
}

bool DiGraph::are_neighbors(node_t a, node_t b) const
{
	return is_in(outneighbors(a), b);
}


size_t DiGraph::num_edges() const
{
	size_t toReturn = 0;

	for (node_t i = 0; i < m_n; ++i)
	{
		toReturn += m_outgraph[i].size();
	}

	return toReturn;
}

node_t DiGraph::edgeID(node_t start, node_t end)
{
	const auto& edges = m_outgraph[start];
	for (const auto& e : edges)
	{
		if (e.node == end)
		{
			return e.id;
		}
	}
	std::stringstream ss;
	ss << "Nodes are not connected, so no edge ID was found. IDS " << start << ", " << end;
	
	throw std::runtime_error(ss.str().c_str());
}

void DiGraph::setWeights(const Eigen::VectorXd& weights)
{
	for (int i = 0; i < m_n; i++)
	{
		for (auto& e : m_outgraph[i])
		{
			e.weight = weights(e.id);
		}
		for (auto& e : m_ingraph[i])
		{
			e.weight = weights(e.id);
		}
	}
}

DiGraph DiGraph::with_nodes_removed(std::vector<node_t>& toRemove) const
{
	if (toRemove.empty())
	{
		return *this;
	}
	DiGraph newGraph = *this;
	newGraph.remove_nodes(toRemove);
	return newGraph;

	//sort(toRemove.begin(), toRemove.end());

	//size_t new_n = m_n - toRemove.size();
	//// 	DiGraph D(new_n);
	//std::vector<std::string> new_names;
	//new_names.reserve(new_n);

	//std::vector<node_t> removalfunction(m_n, INVALID_NODE);
	//std::vector<node_t> removalfunctioninverse(new_n, 0);

	//node_t j = 0;
	//node_t w = 0;

	//for (node_t i = 0; i < m_n; ++i)
	//{
	//	if (i != toRemove[w])
	//	{
	//		removalfunction[i] = j;
	//		removalfunctioninverse[j] = i;
	//		new_names.emplace_back(m_node_names[i]);
	//		++j;
	//	}
	//	else
	//	{
	//		++w;
	//	}
	//}

	//DiGraph D(new_names);

	//for (size_t v = 0; v < new_n; ++v)
	//{
	//	auto oldv = removalfunctioninverse[v];

	//	for (auto oldneigh : outneighbors(oldv))
	//	{
	//		auto newneigh = removalfunction[oldneigh];
	//		D.add_edge(v, newneigh, oldneigh.Weight());
	//	}
	//}

	//return D;
}

void DiGraph::disableEdge(node_t node, node_t edgeId, bool isIn)
{
	int pos = 0;
	std::vector<NeighborNode>& target = isIn ? m_ingraph[node] : m_outgraph[node];
	for(; pos < target.size(); ++pos)
	{
		if(target[pos].id == edgeId)
		{
			// Move delete edge to the back
			std::swap(target[pos], target[target.size() - 1]);
			std::vector<int>& deg = isIn ? m_inDegrees : m_outDegrees;
			// Decrease the degree so that the edge is ignored.
			deg[node]--;
			return;
		}
	}
	throw std::runtime_error("Could not find the appropriate edge to delete");
}

void DiGraph::remove_nodes(std::vector<node_t>& toRemove)
{
	//*this = with_nodes_removed(toRemove);

	//
	for(auto node : toRemove)
	{
		m_outDegrees[node] = 0;
		for(const auto& edge : m_outgraph[node])
		{
			disableEdge(edge.node, edge.id, true);
		}
		m_inDegrees[node] = 0;
		for (const auto& edge : m_ingraph[node])
		{
			disableEdge(edge.node, edge.id, false);
		}
	}
}

DiGraph DiGraph::CreateRandomDiGraph(int n, double p)
{
	DiGraph D(n);

	for (int i = 0; i < n; ++i)
	{
		for (int j = i + 1; j < n; ++j)
		{
			/*if (probability_of_true(p))
			{
				int a = i;
				int b = j;

				if ((rand() % 2) == 1)
				{
					std::swap(a, b);
				}

				D.add_edge(a, b);
			}*/
		}
	}

	// 	D.process();
	return D;
}
//
//
//DiGraph DiGraph::CreateRandomWeightedDiGraph(int n, double p, weight_t minweight, weight_t maxweight)
//{
//	DiGraph D(n);
//
//	for (int i = 0; i < n; ++i)
//	{
//		for (int j = i + 1; j < n; ++j)
//		{
//			if (probability_of_true(p))
//			{
//				int a = i;
//				int b = j;
//				weight_t w = random_real(minweight, maxweight);
//
//				if (rand() % 2 == 1)
//				{
//					std::swap(a, b);
//				}
//
//				D.add_edge(a, b, w);
//			}
//		}
//	}
//
//	// 	D.process();
//	return D;
//}



//std::ostream& operator<<(std::ostream& os, const DiGraph& M)
//{
//	os << "Digraph on " << M.num_vertices() << " vertices: " << M.get_vertex_names();
//
//	for (int i = 0; i < M.num_vertices(); ++i)
//	{
//		const std::string& name = M.get_vertex_name(i);
//
//		for (auto v : M.outneighbors(i))
//		{
//			os << std::endl;
//			const std::string& vname = M.get_vertex_name(v);
//			os << name << u8" ⟼ " << vname << " with weight " << double(v.Weight());
//		}
//	}
//
//	return os;
//}


// std::vector<DiGraph> DiGraph::StronglyConnectedInducedGraphs()
// {
// 	process();
// 	
// 	
// }
