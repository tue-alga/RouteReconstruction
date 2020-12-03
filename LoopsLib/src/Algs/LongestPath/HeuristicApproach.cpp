#include <LoopsLib/Algs/LongestPath/HeuristicApproach.h>
#include <helpers.hpp>
using namespace Algs::LongestPath;
using namespace DS;

param_t HeuristicApproach::GetParams(int i)
{
	if (i == 0)
		return {{1, 4, 16, 64, 1, 4, 16, 64}};
	if (i == 1)
		return {{-43, 31, 11, 58, -4, 23, 43, 45}};
	param_t X;
	for (auto& x : X)
		x = rand() % 100 - 10;
	return X;
}


void HeuristicApproach::buildRanks()
{
	const int num_scc = m_ccs.sccCount();
	m_scc_rank_in.resize(m_vertNum, 0);

	// Strongly connected components and coloring
	const auto& sccs = m_ccs.sccComponents();
	const auto& sccColoring = m_ccs.sccColoring();

	for (int scc = 0; scc < num_scc; ++scc)
	{
		
		for (int x : sccs[scc])
		{
			// x is the actual node in the scc component
			for (auto v : m_graph->inneighbors(x))
			{
				//v is the actual node in the scc component
				int connected_component_neigh = sccColoring[v];

				if (connected_component_neigh == scc) continue;

				int candidate = m_scc_rank_in[connected_component_neigh] + 1;

				if (candidate > m_scc_rank_in[scc])
				{
					m_scc_rank_in[scc] = candidate;
				}
			}
		}
	}

	// 	std::cout << "Finished with this weird loop." << std::endl;

	m_scc_rank_out.resize(num_scc, 0);

	for (int scc = num_scc - 1; scc >= 0; --scc)
	{
		for (int x : sccs[scc])
		{
			// x is the actual node in the scc component
			for (auto v : m_graph->outneighbors(x))
			{
				//v is the actual node in the scc component
				int connected_component_neigh = sccColoring[v];

				if (connected_component_neigh == scc)
				{
					continue;
				}

				int candidate = m_scc_rank_out[connected_component_neigh] + 1;

				if (candidate > m_scc_rank_out[scc])
				{
					m_scc_rank_out[scc] = candidate;
				}
			}
		}
	}

	// 	std::cout << "Finished with this other weird loop." << std::endl;

	int start = 0;

	for (int color = 0; color < num_scc; ++color)
	{
		int s = sccs[color].size();

		if (s > 1)
		{
			m_scc_big_components.emplace_back(start, start + s, color);
		}

		start += s;
	}

	// 	std::cout << "Finished with this third weird loop." << std::endl;

	// 	for (int i = 0; i < m_scc_big_components.size(); ++i)
	// 	{
	// 		auto u = m_scc_big_components[i];
	// 		std::cout << i << ": " << u.start << " --> " << u.end << " w color " << u.color << std::endl;
	// 	}

	// 	std::cout << "Finished printing big component info." << std::endl;
	// 	std::cout << " with m_strongly_connected_components.size() = " << m_strongly_connected_components.size() << std::endl;
	// 	std::cout << " and num_scc = " << num_scc << std::endl;
	// 	for (int i = 0; i < num_scc; ++i)
	// 	{
	// 		std::cout << i << std::endl;
	// 		auto x = m_strongly_connected_components[i];
	// 		if (x.size() > 1)
	// 			std::cout << i << ": " << x << std::endl;
	// 	}

	// 	std::cout << "out of..." << m_strongly_connected_components.size();
	// 	std::cout << "scc_rank_out = " << m_scc_rank_out << std::endl;
}

void HeuristicApproach::ExpandGreedyBack(Path& P) const
{
	while (true)
	{
		auto l = P.back();
		auto& Neighs = m_graph->outneighbors(l);

		auto t = P.first_not_explored(Neighs.begin(), Neighs.begin() + m_graph->outdegree(l));

		// Stop if the sink is reached
		if (t == INVALID_NODE)break;
		
		P.emplace_back(t, m_weights[t.id]);
		if (t == m_sink) break; // Break when the sink is reached
	}
}


void HeuristicApproach::ExpandGreedyFront(Path& P) const
{
	while (true)
	{
		auto l = P.front().node;
		auto& Neighs = m_graph->inneighbors(l);

		auto t = P.first_not_explored(Neighs.begin(), Neighs.begin() + m_graph->indegree(l));

		// Stop if the source is reached.
		if (t == INVALID_NODE) {
			break;
		}

		P.emplace_front(t, t.weight);
		if (t == m_source) break; // Break when the sink is reached
	}
}

void HeuristicApproach::dfs_search_path_backward(Path& P, double maxnumseconds) const
{
	// 	using namespace std;
	// 	cout << "size of P is " << P.size() << endl;
	//ExpandGreedyBack(P);
	ExpandGreedyFront(P);
	Path Q = P;
	auto comp = [this](node_t a, node_t b)
	{
		return in_compare(a, b);
	};

	// 	cout << "m1.5" << endl;
	Chronometer C;
	while (C.Peek() < maxnumseconds && dfs_innext( Q, comp))
	{
		// Only accept source sink paths.
		if (!isSourceSinkPath(Q)) continue;

		// 		cout << "m2" << endl;
		if (Q.value() > P.value())
		{
			P = Q;
			C.Reset();
			// 			std::cout << "(" << ChronometerPeek() << ", " << P.Value() << ")," << std::endl;
			// 			std::cout << P.value()/2 << std::endl;
		}
	}
	// 	cout << "m3" << endl;
}

Path HeuristicApproach::dfs_search_path_forward(node_t start, double maxNumSeconds) const
{
	Path P(m_vertNum, start);
	dfs_search_path_forward(P, maxNumSeconds);
	return P;
}

Path HeuristicApproach::dfs_forward_full() const
{
	Path Best;

	//FORWARD SEARCH

	const int vCount = m_graph->num_vertices();
	int numrestarts = Options.dfs_forward_num_starting_nodes;
	const double t = Options.dfs_time_woimprovement;

	// Apply the constrained search.
	Best = dfs_search_path_forward(m_source, t);

	Path Q = Best;
	auto comp = [this](node_t a, node_t b)
	{
		return in_compare(a, b);
	};
	int it = 0;
	while(!isSourceSinkPath(Q))
	{
		if (!dfs_outnext(Q, comp))
			throw std::runtime_error("Could not find a source-sink path at all?");
		++it;
		std::cout << "At iteration:" << it << std::endl;
	}

	assertSourceSinkPath(Best);
	// TODO enforce ending at sink?
	
	// Cap to number of vertices, does not make sense to do more restarts perse.
	//if (numrestarts > vCount)
	//	numrestarts = vCount;

	//using namespace std;
	//// Try random restarts, looking for the best longest path.
	//for (int i = 0; i < numrestarts; ++i)
	//{
	//	node_t node = m_basic_topological_ordering[i / 2];
	//	// For every odd restart
	//	if ((i % 2) == 1)
	//	{
	//		// Pick a random node
	//		node = rand() % vCount;
	//	}
	//	Path P = dfs_search_path_forward(node, t);
	//	if (P.value() > Best.value())
	//	{
	//		Best = std::move(P);
	//		if (Best.value() > m_globalBest)
	//		{
	//			m_globalBest = Best.value();
	//			//cout << "Found DFS improvement at " << global_chrono.Peek() << " to " << m_globalBest << endl;
	//			cout << "Found DFS improvement to " << m_globalBest << endl;
	//		}
	//	}
	//}
	//// 	cout << "El mejor que encontré tiene tamaño " << Best.size() << endl;
	int num_deletions = Options.dfs_how_many_to_erase_from_opposite_side;
	auto P = Best;
	const int Psize = static_cast<int>(P.size());

	// Cap number of deletions to half the path size.
	if (2 * num_deletions > Psize)
	{
		num_deletions = Psize / 2;
	}

	for (int i = 0; i < num_deletions; ++i)
	{
		P.pop_front();
	}

	dfs_search_path_backward(P, t);

	if (isSourceSinkPath(P) && P.value() > Best.value())
	{
		Best = std::move(P);
		if (Best.value() > m_globalBest)
		{
			m_globalBest = Best.value();
			//cout << "Found DFS improvement at " << global_chrono.Peek() << " to " << global_best << endl;
			std::cout << "Found DFS improvement to " << m_globalBest<< std::endl;
		}
	}

	return Best;
}

Path HeuristicApproach::dfs_search_path_backward(node_t start, double maxnumseconds) const
{
	Path P( m_vertNum, start);
	dfs_search_path_backward(P, maxnumseconds);
	return P;
}

double HeuristicApproach::get_heuristic_out(node_t node)
{
	double a1 = m_params[0];
	double a2 = m_params[1];
	double a3 = m_params[2];
	double a4 = m_params[3];
	double heuristicex = 0;

	for (auto x : m_graph->outneighbors(node))
	{
		heuristicex += a1;

		for (auto y : m_graph->outneighbors(x))
		{
			heuristicex += a2;

			for (auto z : m_graph->outneighbors(y))
			{
				heuristicex += a3 + a4 * m_graph->outneighbors(z).size();
				// 				for (auto r : outgraph[z])
				// 				{
				// 					heuristicex += a4+a5*outgraph[r]./*size()*/;
				// 				}
			}
		}
	}

	return heuristicex;
}


double HeuristicApproach::get_heuristic_in(node_t node)
{
	// 	return 0;
	double a1 = m_params[4];
	double a2 = m_params[5];
	double a3 = m_params[6];
	double a4 = m_params[7];
	double heuristicin = 0;

	for (auto x : m_graph->inneighbors(node))
	{
		heuristicin += a1;

		for (auto y : m_graph->inneighbors(x))
		{
			heuristicin += a2 + a3 * m_graph->inneighbors(y).size();

			for (auto z : m_graph->inneighbors(y))
			{
				heuristicin += a3 + a4 * m_graph->inneighbors(z).size();
			}
		}
	}

	return heuristicin;
}

void HeuristicApproach::heuristic_processing()
{
	m_heuristic_out.resize(m_vertNum, 0);
	m_heuristic_in.resize(m_vertNum, 0);

	// 	double maxtime = 0.2/n;

// 	#pragma omp parallel for
	for (node_t i = 0; i < m_vertNum; ++i)
	{
		m_heuristic_out[i] = get_heuristic_out(i);
		m_heuristic_in[i] = get_heuristic_in(i);
	}


	m_basic_topological_ordering = range<node_t>(m_vertNum);
	sort(m_basic_topological_ordering.begin(), m_basic_topological_ordering.end(), [this](node_t a, node_t b) -> bool
	{
		//First, order by rank
		if (rank_out(a) < rank_out(b))
		{
			return false;
		}
		if (rank_out(a) > rank_out(b))
		{
			return true;
		}

		//If someone doesn't have out-neighbours, they should go last
		if (m_graph->outdegree(a) == 0)
		{
			return false;
		}
		if (m_graph->outdegree(b) == 0)
		{
			return true;
		}

		//The ones who have NO OTHER entry points should go first
		if (m_graph->indegree(b) == 1)
		{
			return false;
		}
		if (m_graph->indegree(a) == 1)
		{
			return true;
		}

		//If all else fails, sort by heuristic
		return m_heuristic_out[a] < m_heuristic_out[b];
	});

	m_basic_topological_ordering_inverse.resize(m_vertNum);

	for (node_t i = 0; i < m_vertNum; ++i)
	{
		m_basic_topological_ordering_inverse[m_basic_topological_ordering[i]] = i;
	}

	m_basic_topological_ordering_in = range<node_t>(m_vertNum);

	sort(m_basic_topological_ordering_in.begin(), m_basic_topological_ordering_in.end(), [this](node_t a, node_t b) -> bool
	{
		if (rank_in(a) != rank_in(b))
		{
			return rank_in(a) < rank_in(b);
		}

		// TODO check if the order matters here.
		if (m_graph->indegree(a) == 0 || m_graph->outdegree(b) == 1)
		{
			return false;
		}
		if (m_graph->indegree(b) == 0 || m_graph->outdegree(a) == 1)
		{
			return true;
		}

		return m_heuristic_in[a] < m_heuristic_in[b];
	});

	m_basic_topological_ordering_inverse_in.resize(m_vertNum);

	for (node_t i = 0; i < m_vertNum; ++i)
	{
		m_basic_topological_ordering_inverse_in[m_basic_topological_ordering_in[i]] = i;
	}


	m_graph->sort_outneighbours([this](node_t a, node_t b) -> bool
	{
		return ex_compare(a, b);
	});

	m_graph->sort_inneighbours([this](node_t a, node_t b) -> bool
	{
		return in_compare(a, b);
	});
}

void HeuristicApproach::dfs_search_path_forward(Path& P, double maxnumseconds) const
{
	ExpandGreedyBack(P);
	assertSourceSinkPath(P);

	// 	std::cout << "(" << ChronometerPeek() << ", " << P.Value() << ")," << std::endl;
	if (P.value() > m_globalBest)
	{
		m_globalBest = P.value();
		//std::cout << "Found DFS improvement at " << global_chrono.Peek() << " to " << global_best << std::endl;
		std::cout << "Found DFS improvement to " << m_globalBest << std::endl;
	}
	Chronometer C;
	Path Q = P;
	auto comp = [this](node_t a, node_t b)
	{
		return ex_compare(a, b);
	};

	while (C.Peek() < maxnumseconds && dfs_outnext(Q, comp))
	{
		if (Q.value() > P.value())
		{
			P = Q;
			C.Reset();
			// 			std::cout << "(" << C.Peek() << ", " << P.value() << ")," << std::endl;
			// 			std::cout << P.value()/2 << std::endl;
		}
	}
}


void HeuristicApproach::set_parameters(const param_t& new_params)
{
	m_params = new_params;
	heuristic_processing();
}

Path HeuristicApproach::dfs_backward_full()
{
	Path Best;

	//FORWARD SEARCH

	auto numrestarts = Options.dfs_backward_num_starting_nodes;
	auto t = Options.dfs_time_woimprovement;

	//if (numrestarts > m_vertNum)
	//	numrestarts = m_vertNum; //really no point in starting from more than the number of vertices.

	//for (int i = 0; i < numrestarts; ++i)
	//{
	//	node_t node = m_basic_topological_ordering_in[i / 2];
	//	if ((i % 2) == 1)
	//	{
	//		node = rand() % m_vertNum;
	//	}

	//	Path P = dfs_search_path_backward(node, t);

	//	if (P.value() > Best.value())
	//	{
	//		Best = std::move(P);
	//	}
	//}
	// CHANGED: only start search from sink for backwards DFS.
	Best = dfs_search_path_backward(m_sink, t);

	Path Q = Best;
	auto comp = [this](node_t a, node_t b)
	{
		return ex_compare(a, b);
	};
	int it = 0;
	while (!isSourceSinkPath(Q))
	{
		if (!dfs_innext(Q, comp))
			throw std::runtime_error("Could not find a source-sink path at all?");
		++it;
		std::cout << "At iteration:" << it << std::endl;
	}

	assertSourceSinkPath(Best);

	assertSourceSinkPath(Best);

	// Delete elements from the back
	int num_deletions = Options.dfs_how_many_to_erase_from_opposite_side;
	auto P = Best;
	int Psize = static_cast<int>(P.size());

	if (num_deletions > Psize / 2)
	{
		num_deletions = Psize / 2;
	}

	for (int i = 0; i < num_deletions; ++i)
	{
		P.pop_back();
	}

	// CHANGED: only allow valid s-t path.
	dfs_search_path_forward(P, t);

	if (P.value() > Best.value())
		Best = std::move(P);

	return Best;
}

Path HeuristicApproach::dfs_search()
{
	Path Best;
	using namespace std;
	// Repeatedly apply DFS
	for (int i = 0; i < Options.dfs_num_parameter_restarts; ++i)
	{
		set_parameters(GetParams(i));
		//std::cout << "Done setting parameters" << std::endl;
		auto ForwardPath = dfs_forward_full();
		assert(ForwardPath.front().node == m_source);
		assert(ForwardPath.back().node == m_sink);

		if (ForwardPath.value() > Best.value())
			Best = std::move(ForwardPath);

		//std::cout << "Done with forward BFS search" << std::endl;

		auto BackwardPath = dfs_backward_full();
		assert(BackwardPath.front().node == m_source);
		assert(BackwardPath.back().node == m_sink);
		if (BackwardPath.value() > Best.value())
			Best = std::move(BackwardPath);
		//std::cout << "Done with backward BFS search" << std::endl;
	}
	return Best;
}

void HeuristicApproach::remove_bad_nodes()
{
	std::vector<node_t> toRemove;
	toRemove.reserve(m_vertNum / 2);

	// 		std::cout << "removing bad nodes!" << std::endl;
	// 	int val = dfs_search_path(0.1).Value();

	size_t val = 0;

	// The largest number of elements in a single weakly connected component
	val=reduce(m_ccs.wccComponents().begin(), m_ccs.wccComponents().end(), [](int& accum, int curr, const auto& el)
	{
		accum = std::max(accum, static_cast<int>(el.size()));
	},0);

	// Nodes to remove from the graph
	toRemove = reduce(m_ccs.wccComponents().begin(), m_ccs.wccComponents().end(), [val](std::vector<node_t>& accum, int curr, const auto& el)
	{
		if(el.size() < val)
		{
			for (auto node : el) accum.push_back(node);
		}
	},std::vector<node_t>{});

	// 	std::cout << "done adding toremove! sorting..." << std::endl;
	// 	sort(toRemove.begin(), toRemove.end());

	// 	for (int i = 0; i < n; ++i)
	// 	{
	// 		if (outgraph[i].size() == 0 && ingraph[i].size() == 0)
	// 			toRemove.push_back(i);
	// 	}
	// 	std::cout << "Removing: " << toRemove.size() << " nodes" << std::endl;
	m_graph->remove_nodes(toRemove);
}

void HeuristicApproach::process()
{
	if (!m_processed)
	{
		
		// 		std::cout << "Processing graph..." << std::endl;
		Chronometer C;
		m_ccs.process(); // We may do this only once if possible.

		buildRanks();
		//find_weakly_connected_components();
		// 		std::cout << "weak coloring = " << m_weak_coloring << std::endl;
		// 		std::cout << "before removing, size: " << num_vertices() << std::endl;
		//remove_bad_nodes();
		// 		std::cout << "After removing, new size: " << m_n << std::endl;

		//find_strongly_connected_components();

		// 		std::cout << "Scc: " << m_scc_coloring << std::endl;

		m_processed = true;
		// 		std::cout << "Finished processing graph after " << C.Peek() << " seconds" << std::endl;
	}
}

Path Algs::LongestPath::HeuristicApproach::findLongestPath()
{
	process();

	//std::cout << "Finished preprocessing in: " << global_chrono.Reset() << std::endl;

	std::cout << "Doing DFS search..." << std::endl;
	Path best = dfs_search();


	//std::cout << "Done DFS search! Best Value = " << global_best << std::endl;
	//std::cout << "Time taken for dfs: " << global_chrono.Peek() << std::endl;

	std::cout << "Doing PTO improving search..." << std::endl;
	pto_search(best);

	return best;
}
void HeuristicApproach::pto_search(Path& A) const
{
	Chronometer C;
	std::cout << "Path size: " << A.size() << std::endl;

	// Generate a pseudo topological order.
	PseudoTopoOrder PTO = get_random_pseudotopological_order();
	// 	std::cout << "Before applying A, PTO.Value() = " << PTO.Value() << std::endl;
	std::cout << "Created PTO of size " << PTO.size() << std::endl;
	//
	PTO.apply(A);

	// 	if (!check_path(A))
	// 	{
	// 		std::cout << "SHIT! path is not correct!" << std::endl;
	// 		throw;
	// 	}
	 	/*auto val = PTO.Value();
	 	std::cout << "After applying A, PTO.Value() = " << val << std::endl;
	 	A = PTO.get_path();
	 	std::cout << "Uh, the path then should have the same value: " << A.value() << std::endl;
	 	std::cout << "is_path_in_order: " << PTO.is_path_in_order(A) << std::endl; */
	// 	if (!check_path(A))
	// 	{
	// 		std::cout << "SHIT! path is not correct!" << std::endl;
	// 		throw;
	// 	} else
	// 	{
	// 		std::cout << "Good path of value " << A.value()  << std::endl;
	// 	}

	// 	return;
	PTO.open_edges_until_no_more_improvement_found();

	// 	std::cout << "After opening edges, PTO.Value() = " << PTO.Value() << std::endl;

	A = PTO.get_path();
	// 	if (!check_path(A))
	// 	{
	// 		std::cout << "SHIT! path is not correct!" << std::endl;
	// 		throw;
	// 	} else
	// 	{
	// 		std::cout << "Good path!" << std::endl;
	// 	}
}
PseudoTopoOrder HeuristicApproach::get_random_pseudotopological_order() const
{
	std::vector<node_t> topo_sort(m_vertNum, 0); // at index i, it has a node topo_sort[i]
	std::vector<node_t> topo_sort_inverse(m_vertNum, 0); // at node i, x=topo_sort_inverse[i] is such that topo_sort[x] = i
	int i = 0;

	for (auto X : m_ccs.sccComponents())
	{
		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(X.begin(), X.end(), g);

		for (auto x : X)
		{
			topo_sort[i] = x;
			topo_sort_inverse[x] = i;
			++i;
		}
	}

	return PseudoTopoOrder(*this, topo_sort, topo_sort_inverse);
}