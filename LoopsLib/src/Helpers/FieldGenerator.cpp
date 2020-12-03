#include <LoopsLib/Helpers/FieldGenerator.h>
#include <random>
#include <LoopsLib/Algs/Processing/ReachabilityLabeling.h>
#include <LoopsLib/Helpers/reduce.h>
#include <LoopsLib/Helpers/RandomHelpers.h>
#include <LoopsLib/GraphAlgs/ShortestPath.h>
using namespace LoopsLib;

void Helpers::FieldGenerator::setNormalCoeffs(double mean, double sd)
{
	m_mean = mean;
	m_sd = sd;
}

void Helpers::FieldGenerator::generateRandomField(RandomType type, Eigen::VectorXd& field)
{
	const int fieldSize = m_graph->number_of_edges();
	using namespace Eigen;
	if (type == RandomType::Uniform)
	{
		field.setRandom(fieldSize);
		field = (field + VectorXd::Ones(fieldSize)) * m_maxSpan / 2.0 + VectorXd::Ones(fieldSize) * m_offset;
	}
	else
	{
		// Sampler for flow
		std::random_device rd{};
		std::mt19937 gen{rd()};
		std::normal_distribution<> d{m_mean, m_sd};

		field.setConstant(fieldSize, 0);
		for (int j = 0; j < fieldSize; ++j) field(j) = std::max(0.0, d(gen));
	}
}

void Helpers::FieldGenerator::generateRandomPathField(int source, int sink, int maxPaths, double maxVal, Eigen::VectorXd& field)
{
    // Generate DAG views, select paths from the DAG.

	std::cout << "Graph size: " << m_graph->number_of_vertices() << std::endl;
	// Get distance labels to sink
	Algs::Processing::ReachabilityLabeling labeling(m_graph);
	std::vector<int> distLabels;
	labeling.apply(sink, distLabels);

	// Determine actual outdegrees with requirement to be able to reach the sink
	std::vector<int> outDegs;
	outDegs.resize(m_graph->number_of_vertices(), 0);
    // Count actual number of edgeswith finite distance labels
	for(int i = 0 ; i < m_graph->number_of_vertices(); i++)
	{
        auto* vert = m_graph->vertex(i);
		for(auto n : vert->m_outEdges)
		{
            auto nId = n->m_sink->id();
			if (distLabels[nId] <= m_graph->number_of_vertices()) outDegs[i]++;
		}
	}
	// Sort out neighbours by distance label.
	/*auto sorter = [&distLabels](const DS::NeighborNode& n1, const DS::NeighborNode& n2)
	{
		return distLabels[n1] < distLabels[n2];
	};
	m_graph->sort_outneighbours(sorter);*/

	// Compare elements on distance label
	auto comp = [&distLabels](const DS::BaseGraph::Edge* n1, const DS::BaseGraph::Edge* n2)
	{
		return distLabels[*n1->m_sink] < distLabels[*n2->m_sink];
	};

	// Number of paths to use
	const int targetPaths = maxPaths / 2 + (rand() % (maxPaths / 2));

	// Flow value per path
	std::vector<double> vals;
	vals.resize(targetPaths, maxVal / (double)targetPaths);
	// Randomize values a bit by exchanging part of the flow value.
	for(int i = 0; i < targetPaths; i++)
	{
		std::default_random_engine eng;
		std::uniform_real_distribution<double> urd;
		double val = urd(eng);
		// Select a target to exchange with
		int target = Helpers::RandomHelpers::randomOther(i, 0, vals.size());
		
		// Exchange some random part of 10% of the value.
		double part = val * 0.1 * vals[i];
		vals[target] += part;
		vals[i] -= part;
	}

	// Initialize path
	field.setConstant(m_graph->number_of_edges(),0);

    std::default_random_engine eng;
	std::uniform_real_distribution<double> urd;
	
	for(int i = 0 ; i < targetPaths; i++)
	{
		std::cout << "Path " << (i + 1) << " of " << targetPaths << std::endl;
		std::vector<DS::BaseGraph::Vertex*> path;
		// Sentinel for source start
        path.push_back(m_graph->vertex(source));

		bool isSearchingEnd= false;
		while(*path.back() != sink)
		{
			double p = urd(eng);
			// With some probability, start looking for the sink
			if (!isSearchingEnd && p < m_probSearchEnd) {
				std::cout << "To out ! At " << path.size() <<  std::endl;
				isSearchingEnd = true;
			}
			// Successors
			const auto& succ = path.back()->m_outEdges;
			// Take shortest paths.
			if(isSearchingEnd)
			{
                auto el = std::min_element(succ.begin(), succ.end(), comp);
				path.push_back((*el)->m_sink);
			}
			// Pick random neighbour.
			else
			{
				//// Pick a valid neighbour in the top half of the distances
				//int target = outDegs[path.back()] / 2 + (rand() % ( ( outDegs[path.back()] + (outDegs[path.back()] & 1)) / 2));
				//// Avoid weird rounding etc.
				//target = std::max(outDegs[path.back()] - 1, target);
                //auto el = std::max_element(succ.begin(), succ.end(), comp);
                auto el = RandomHelpers::randomElement(succ.begin(), succ.end());

				// The out neighbours should be sorted on distance label here.
				path.push_back((*el)->m_sink);
			}
		}
		for(int j = 0; j < path.size()-1; j++)
		{
			//const int e = m_graph->edgeID(path[j], path[j + 1]);
            const auto* e = path[j]->findOutEdge(path[j + 1]);
			field(e->id()) += vals[i];
		}
		std::cout << "Added path of value " << vals[i] << " and size " << path.size() << std::endl;
	}

}

void Helpers::FieldGenerator::setNumberOfPaths(int numberOfPaths)
{
    m_numberOfPaths = numberOfPaths;
}

void Helpers::FieldGenerator::setGraph(DS::BaseGraph* graph)
{
	m_graph = graph;
}


void Helpers::FieldGenerator::generatePathsField()
{
    m_decompObj->m_paths.clear();
    m_decompObj->m_pathValues.clear();

    std::uniform_int_distribution<int> intRand(0, m_decompObj->m_sinks.size()-1);
    std::uniform_real_distribution<double> doubleRand(0, 0.5 * m_decompObj->m_graph->number_of_vertices());
    auto eengine = RandomHelpers::getRandomEngine();
    auto randomSink = [&intRand, &eengine]()
    {
        return intRand(eengine);
    };
    std::vector<DS::BaseGraph::Id_t> sinks(m_decompObj->m_sinks.begin(), m_decompObj->m_sinks.end());

    std::vector<double> weights;
    for(int i = 0; i < m_decompObj->m_graph->number_of_edges(); ++i)
    {
        weights.push_back(doubleRand(eengine));
    }

    GraphAlgs::WeightedShortestPath<std::vector<double>> shortestPath;
    // Setup the field.
    m_decompObj->m_field.assign(m_decompObj->m_graph->number_of_edges(), 0.0);

    for (auto s : m_decompObj->m_sources)
    {
        for(int i = 0; i < m_numberOfPaths; ++i)
        {
            auto sink = sinks[randomSink()];
            std::shuffle(weights.begin(), weights.end(), eengine);
            std::vector<DS::BaseGraph::Edge*> path;
            shortestPath.computeShortestPath(m_decompObj->m_graph, weights, s, sink, path);
            if(!path.empty())
            {
                m_decompObj->m_paths.push_back(path);
                for(auto* e : path)
                {
                    m_decompObj->m_field[e->id()] += 1.0; //TODO arbitrary weight?
                }
                m_decompObj->m_pathValues.push_back(1.0);
            }
        }
    }
}

void Helpers::FieldGenerator::generateRandomPathsField()
{
    m_decompObj->m_paths.clear();
    m_decompObj->m_pathValues.clear();
    std::uniform_int_distribution<int> intRand(0, m_decompObj->m_graph->number_of_vertices() - 1);
    std::uniform_real_distribution<double> doubleRand(0, 0.5 * m_decompObj->m_graph->number_of_vertices());
    auto eengine = RandomHelpers::getRandomEngine();
    auto randomVertt = [&intRand, &eengine]()
    {
        return intRand(eengine);
    };
    std::vector<double> weights;
    for (int i = 0; i < m_decompObj->m_graph->number_of_edges(); ++i)
    {
        weights.push_back(doubleRand(eengine));
    }

    GraphAlgs::WeightedShortestPath<std::vector<double>> shortestPath;
    m_decompObj->m_sources.clear();
    m_decompObj->m_sinks.clear();

    // Setup the field.
    m_decompObj->m_field.assign(m_decompObj->m_graph->number_of_edges(), 0.0);

    for (int i = 0; i < m_numberOfPaths; ++i)
    {
        auto src = randomVertt();
        auto sink = randomVertt();
        std::shuffle(weights.begin(), weights.end(), eengine);
        std::vector<DS::BaseGraph::Edge*> path;
        shortestPath.computeShortestPath(m_decompObj->m_graph, weights, src, sink, path);
        if (!path.empty())
        {
            m_decompObj->m_paths.push_back(path);
            for (auto* e : path)
            {
                m_decompObj->m_field[e->id()] += 1.0; //TODO arbitrary weight?
            }
            m_decompObj->m_sources.insert(src);
            m_decompObj->m_sinks.insert(src);
            m_decompObj->m_pathValues.push_back(1.0);
        }
    }
}

//void Helpers::FieldGenerator::generateField(int source, int sink, std::vector<MovetkGeometryKernel::NT>& field)
//{
//    if(!m_decompObj->m_graph)
//    {
//        std::cerr << "No graph set, so not generating field" << std::endl;
//        return;
//    }
//    m_graph = m_decompObj->m_graph;
//	if (m_activeGen == Generators::UniformRandom)
//		generateRandomField(RandomType::Uniform, field);
//	else if (m_activeGen == Generators::NormalRandom)
//		generateRandomField(RandomType::Normal, field);
//    else if(m_activeGen == Generators:: VertexOrder)
//    {
//        
//    }
//	else
//	{
//		generateRandomPathField(source, sink, m_numberOfPaths, m_pathValue, field);
//	}
//}
