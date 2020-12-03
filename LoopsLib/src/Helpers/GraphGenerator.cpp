//#include <LoopsLib/Helpers/GraphGenerator.h>
//#include <unordered_set>
//#include <LoopsLib/Algs/Processing/IsReachable.h>
//#include <LoopsLib/Algs/Processing/RemoveDeadends.h>
//#include <CGAL/function_objects.h>
//#include <CGAL/Triangulation_2.h>
//#include <CGAL/point_generators_2.h>
//#include <CGAL/Triangulation_vertex_base_with_info_2.h>
//#include <CGAL/Delaunay_triangulation_2.h>
//#include <CGAL/nearest_neighbor_delaunay_2.h>
//#include <LoopsLib/Helpers/RandomHelpers.h>
//using namespace LoopsLib;
//struct pair_hash
//{
//	template <class T1, class T2>
//	std::size_t operator() (const std::pair<T1, T2> &pair) const
//	{
//		return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
//	}
//};
//
//void src::Helpers::GraphGenerator::randomFlipEdges(DS::BaseGraph& graph)
//{
//	// Not efficient
//    std::vector<DS::BaseGraph::Edge*> toFlip;
//    for(auto e : graph.edges())
//    {
//        if (::Helpers::RandomHelpers::randomDouble(0, 1) < m_flipProbability)
//        {
//            toFlip.emplace_back(e);
//        }
//    }
//	for(auto fl : toFlip)
//	{
//        fl->flip();
//	}
//}
//
//void src::Helpers::GraphGenerator::postProcess(std::unordered_set<std::pair<int, int>, PairHash<int,int>>& edges, int source, int sink, DS::BaseGraph& graph, std::unordered_set<int>& removedVerts)
//{
//	if(m_flipEdges)
//	{
//		randomFlipEdges(graph);
//	}
//
//	// For now, we don't do deletion.
//	Algs::Processing::IsReachable reachability;
//
//	// As long as the source and sink are not connected, generate extra edges
//	while (!reachability.apply(&graph, source, sink))
//	{
//		std::cout << "[GraphGen]\tGenerating more edges to connect source and sink" << std::endl;
//		// Generate more edges to hopefully get a connected graph.
//		for (int j = 0; j < 10; j++)
//		{
//			int start = rand() % m_vertNum;
//			int end = rand() % m_vertNum;
//			if (start == end) continue;
//			// Avoid duplicates
//			if (graph.vertex(start)->connectedTo(end)) continue;
//            graph.addEdge(start, end);
//		}
//	}
//	// Cleanup the graph
//	//Algs::Processing::RemoveDeadends removeDeadends;
//	DS::BaseGraph finalOutput;
//	std::set<int> removedNodes;
//	//removeDeadends.apply(outGraph, outSource, outSink, finalOutput, outSource, outSink, removedNodes);
//	//outGraph = finalOutput;
//
//	/*for (int i = 0; i < m_vertNum; i++)
//	{
//		if (removedNodes.find(i) == removedNodes.end())
//		{
//			outputPoints.push_back(points[i].first);
//		}
//	}*/
//}
//
//void src::Helpers::GraphGenerator::setNumberOfVertices(int vertexNum)
//{
//	m_vertNum = vertexNum;
//}
//
//void src::Helpers::GraphGenerator::generateGraphFromTriangulation(double graphDensity, int& outSource, int& outSink,
//                                                                  DS::BaseGraph& outGraph,
//                                                                  std::vector<Kernel::Point_2>& outputPoints)
//{
//
//	using namespace CGAL;
//	//using R = Simple_cartesian<double>; //Kernel
//	using Point = Kernel::Point_2; //Point type
//	using Creator = Creator_uniform_2<double, Point>; // Uniform creator algorithm
//	using PointsList = std::vector<std::pair<Point,int>>; //Redef for a list of points for readability.
//
//	using TriStructure = CGAL::Triangulation_data_structure_2<
//		CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>,
//		CGAL::Triangulation_face_base_2<Kernel>
//	>;
//	using DelaunayTriangulation = CGAL::Delaunay_triangulation_2<Kernel, TriStructure>;
//
//	// Determine number of edges in complete graph.
//	const int maxEdges = ((m_vertNum) * (m_vertNum - 1)) / 2;
//	const int edgeNum = (int)(graphDensity * maxEdges);
//
//	PointsList points;
//	points.reserve(m_vertNum);
//	// Random points in square [-200,200] for both x and y
//	Random_points_in_square_2<Point, Creator> randomGen(200.); 
//	// Select elements from the generator
//	for(int i = 0; i < m_vertNum; i++)
//	{
//		points.emplace_back(*randomGen, i);
//		++randomGen;
//	}
//	//std::copy_n(randomGen, m_vertNum, std::back_inserter(points));
//
//	// Generate the associated Delaunay triangulation as a starting point.
//	DelaunayTriangulation t;
//	t.insert(points.begin(), points.end());
//
//	// Determine source and sink on convex hull.
//	auto circ = t.infinite_vertex()->incident_vertices();
//	auto curr = circ;
//
//	using VertH = DelaunayTriangulation::Vertex_handle;
//	VertH minVert;
//	VertH maxVert;
//	double minX = std::numeric_limits<double>::max();
//	double maxX = -std::numeric_limits<double>::max();
//	do
//	{
//		if(curr->point().x() < minX)
//		{
//			minX = curr->point().x();
//			minVert = curr;
//		}
//		if (curr->point().x() > maxX)
//		{
//			maxX = curr->point().x();
//			maxVert = curr;
//		}
//		++curr;
//	} while (curr != circ);
//
//	// Record known edges
//	std::unordered_set<std::pair<int, int>, PairHash<int,int>> edges;
//
//	// Construct outgraph
//	outGraph.allocateVertices(m_vertNum);
//	outSource = minVert->info();
//	outSink = maxVert->info();
//	Kernel::Vector_2 dir = maxVert->point() - minVert->point();
//	dir = Kernel::Vector_2(dir.x(), 0);
//	for(auto e : t.finite_edges())
//	{
//		auto v0 = e.first->vertex(e.first->cw(e.second));
//		auto v1 = e.first->vertex(e.first->ccw(e.second));
//		auto edgeDir = v1->point() - v0->point();
//		if(edgeDir * dir > 0)
//		{
//			outGraph.addEdge(v0->info(), v1->info());
//			edges.insert(std::make_pair(v0->info(), v1->info()));
//		}
//		else
//		{
//			outGraph.addEdge(v1->info(), v0->info());
//			edges.insert(std::make_pair(v1->info(), v0->info()));
//		}
//	}
//	// Add remainder edges
//	for(int i = outGraph.number_of_edges(); i < edgeNum; i++)
//	{
//		int start = rand() % m_vertNum;
//		int end = rand() % m_vertNum;
//		if (start == end) continue;
//		// Avoid duplicates
//		if (edges.find(std::make_pair(start, end)) != edges.end()) continue;
//		outGraph.addEdge(start, end);
//	}
//	std::unordered_set<int> removedVerts;
//	postProcess(edges, outSource, outSink, outGraph, removedVerts);
//
//	outputPoints.reserve(outGraph.number_of_vertices() - removedVerts.size());
//	for(int i = 0; i < points.size(); i++)
//	{
//		if(removedVerts.find(i) == removedVerts.end())
//		{
//			outputPoints.push_back(std::get<0>(points[i]));
//		}
//	}
//}
//
//void src::Helpers::GraphGenerator::twoWayPlanar(DS::BaseGraph& outGraph,
//    std::vector<MovetkGeometryKernel::MovetkPoint>& points)
//{
//    auto makePnt = movetk_core::MakePoint<MovetkGeometryKernel>();
//    std::vector<Kernel::Point_2> initialPoints;
//    outGraph.clear();
//    points.clear();
//    twoWayPlanar(outGraph, initialPoints);
//    points.reserve(initialPoints.size());
//    for(auto pnt: initialPoints)
//    {
//        const std::vector<MovetkGeometryKernel::NT> coords = { pnt.x(), pnt.y() };
//        static_assert(std::is_same_v<MovetkGeometryKernel::NT, std::decay_t<decltype(*coords.begin())>>, "Incorrect val");
//        points.push_back(makePnt(coords.begin(), coords.end()));
//    }
//}
//
//void src::Helpers::GraphGenerator::twoWayPlanar(DS::BaseGraph& outGraph,
//	std::vector<Kernel::Point_2>& points)
//{
//    // Clear any stuff that may be in the graph already.
//    outGraph.clear();
//    points.clear();
//
//    using namespace CGAL;
//    //using R = Simple_cartesian<double>; //Kernel
//    using Point = Kernel::Point_2; //Point type
//    using Creator = Creator_uniform_2<double, Point>; // Uniform creator algorithm
//    using PointsList = std::vector<std::pair<Point, int>>; //Redef for a list of points for readability.
//
//    using TriStructure = CGAL::Triangulation_data_structure_2<
//        CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>,
//        CGAL::Triangulation_face_base_2<Kernel>
//    >;
//    using DelaunayTriangulation = CGAL::Delaunay_triangulation_2<Kernel, TriStructure>;
//
//    // 
//    const int maxEdges = ((m_vertNum) * (m_vertNum - 1)) / 2;
//
//    // TODO: random graph from triangulation of a random pointset.
//    points.reserve(m_vertNum);
//    std::vector<std::pair<Point, int>> localPoints;
//    localPoints.reserve(m_vertNum);
//    // Random points in square [-200,200] for both x and y
//    Random_points_in_square_2<Point, Creator> randomGen(200.);
//    // Select elements from the generator
//    for (int i = 0; i < m_vertNum; i++)
//    {
//        localPoints.emplace_back(*randomGen, i);
//        points.push_back(localPoints.back().first);
//        ++randomGen;
//    }
//    // Generate the associated Delaunay triangulation as a starting point.
//    DelaunayTriangulation t;
//    t.insert(localPoints.begin(), localPoints.end());
//
//    // Determine source and sink on convex hull.
//    auto circ = t.infinite_vertex()->incident_vertices();
//    auto curr = circ;
//
//    using VertH = DelaunayTriangulation::Vertex_handle;
//    VertH minVert;
//    VertH maxVert;
//    double minX = std::numeric_limits<double>::max();
//    double maxX = -std::numeric_limits<double>::max();
//    do
//    {
//        if (curr->point().x() < minX)
//        {
//            minX = curr->point().x();
//            minVert = curr;
//        }
//        if (curr->point().x() > maxX)
//        {
//            maxX = curr->point().x();
//            maxVert = curr;
//        }
//        ++curr;
//    } while (curr != circ);
//
//    // Record known edges
//    std::unordered_set<std::pair<int, int>, pair_hash> edges;
//
//    // Construct outgraph
//    outGraph.allocateVertices(m_vertNum);
//
//    for(auto edge : t.finite_edges())
//    {
//        auto f = edge.first;
//        auto c = edge.second;
//        auto v0 = f->vertex(f->cw(c));
//        auto v1 = f->vertex(f->ccw(c));
//        // Add two-way edges
//        outGraph.addEdge(v0->info(), v1->info());
//        outGraph.addEdge(v1->info(), v0->info());
//    }
//}
//
//void src::Helpers::GraphGenerator::generateGraphFromTriangulationWithNeighbours(double graphDensity, int maxNeigh,
//	int& outSource, int& outSink, DS::BaseGraph& outGraph, std::vector<Kernel::Point_2>& outputPoints)
//{
//	using namespace CGAL;
//	//using R = Simple_cartesian<double>; //Kernel
//	using Point = Kernel::Point_2; //Point type
//	using Creator = Creator_uniform_2<double, Point>; // Uniform creator algorithm
//	using PointsList = std::vector<std::pair<Point, int>>; //Redef for a list of points for readability.
//
//	using TriStructure = CGAL::Triangulation_data_structure_2<
//		CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>,
//		CGAL::Triangulation_face_base_2<Kernel>
//	>;
//	using DelaunayTriangulation = CGAL::Delaunay_triangulation_2<Kernel, TriStructure>;
//
//	// 
//	const int maxEdges = ((m_vertNum) * (m_vertNum - 1)) / 2;
//	const int edgeNum = (int)(graphDensity * maxEdges);
//
//	// TODO: random graph from triangulation of a random pointset.
//	PointsList points;
//	points.reserve(m_vertNum);
//	// Random points in square [-200,200] for both x and y
//	Random_points_in_square_2<Point, Creator> randomGen(200.);
//	// Select elements from the generator
//	for (int i = 0; i < m_vertNum; i++)
//	{
//		points.emplace_back(*randomGen, i);
//		++randomGen;
//	}
//	//std::copy_n(randomGen, m_vertNum, std::back_inserter(points));
//
//	// Generate the associated Delaunay triangulation as a starting point.
//	DelaunayTriangulation t;
//	t.insert(points.begin(), points.end());
//
//	// Determine source and sink on convex hull.
//	auto circ = t.infinite_vertex()->incident_vertices();
//	auto curr = circ;
//
//	using VertH = DelaunayTriangulation::Vertex_handle;
//	VertH minVert;
//	VertH maxVert;
//	double minX = std::numeric_limits<double>::max();
//	double maxX = -std::numeric_limits<double>::max();
//	do
//	{
//		if (curr->point().x() < minX)
//		{
//			minX = curr->point().x();
//			minVert = curr;
//		}
//		if (curr->point().x() > maxX)
//		{
//			maxX = curr->point().x();
//			maxVert = curr;
//		}
//		++curr;
//	} while (curr != circ);
//
//	// Record known edges
//	std::unordered_set<std::pair<int, int>, pair_hash> edges;
//
//	// Construct outgraph
//	outGraph.allocateVertices(m_vertNum);
//	outSource = minVert->info();
//	outSink = maxVert->info();
//	Kernel::Vector_2 dir = maxVert->point() - minVert->point();
//	for (auto e : t.finite_edges())
//	{
//		auto v0 = e.first->vertex(e.first->cw(e.second));
//		auto v1 = e.first->vertex(e.first->ccw(e.second));
//		auto edgeDir = v1->point() - v0->point();
//		if (edgeDir * dir > 0)
//		{
//			outGraph.addEdge(v0->info(), v1->info());
//			edges.insert(std::make_pair(v0->info(), v1->info()));
//		}
//		else
//		{
//			outGraph.addEdge(v1->info(), v0->info());
//			edges.insert(std::make_pair(v1->info(), v0->info()));
//		}
//	}
//
//	// Create list of vertices
//	std::vector<VertH> vertexHandles;
//	for(auto v : t.finite_vertex_handles())
//	{
//		vertexHandles.push_back(v);
//	}
//
//	// Add remainder edges
//	for (int i = outGraph.number_of_edges(); i < edgeNum; i++)
//	{
//		int start = rand() % m_vertNum;
//
//		// Find neighbours to attach to
//		std::vector<VertH> neighbs;
//		CGAL::nearest_neighbors(t, vertexHandles[start], maxNeigh, std::back_inserter(neighbs));
//
//		int end = rand() % maxNeigh;
//		int vEndInd = neighbs[end]->info();
//		// Avoid duplicates
//		if (edges.find(std::make_pair(start, vEndInd)) != edges.end()) continue;
//		outGraph.addEdge(start, vEndInd);
//	}
//	// For now, we don't do deletion.
//	Algs::Processing::IsReachable reachability;
//
//	// As long as the source and sink are not connected, generate extra edges
//	while (!reachability.apply(&outGraph, outSource, outSink))
//	{
//		std::cout << "[GraphGen]\tGenerating more edges to connect source and sink" << std::endl;
//		// Generate more edges to hopefully get a connected graph.
//		for (int j = 0; j < 10; j++)
//		{
//			int start = rand() % m_vertNum;
//
//			// Find neighbours to attach to
//			std::vector<VertH> neighbs;
//			CGAL::nearest_neighbors(t, vertexHandles[start], maxNeigh, std::back_inserter(neighbs));
//
//			int end = rand() % maxNeigh;
//			int vEndInd = neighbs[end]->info();
//			// Avoid duplicates
//			if (edges.find(std::make_pair(start, vEndInd)) != edges.end()) continue;
//			outGraph.addEdge(start, vEndInd);
//		}
//	}
//	// Cleanup the graph
//	//Algs::Processing::RemoveDeadends removeDeadends;
//	DS::BaseGraph finalOutput;
//	std::set<int> removedNodes;
//	//removeDeadends.apply(outGraph, outSource, outSink, finalOutput, outSource, outSink, removedNodes);
//	//outGraph = finalOutput;
//
//	for (int i = 0; i < m_vertNum; i++)
//	{
//		if (removedNodes.find(i) == removedNodes.end())
//		{
//			outputPoints.push_back(points[i].first);
//		}
//	}
//}
//
//void src::Helpers::GraphGenerator::generateRandomGraph(double targetGraphDensity, int& source, int& sink,
//                                                       DS::BaseGraph& outputGraph)
//{
//	DS::BaseGraph graph(m_vertNum);
//	const int maxEdges = ((m_vertNum) * (m_vertNum - 1)) / 2;
//	const int edgeNum = (int)(targetGraphDensity * maxEdges);
//
//
//	// Hacky
//	std::unordered_set<std::pair<int, int>, pair_hash> edges;
//	int finalEdgeNum = 0;
//	for (int j = 0; j < edgeNum; j++)
//	{
//		int start = rand() % m_vertNum;
//		int end = rand() % m_vertNum;
//		if (start == end) continue;
//		// Avoid duplicates
//		if (edges.find(std::make_pair(start, end)) != edges.end()) continue;
//		graph.addEdge(start, end);
//		// Generate random weight
//		finalEdgeNum++;
//	}
//	Algs::Processing::IsReachable reachability;
//
//	// As long as the source and sink are not connected, generate extra edges
//	while (!reachability.apply(&graph, source, sink))
//	{
//		std::cout << "[GraphGen]\tGenerating more edges to connect source and sink" << std::endl;
//		// Generate more edges to hopefully get a connected graph.
//		for (int j = 0; j < 10; j++)
//		{
//			int start = rand() % m_vertNum;
//			int end = rand() % m_vertNum;
//			if (start == end) continue;
//			// Avoid duplicates
//			if (edges.find(std::make_pair(start, end)) != edges.end()) continue;
//			graph.addEdge(start, end);
//			// Generate random weight
//			finalEdgeNum++;
//		}
//	}
//	// Cleanup the graph
//	Algs::Processing::RemoveDeadends removeDeadends;
//	std::set<int> removedNodes;
//	removeDeadends.apply(graph, source, sink, outputGraph, source, sink, removedNodes);
//}
