#include <LoopsAlgs/Frechet/LowMemoryFrechetData.h>
#include <LoopsLib/Math/Vector.h>
#include <movetk/geom/GeometryInterface.h>
using namespace LoopsLib;

void LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::clear()
{
    lrPointers.clear();
    polyline.clear();

    WhiteIntervals.clear();
    m_view.clear();
}

std::pair<std::size_t, std::size_t> LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::intervalRangeForLrPointer(
    std::size_t targetVert, const Interval& interval) const
{
    // Return overlapping white intervals
    EndPointView view(&WhiteIntervals[targetVert]);
    auto lower = view.lower_bound(interval.min); //First el >= min
    auto upper = view.lower_bound(interval.max); //First el >= min

    if(!upper->isMax)
    {
        throw std::runtime_error("Invalid lrpointer given to intervalRangeForLrPointer(). Max should point to end of an interval");
    }

    return std::make_pair(lower->interId, upper->interId);
}

std::size_t LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::intervalForVertexLocation(std::size_t targetVert, NT pos) const
{
    EndPointView view(&WhiteIntervals[targetVert]);
    // Returns first element such that its value is >= pos.
    auto higherElement = view.lower_bound(pos);
    if(higherElement == view.end())
    {
        throw std::invalid_argument("Gave position to intervalForVertexLocation() " + std::to_string(pos) + " that is above highest white interval");
    }
    if(higherElement->value == pos)
    {
        return higherElement->interId;
    }
    if(!higherElement->isMax)
    {
        throw std::invalid_argument("Gave position outside white interval");
    }
    return higherElement->interId;
}

long long LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::firstIntervalAboveLocation(std::size_t targetVert,
    NT pos) const
{

    EndPointView view(&WhiteIntervals[targetVert]);
    auto higherElement = view.lower_bound(pos);
    if (higherElement == view.end()) return -1;
    return higherElement->interId;
}

std::size_t LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::number_of_vertices() const
{
    return m_view.number_of_vertices();
}

LoopsAlgs::Frechet::Interval& LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::getWhiteInterval(
    std::size_t targetVert, const NT& lowEndpoint)
{
    const auto iId = intervalForVertexLocation(targetVert, lowEndpoint);
    return WhiteIntervals[targetVert][iId];
}
const LoopsAlgs::Frechet::Interval& LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::getWhiteInterval(
    std::size_t targetVert, const NT& lowEndpoint) const
{
    const auto iId = intervalForVertexLocation(targetVert, lowEndpoint);
    return WhiteIntervals[targetVert][iId];
}

bool LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::isPotentialEndpoint(std::size_t vert, std::size_t interval) const
{
    return interval == WhiteIntervals[vert].size() - 1 && WhiteIntervals[vert][interval].max > polyline.size() - 1 &&
        WhiteIntervals[vert][interval].containsApprox(polyline.size(), 0.0001);
}


void LoopsAlgs::Frechet::LowMemory::LowMemoryFrechetGraphData::setup(const LoopsAlgs::Frechet::Graph* graph,
                                                  const std::vector<LoopsAlgs::Frechet::Point>& lineLocations)
{
    auto makSeg = movetk_core::MakeSegment<LoopsAlgs::Frechet::Kernel>();
    for (int i = 0; i < lineLocations.size() - 1; ++i)
    {
        polyline.push_back(makSeg(lineLocations[i], lineLocations[i + 1]));
    }
    m_graph = graph;

    lrPointers.clear();
    lrPointers.resize(graph->number_of_vertices(), std::vector<PointerMap>{});
    for (int i = 0; i < graph->number_of_vertices(); ++i)
    {
        lrPointers[i].resize(polyline.size(), PointerMap{});
    }
    m_view.setGraph(graph);
}

double LoopsAlgs::Frechet::LowMemory::FrechetGraphComputations::fdiTimeMs() const
{
    return m_fdiTimeMs;
}

double LoopsAlgs::Frechet::LowMemory::FrechetGraphComputations::lrPointersTimeMs() const
{
    return m_lrPointersMs;
}

void LoopsAlgs::Frechet::LowMemory::FrechetGraphComputations::constructFrechetView(LowMemoryFrechetGraphData& data) const
{
    // Query the point index of locations that are possibly within epsilon distance.
    std::vector<LoopsAlgs::Frechet::Graph::Id_t> els;
    const auto& index = data.m_graph->getIndex();
    if(index.isEmpty())
    {
        throw std::runtime_error("Cannot construct Frechet view when index on graph is empty");
    }
    std::size_t ind = 0;
    const auto epsilon = m_epsilon;
    for (; ind < data.polyline.size(); ++ind)
    {
        const auto& seg = data.polyline[ind];
        auto x = seg.get().vertex(0).x();
        auto y = seg.get().vertex(0).y();
        auto x2 = seg.get().vertex(1).x();
        auto y2 = seg.get().vertex(1).y();
        index.containedInDisk(x, y, epsilon, els);
        if (ind == data.polyline.size() - 1)
        {
            index.containedInDisk(x2, y2, epsilon, els);
        }
        auto len = std::hypot((x2 - x) ,(y2 - y));
        auto rotX = -(y2 - y);
        auto rotY = x2 - x;

        auto pnt = [](NT x, NT y) { return std::make_pair(x, y); };
        //
        index.containedInPoly({
            pnt(x + rotX / len * epsilon, y + rotY / len * epsilon),
            pnt(x - rotX / len * epsilon, y - rotY / len * epsilon),
            pnt(x2 - rotX / len * epsilon, y2 - rotY / len * epsilon),
            pnt(x2 + rotX / len * epsilon, y2 + rotY / len * epsilon),
        }, els);
    }
    auto& verts = data.m_view.availableVertices();
    verts.clear();
    verts.insert(els.begin(), els.end());
    data.m_view.updateAvailableVerts();
}

LoopsAlgs::Frechet::Interval LoopsAlgs::Frechet::LowMemory::FrechetGraphComputations::
computeIntersectionInterval(const Segment& seg, const Point& p, const NT& eps)
{
    LoopsLib::Math::Vec2<NT> v0{ seg[0].get().x(),seg[0].get().y() };
    LoopsLib::Math::Vec2<NT> v1{ seg[1].get().x(),seg[1].get().y() };
    LoopsLib::Math::Vec2<NT> pnt{ p.get().x(),p.get().y() };

    if((v1-v0).length() < 0.0001)
    {
        bool isWithinReach = (pnt - v0).length() < eps;
        return isWithinReach ? Interval(-1, 2) : Interval{};
    }

    IntervalPolynomial poly;
    poly.compute(v0, v1, pnt);

    return poly.getIntervalUnclamped(eps);
}

LoopsAlgs::Frechet::Interval LoopsAlgs::Frechet::LowMemory::
FrechetGraphComputations::computeIntersectionInterval(const Point& s0, const Point& s1, const Point& p, const NT& eps)
{
    LoopsLib::Math::Vec2<NT> v0{ s0.get().x(),s0.get().y() };
    LoopsLib::Math::Vec2<NT> v1{ s1.get().x(),s1.get().y() };
    LoopsLib::Math::Vec2<NT> pnt{ p.get().x(),p.get().y() };

    IntervalPolynomial poly;
    poly.compute(v0, v1, pnt);

    return poly.getIntervalUnclamped(eps);
}

void LoopsAlgs::Frechet::LowMemory::FrechetGraphComputations::computeFDi(LowMemoryFrechetGraphData& data)
{
    // Query the point index
    if (data.m_view.number_of_vertices() == 0)
    {
        constructFrechetView(data);
    }
    data.m_epsilon = m_epsilon;

    LoopsLib::Helpers::ScopeTimer timer(&m_fdiTimeMs);
    auto* graphView = &data.m_view;
    data.WhiteIntervals.resize(graphView->number_of_vertices(), {});


    const auto polyEdgeCount = data.polylineEdgeCount();

    // Reserve interval list for single vertex
    std::vector<Interval> intervals;
    intervals.resize(polyEdgeCount, Interval{});

    //Foreach vertex in the graph, compute the FD_i strip.
    for (int i = 0; i < graphView->number_of_vertices(); ++i)
    {
        intervals.assign(intervals.size(), Interval{});
        auto* v = graphView->vertexByIndex(i);

        // ID of the current interval being processed
        int currentInterval = -1;

        // Was the previous cell not empty
        bool prevNotEmpty = false;
        bool prevRightIncluded = false;

        // Compute white interval per cell
        for (int j = 0; j < polyEdgeCount; ++j)
        {
            // Computes the interval in that is white, unclamped.
            const auto intervalResult = computeIntersectionInterval(data.polyline[j], data.m_graph->locations()[v->id()],
                                                              m_epsilon);
            // Save the results for the cell. NOTE: this interval is unclamped! Clamp to [0,1] when using for actual interval computation!
            intervals[j] = intervalResult;

            // Verify we don't create abominations
            if (std::isnan(intervals[j].min) || std::isnan(intervals[j].max))
            {
                std::cerr << "Encountered NaN in cell white intervals" << std::endl;
            }

            // Check if this is a new whiteinterval or connected to the previous interval.
            if(!intervals[j].isEmpty())
            {
                // New white interval
                if(!prevNotEmpty || !prevRightIncluded)
                {
                    // Finish the previous white interval
                    if (prevNotEmpty)
                    {
                        data.WhiteIntervals[i].back().max = j - 1 + std::min(intervals[j - 1].max,(NT)1.0);
                    }
                    ++currentInterval;
                    data.WhiteIntervals[i].emplace_back(j + std::max(intervals[j].min,(NT)0), -1);
                }
                prevNotEmpty = true;

               
                prevRightIncluded = intervals[j].contains(1.0);
            }
            else
            {
                if(prevNotEmpty)
                {
                    data.WhiteIntervals[i].back().max = j - 1 + std::min(intervals[j - 1].max,(NT)1.0);
                }
                prevNotEmpty = false;
                prevRightIncluded = false;
            }
        }
        // Last cell white interval: set 
        if (!intervals.back().isEmpty())
        {
            data.WhiteIntervals[i].back().max = data.polylineEdgeCount()-1 + std::min((NT)1.0, intervals.back().max);
        }
    }
}

void LoopsAlgs::Frechet::LowMemory::FrechetGraphComputations::computeLeftRightPointersBF(LowMemoryFrechetGraphData& data)
{
    LoopsLib::Helpers::ScopeTimer timer(&m_lrPointersMs);
    using MakeSegment = typename movetk_core::MakeSegment<Kernel>;
    auto* graphView = &data.m_view;

    // Lrpointers per vertex per cell
    data.lrPointers.resize(graphView->number_of_vertices(), std::vector<PointerMap>{});
    for (int i = 0; i < graphView->number_of_vertices(); ++i)
    {
        data.lrPointers[i].resize(data.WhiteIntervals[i].size(), PointerMap{});
    }

    // Foreach interval in an FD_i strip, compute the l- and r-pointer per adjacent vertex.
    for (int i = 0; i < graphView->number_of_vertices(); ++i)
    {
        // Ignore empty
        if (data.WhiteIntervals[i].empty()) continue;

        // Active vertex
        auto* vert = graphView->vertexByIndex(i);
        // Number of segments of the polyline
        const auto polysize = data.polylineEdgeCount();

        const auto v0 = data.m_graph->locations()[vert->id()];

        // Loop over outgoing edges
        for (auto* e : graphView->outEdges(vert->id()))
        {
            // ID of the target vertex, in the graph view!!
            const auto vId = graphView->indexForVertex(e->m_sink->id());

            // Ignore empty
            if (data.WhiteIntervals[vId].empty()) continue;

            // Precompute between intervals: intervals at some vertex of the polyline for the edge between the vertices of edge e
            const auto v1 = data.m_graph->locations()[e->m_sink->id()];
            std::vector<Interval> between;
            for (std::size_t i = 0; i < data.polylineEdgeCount(); ++i)
            {
                auto res = computeIntersectionInterval(v0, v1, data.polyline[i].get().vertex(0), m_epsilon);
                between.push_back(res);
            }

            const auto& targetWis = data.WhiteIntervals[vId];

            EndPointView view(&data.WhiteIntervals[vId]);
            for (std::size_t wi = 0; wi < data.WhiteIntervals[i].size(); ++i)
            {

                const auto& currentWi = data.WhiteIntervals[i][wi];

                auto lower = view.lower_bound(currentWi.min);

                // First endpoint that is higher than the lower part of the active white interval is not present:
                // no intervals reachable
                if (lower == view.end()) continue;

                // The leftright pointer to fill
                long long minReachable = std::numeric_limits<long long>::max();
                long long maxReachable = -std::numeric_limits<long long>::max();

                auto activeTargetInterval = lower->interId;

                auto startCell = (std::size_t)currentWi.min;
                // For every cell, start at vert, going over e.
                NT lowest = 0;
                for (long long k = startCell; k < polysize; ++k)
                {
                    if((NT)k > targetWis[activeTargetInterval].max)
                    {
                        ++activeTargetInterval;
                        if (activeTargetInterval == targetWis.size()) break;
                    }
                    // Check if the current cell interval overlaps. We already know that it was reachable

                    if(Interval(k,k+1).intersectsWith(targetWis[activeTargetInterval]))
                    {
                        minReachable = std::min(minReachable, activeTargetInterval);
                        maxReachable = std::max(maxReachable, activeTargetInterval);
                    }
                    if (k >= polysize - 1) break;
                    auto inter = between[k + 1];
                    if (inter.isEmpty() || lowest > inter.max) break;
                    lowest = std::max(inter.min, lowest);
                }
                // No reachable intervals found
                if (minReachable > maxReachable) continue;

                data.lrPointers[i][wi][vId] = LeftRightPointers{
                    minReachable,
                    maxReachable
                };
            }
        }

        
    }
}
