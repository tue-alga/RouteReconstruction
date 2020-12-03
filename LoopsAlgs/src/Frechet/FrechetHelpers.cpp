#include <LoopsAlgs/Frechet/FrechetHelpers.h>
#include <LoopsLib/Math/Vector.h>
#include <movetk/geom/GeometryInterface.h>
using namespace LoopsLib;

void LoopsAlgs::Frechet::IntervalPolynomial::compute(const LoopsLib::Math::Vec2<NT>& seg0,
                                                     const LoopsLib::Math::Vec2<NT>& seg1,
                                                     const LoopsLib::Math::Vec2<NT>& point)
{
    auto dir = seg1 - seg0;
    segLen = dir.length();
    // Projected value on line through segment,
    // with origin at start of segment
    proj = (point - seg0).dot(dir) / segLen;
    perp = (point - seg0 - dir.normalized() * proj).length();
    type = 'i';
    if (proj > segLen)
    {
        type = 'a';
        minEps = (point - seg1).length();
    }
    else if (proj < 0)
    {
        type = 'b';
        minEps = (point - seg0).length();
    }
    else
    {
        minEps = perp;
    }
}

NT LoopsAlgs::Frechet::IntervalPolynomial::lowerUnclamped(NT eps) const
{
    if (eps < minEps) return std::numeric_limits<NT>::max();
    return proj / segLen - sqrtDiff(sq(eps), sq(perp)) / segLen;
}

NT LoopsAlgs::Frechet::IntervalPolynomial::upperUnclamped(NT eps) const
{
    if (eps < minEps) return std::numeric_limits<NT>::lowest();
    return proj / segLen + sqrtDiff(sq(eps), sq(perp)) / segLen;
}

LoopsAlgs::Frechet::Interval LoopsAlgs::Frechet::IntervalPolynomial::getIntervalUnclamped(NT eps) const
{
    if (eps < minEps) return Interval{};
    return Interval(lowerUnclamped(eps), upperUnclamped(eps));
}


bool LoopsAlgs::Frechet::IntervalPolynomial::containsLeft(NT eps) const
{
    if (eps < minEps) return false;
    switch (type)
    {
    case 'b': return true;
    case 'i': return proj / segLen - sqrtDiff(sq(eps) ,sq(perp)) / segLen <= 0;
    case 'a': return proj / segLen - sqrtDiff(sq(eps) ,sq(perp)) / segLen <= 0;
    default:
        assert(false);
        throw std::runtime_error("Invalid critical value type");
    }
}

bool LoopsAlgs::Frechet::IntervalPolynomial::containsRight(NT eps) const
{
    if (eps < minEps) return false;
    switch (type)
    {
    case 'b': return proj / segLen + sqrtDiff(sq(eps) , sq(perp)) / segLen >= 1;
    case 'i': return proj / segLen + sqrtDiff(sq(eps), sq(perp)) / segLen >= 1;
    case 'a': return true;
    default:
        assert(false);
        throw std::runtime_error("Invalid critical value type");
    }
}

NT LoopsAlgs::Frechet::IntervalPolynomial::lower(NT eps) const
{
    if (eps < minEps) return std::numeric_limits<NT>::max();
    switch (type)
    {
    case 'b': return 0;
    case 'i': return clamp(proj / segLen - sqrtDiff(sq(eps), sq(perp)) / segLen, 0, 1);
    case 'a': return clamp(proj / segLen - sqrtDiff(sq(eps), sq(perp)) / segLen, 0, 1);
    default:
        assert(false);
        throw std::runtime_error("Invalid critical value type");
    }
}

NT LoopsAlgs::Frechet::IntervalPolynomial::upper(NT eps) const
{
    if (eps < minEps) return -std::numeric_limits<NT>::max();
    switch (type)
    {
    case 'b': return clamp(proj / segLen + sqrtDiff(sq(eps), sq(perp)) / segLen, 0, 1);
    case 'i': return clamp(proj / segLen + sqrtDiff(sq(eps), sq(perp)) / segLen, 0, 1);
    case 'a': return (NT)1.0;
    default:
        assert(false);
        throw std::runtime_error("Invalid critical value type");
    }
}

LoopsAlgs::Frechet::Interval LoopsAlgs::Frechet::IntervalPolynomial::getInterval(NT eps) const
{
    // TODO warn invalid?
    if (type == 'n') return Interval{};
    if (eps < minEps) return Interval{};
    return Interval(lower(eps), upper(eps));
}

std::size_t LoopsAlgs::Frechet::StrongFrechetGraphData::byteSizeEstimate() const
{
    std::size_t totalSize = 0;
    totalSize += sizeof m_graph;
    totalSize += sizeof(polyline) + polyline.size() * sizeof(Segment);
    for (const auto& el : FDiWhiteIntervals)
    {
        totalSize += sizeof(Interval) * el.size() + sizeof(el);
    }
    for (const auto& el : WhiteIntervals)
    {
        totalSize += sizeof(Interval) * el.size() + sizeof(el);
    }
    for (const auto& el : IntervalInclusions)
    {
        totalSize += sizeof(el[0]) * el.size() + sizeof(el);
    }
    for (const auto& el : CellToIntervalId)
    {
        totalSize += sizeof(el[0]) * el.size() + sizeof(el);
    }
    for (const auto& el : IntervalIdToCell)
    {
        totalSize += sizeof(el[0]) * el.size() + sizeof(el);
    }
    for (const auto& el : lrPointers)
    {
        totalSize += sizeof(el[0]) * el.size() + sizeof(el);
        for (const auto& pair : el)
        {
            totalSize += pair.size() * (sizeof(LoopsLib::DS::BaseGraph::Id_t) + sizeof(PathPointer));
        }
    }
    totalSize += sizeof(GraphView) + 2 * sizeof(LoopsLib::DS::BaseGraph::Id_t) * m_view.number_of_vertices();

    return totalSize;
}

void LoopsAlgs::Frechet::StrongFrechetGraphData::clear()
{
    lrPointers.clear();
    polyline.clear();

    FDiWhiteIntervals.clear();
    WhiteIntervals.clear();
    IntervalInclusions.clear();
    CellToIntervalId.clear();
    IntervalIdToCell.clear();
    m_view.clear();
}

std::pair<std::size_t, std::size_t> LoopsAlgs::Frechet::StrongFrechetGraphData::intervalRangeForLrPointer(
    std::size_t targetVert, const Interval& interval) const
{
    auto iIdMax = CellToIntervalId[targetVert][(std::size_t)interval.max];
    if(iIdMax < 0)
    {
        iIdMax = CellToIntervalId[targetVert][((std::size_t)interval.max) - 1];
        if (iIdMax < 0) throw std::runtime_error("Invalid upperbound for interval range for LR pointer");
    }

    return std::make_pair(CellToIntervalId[targetVert][(std::size_t)interval.min], iIdMax);
}

std::size_t LoopsAlgs::Frechet::StrongFrechetGraphData::intervalForVertexLocation(std::size_t targetVert, NT pos) const
{
    if (pos < 0)
    {
        throw std::invalid_argument("Gave negative position to intervalForVertexLocation()" + std::to_string(pos));
    }
    auto localPos = pos;
    if (pos >= polylineEdgeCount()) localPos = pos - 0.1;
    std::size_t locI = (std::size_t)localPos;
    auto iId = CellToIntervalId[targetVert][locI];
    if (iId == -1)
    {
        if (std::abs(localPos - (NT)locI) < 0.00001)
        {
            iId = CellToIntervalId[targetVert][locI - 1];
        }
        if (iId == -1)
        {
            std::stringstream message;
            message << "Accessing non-white interval: v " << targetVert << ", pos " << pos << "\n";
            std::size_t wi = 0;
            for (const auto& el : WhiteIntervals[targetVert])
            {
                message << "Wi " << wi << ": " << el.min << "," << el.max << "\n";
                ++wi;
            }
            throw std::runtime_error(message.str());
        }
    }
    return iId;
}

int LoopsAlgs::Frechet::StrongFrechetGraphData::firstIntervalAbove(std::size_t targetVert, NT pos) const
{
    if (WhiteIntervals[targetVert].empty()) return -1;
    int index = 0;
    // WhiteIntervals should be sorted (which is always assumed)
    for(const auto& wi : WhiteIntervals[targetVert])
    {
        if (wi.contains(pos) || wi.min > pos) return index;
        ++index;
    }
    return -1;
}

std::size_t LoopsAlgs::Frechet::StrongFrechetGraphData::polylineEdgeCount() const
{
    return polyline.size();
}

bool LoopsAlgs::Frechet::StrongFrechetGraphData::isWhitePoint(std::size_t targetVert, NT pos) const
{
    // This should actually not be called?
    if (FDiWhiteIntervals[targetVert].empty()) return false;

    std::size_t cell = static_cast<std::size_t>(std::min(
        static_cast<long long>(FDiWhiteIntervals[targetVert].size() - 1),
        std::max(0LL, static_cast<long long>(pos))));
    const auto& interval = FDiWhiteIntervals[targetVert][cell];
    return !interval.isEmpty() && interval.containsApprox(pos - (LoopsLib::NT)cell, 0.001);
}

std::size_t LoopsAlgs::Frechet::StrongFrechetGraphData::positionToCell(NT freeSpacePos) const
{
    // TODO: check if truly invalid position given?
    return (std::size_t)std::max(0LL,
                                 std::min((long long)polylineEdgeCount()-1, (long long)freeSpacePos)
    );
}

std::size_t LoopsAlgs::Frechet::StrongFrechetGraphData::number_of_vertices() const
{
    return m_view.number_of_vertices();
}

LoopsAlgs::Frechet::Interval& LoopsAlgs::Frechet::StrongFrechetGraphData::getWhiteInterval(
    std::size_t targetVert, const NT& lowEndpoint)
{
    const auto iId = intervalForVertexLocation(targetVert, lowEndpoint);
    return WhiteIntervals[targetVert][iId];
}
const LoopsAlgs::Frechet::Interval& LoopsAlgs::Frechet::StrongFrechetGraphData::getWhiteInterval(
    std::size_t targetVert, const NT& lowEndpoint) const
{
    const auto iId = intervalForVertexLocation(targetVert, lowEndpoint);
    return WhiteIntervals[targetVert][iId];
}

bool LoopsAlgs::Frechet::StrongFrechetGraphData::isPotentialEndpoint(std::size_t vert, std::size_t interval) const
{
    return interval == WhiteIntervals[vert].size() - 1 && WhiteIntervals[vert][interval].max > polyline.size() - 1 &&
        IntervalInclusions[vert].back().rightIncluded;
}

bool LoopsAlgs::Frechet::StrongFrechetGraphData::isPotentialStartPoint(std::size_t vert) const
{
    return !WhiteIntervals[vert].empty() && IntervalInclusions[vert][0].leftIncluded;
}

LoopsAlgs::Frechet::StrongFrechetGraphData::StrongFrechetGraphData()
{
}

void LoopsAlgs::Frechet::StrongFrechetGraphData::setup(const LoopsAlgs::Frechet::Graph* graph)
{
    m_graph = graph;
    FDiWhiteIntervals.clear();
    FDiWhiteIntervals.resize(graph->number_of_vertices(), std::vector<LoopsAlgs::Frechet::Interval>{});

    lrPointers.clear();
    lrPointers.resize(graph->number_of_vertices(), std::vector<PointerMap>{});
    for (int i = 0; i < graph->number_of_vertices(); ++i)
    {
        lrPointers[i].resize(polyline.size(), PointerMap{});
    }
    m_view.setGraph(graph);
}

void LoopsAlgs::Frechet::StrongFrechetGraphData::setup(const LoopsAlgs::Frechet::Graph* graph,
                                                  const std::vector<LoopsAlgs::Frechet::Point>& lineLocations)
{
    auto makSeg = movetk_core::MakeSegment<LoopsAlgs::Frechet::Kernel>();
    for (int i = 0; i < lineLocations.size() - 1; ++i)
    {
        polyline.push_back(makSeg(lineLocations[i], lineLocations[i + 1]));
    }
    setup(graph);
}

void LoopsAlgs::Frechet::StrongFrechetGraphData::setup(const LoopsAlgs::Frechet::Graph* graph,
                                                  const std::vector<LoopsAlgs::Frechet::Graph::Edge*>& lineEdges)
{
    auto makSeg = movetk_core::MakeSegment<LoopsAlgs::Frechet::Kernel>();
    for (auto* e : lineEdges)
    {
        polyline.push_back(makSeg(graph->locations()[e->m_source->id()], graph->locations()[e->m_sink->id()]));
    }
    setup(graph);
}
void LoopsAlgs::Frechet::StrongFrechetGraphData::setup(const LoopsAlgs::Frechet::Graph* graph,
    const std::vector<LoopsAlgs::Frechet::Graph::Id_t>& lineEdges)
{
    auto makSeg = movetk_core::MakeSegment<LoopsAlgs::Frechet::Kernel>();
    for (const auto& eId: lineEdges)
    {
        auto* e = graph->edge(eId);
        polyline.push_back(makSeg(graph->locations()[e->m_source->id()], graph->locations()[e->m_sink->id()]));
    }
    setup(graph);
}

double LoopsAlgs::Frechet::FrechetGraphComputations::fdiTimeMs() const
{
    return m_fdiTimeMs;
}

double LoopsAlgs::Frechet::FrechetGraphComputations::lrPointersTimeMs() const
{
    return m_lrPointersMs;
}

void LoopsAlgs::Frechet::FrechetGraphComputations::constructFrechetView(StrongFrechetGraphData& data) const
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

std::pair<LoopsAlgs::Frechet::Interval, LoopsAlgs::Frechet::LeftRightInclusion> LoopsAlgs::Frechet::FrechetGraphComputations::
computeIntersectionInterval(const Segment& seg, const Point& p, const NT& eps)
{
    LoopsLib::Math::Vec2<NT> v0{ seg[0].get().x(),seg[0].get().y() };
    LoopsLib::Math::Vec2<NT> v1{ seg[1].get().x(),seg[1].get().y() };
    LoopsLib::Math::Vec2<NT> pnt{ p.get().x(),p.get().y() };

    if((v1-v0).length() < 0.0001)
    {
        bool isWithinReach = (pnt - v0).length() < eps;
        return std::make_pair(
            isWithinReach ? Interval(0, 1) : Interval{},
            LeftRightInclusion{ isWithinReach, isWithinReach }
        );
    }

    IntervalPolynomial poly;
    poly.compute(v0, v1, pnt);

    return std::make_pair(
        poly.getInterval(eps),
        LeftRightInclusion{ poly.containsLeft(eps), poly.containsRight(eps) }
    );
}

std::pair<LoopsAlgs::Frechet::Interval, LoopsAlgs::Frechet::LeftRightInclusion> LoopsAlgs::Frechet::
FrechetGraphComputations::computeIntersectionInterval(const Point& s0, const Point& s1, const Point& p, const NT& eps)
{
    LoopsLib::Math::Vec2<NT> v0{ s0.get().x(),s0.get().y() };
    LoopsLib::Math::Vec2<NT> v1{ s1.get().x(),s1.get().y() };
    LoopsLib::Math::Vec2<NT> pnt{ p.get().x(),p.get().y() };

    IntervalPolynomial poly;
    poly.compute(v0, v1, pnt);

    return std::make_pair(
        poly.getInterval(eps),
        LeftRightInclusion{ poly.containsLeft(eps), poly.containsRight(eps) }
    );
}

void LoopsAlgs::Frechet::FrechetGraphComputations::computeFDi(StrongFrechetGraphData& data)
{
    // Query the point index
    if (data.m_view.number_of_vertices() == 0)
    {
        constructFrechetView(data);
    }
    data.m_epsilon = m_epsilon;

    LoopsLib::Helpers::ScopeTimer timer(&m_fdiTimeMs);
    auto* graphView = &data.m_view;
    data.FDiWhiteIntervals.resize(graphView->number_of_vertices(), {});
    data.IntervalInclusions.resize(graphView->number_of_vertices(), std::vector<LeftRightInclusion>{});
    data.CellToIntervalId.resize(graphView->number_of_vertices(), std::vector<int>{});
    data.IntervalIdToCell.resize(graphView->number_of_vertices(), {});
    data.WhiteIntervals.resize(graphView->number_of_vertices(), {});


    const auto polyEdgeCount = data.polylineEdgeCount();

    //Foreach vertex in the graph, compute the FD_i strip.
    for (int i = 0; i < graphView->number_of_vertices(); ++i)
    {
        auto* v = graphView->vertexByIndex(i);

        // Reserve data
        std::vector<Interval>& intervals = data.FDiWhiteIntervals.at(i);
        intervals.resize(polyEdgeCount, Interval{});
        std::vector<LeftRightInclusion>& lrInclusion = data.IntervalInclusions.at(i);
        lrInclusion.resize(polyEdgeCount, LeftRightInclusion{});

        // Cell to interval ID mapping
        auto& intervalIds = data.CellToIntervalId.at(i);
        intervalIds.resize(polyEdgeCount + 1, -1);

        // ID of the current interval being processed
        int currentInterval = -1;

        // Was the previous cell not empty
        bool prevNotEmpty = false;

        // Compute white interval per cell
        for (int j = 0; j < polyEdgeCount; ++j)
        {
            // Computes the interval in [0,1] that is white.
            const auto intervalResult = computeIntersectionInterval(data.polyline[j], data.m_graph->locations()[v->id()],
                                                              m_epsilon);
            // Save the results for the cell
            std::tie(intervals[j], lrInclusion[j]) = intervalResult;

            // Verify we don't create abominations
            if (std::isnan(intervals[j].min) || std::isnan(intervals[j].max))
            {
                std::cerr << "Encountered NaN in cell white intervals" << std::endl;
            }

            // Check if this is a new whiteinterval or connected to the previous interval.
            if(!intervals[j].isEmpty())
            {
                // Continuation of previous
                if(prevNotEmpty && lrInclusion[j - 1].rightIncluded)
                {
                    intervalIds[j] = currentInterval;
                }
                // New white interval
                else
                {
                    // Finish the previous white interval
                    if (prevNotEmpty)
                    {
                        data.WhiteIntervals[i].back().max = j - 1 + intervals[j - 1].max;
                    }
                    ++currentInterval;
                    intervalIds[j] = currentInterval;
                    data.IntervalIdToCell[i].push_back(j);
                    data.WhiteIntervals[i].push_back(Interval{ j + intervals[j].min, -1 });
                }
                prevNotEmpty = true;

                // Last cell white interval: set 
                if(j == polyEdgeCount -1)
                {
                    data.WhiteIntervals[i].back().max = j + intervals[j].max;
                }
            }
            else
            {
                if(prevNotEmpty)
                {
                    data.WhiteIntervals[i].back().max = j - 1 + intervals[j - 1].max;
                }
                prevNotEmpty = false;
            }
            // Verify range of computed interval
            assert(intervals[j].min >= 0 && intervals[j].max <= 1.00001);
        }
        // Add sentinel to end of intervalID map.
        if (lrInclusion[data.polyline.size() - 1].rightIncluded)
        {
            intervalIds[data.polyline.size()] = currentInterval;
        }
        assert(currentInterval + 1 == data.WhiteIntervals[i].size());
    }
}

void LoopsAlgs::Frechet::FrechetGraphComputations::computeLeftRightPointersBF(StrongFrechetGraphData& data)
{
    LoopsLib::Helpers::ScopeTimer timer(&m_lrPointersMs);
    using MakeSegment = typename movetk_core::MakeSegment<Kernel>;
    auto* graphView = &data.m_view;

    // Lrpointers per vertex per cell
    data.lrPointers.resize(graphView->number_of_vertices(), std::vector<PointerMap>{});
    for (int i = 0; i < graphView->number_of_vertices(); ++i)
    {
        data.lrPointers[i].resize(data.polyline.size(), PointerMap{});
    }

    // Foreach interval in an FD_i strip, compute the l- and r-pointer per adjacent vertex.
    for (int i = 0; i < graphView->number_of_vertices(); ++i)
    {
        // Active vertex in original graph
        auto* vert = graphView->vertexByIndex(i);

        // Number of segments of the polyline
        const auto polysize = data.FDiWhiteIntervals[i].size();

        const auto v0 = data.m_graph->locations()[vert->id()];

        // Loop over outgoing edges
        for (auto* e : graphView->outEdges(vert->id()))
        {
            // ID of the target vertex, in the graph view!!
            const auto targetVertId = graphView->indexForVertex(e->m_sink->id());
            const auto v1 = data.m_graph->vertexLocation(e->m_sink->id());

            // Precompute between intervals
            std::vector<Interval> between;
            for (std::size_t i = 0; i < data.polylineEdgeCount(); ++i)
            {
                auto res = LoopsAlgs::Frechet::FrechetGraphComputations::computeIntersectionInterval(v0, v1, data.polyline[i].get().vertex(0), m_epsilon);
                between.push_back(res.first);
            }
            
            // For every cell, start at vert, going over e.
            for (int k = 0; k < polysize; ++k)
            {
                // No white interval at the source vertex, so continue.
                if (data.FDiWhiteIntervals[i][k].isEmpty()) continue;

                NT lowest = 0;
                // Cell 
                int leftMax = -1;
                // Data for later
                bool leftIncluded = false;
                NT leftCoord = data.FDiWhiteIntervals[i][k].min + k;
                for (int kPrime = k; kPrime < polysize; ++kPrime)
                {
                    // Non-empty interval that is also reachable from vertex i.
                    if (!data.FDiWhiteIntervals[targetVertId][kPrime].isEmpty() && data.FDiWhiteIntervals[targetVertId][kPrime].max + kPrime > leftCoord)
                    {
                        // Left coord may be cut off by start interval in case they are in the same cell.
                        if(data.FDiWhiteIntervals[targetVertId][kPrime].min + kPrime > leftCoord)
                        {
                            leftCoord = data.FDiWhiteIntervals[targetVertId][kPrime].min + kPrime;
                            leftIncluded = data.IntervalInclusions[targetVertId][kPrime].leftIncluded;
                        }
                        else
                        {
                            leftCoord = data.FDiWhiteIntervals[i][k].min + k;
                            leftIncluded = data.IntervalInclusions[i][k].leftIncluded;
                        }
                        leftMax = kPrime;
                        break;
                    }
                    if (kPrime >= polysize - 1) break;
                    // Compute intersection interval at left boundary of next cell
                    auto inter = between[kPrime+1];
                    // Can we reach the next cell with a monotone path?
                    if (inter.isEmpty() || inter.max < lowest) break;
                    // Propagate lowest monotone path
                    lowest = std::max(inter.min, lowest);
                }
                // No interval found
                if (leftMax < 0) continue;

                // Lr pointer should be non-empty. So find right maximum
                int rightMax = leftMax;

                // Find other end
                for (int kPrime = leftMax; kPrime < polysize; ++kPrime)
                {
                    // Non-empty cell: update the right maximum reachable free cell
                    if (!data.FDiWhiteIntervals[targetVertId][kPrime].isEmpty()) rightMax = kPrime;

                    if (kPrime >= polysize - 1) break;
                    auto inter = between[kPrime+1];
                    if (inter.isEmpty() || lowest > inter.max) break;
                    lowest = std::max(inter.min, lowest);
                }
                Interval reachable(leftCoord,
                    rightMax + data.FDiWhiteIntervals[targetVertId][rightMax].max);

                data.lrPointers[i][k][targetVertId] = LeftRightPointers{
                    reachable,
                    LeftRightInclusion{
                        leftIncluded,
                        data.IntervalInclusions[targetVertId][rightMax].rightIncluded
                    },
                    data.CellToIntervalId[targetVertId][leftMax],
                    data.CellToIntervalId[targetVertId][rightMax]
                };
            }
        }
    }
}
