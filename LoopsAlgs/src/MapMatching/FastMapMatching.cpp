#include <LoopsAlgs/MapMatching/FastMapMatching.h>

LoopsAlgs::MapMatching::FastMapMatching::MemoizedEdgeLength::
MemoizedEdgeLength(LoopsLib::DS::EmbeddedGraph* graph): m_graph(graph)
{
}

LoopsLib::NT LoopsAlgs::MapMatching::FastMapMatching::MemoizedEdgeLength::operator[](
    LoopsLib::DS::EmbeddedGraph::EdgeIndex edge) const
{
    if (lengths.find(edge) == lengths.end())
    {
        lengths[edge] = m_graph->edgeLength(edge);
    }
    return lengths.at(edge);
}

LoopsAlgs::MapMatching::FastMapMatching::TransmissionProbProvider::TransmissionProbProvider(
    const std::vector<std::vector<Candidate>>& candidates, Graph_t* graph,
    const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& trajectory, NT speedBound):
    m_candidates(candidates),
    m_graph(graph),
    m_edgeLengths(graph),
    m_trajectory(trajectory),
    m_speedBound(speedBound)
{
}

void LoopsAlgs::MapMatching::FastMapMatching::TransmissionProbProvider::setVerbose(bool value)
{
    m_verbose = value;
}

std::pair<bool, LoopsLib::NT> LoopsAlgs::MapMatching::FastMapMatching::TransmissionProbProvider::operator()(
    std::size_t startLayer, std::size_t candidate, std::size_t endLayer, std::size_t endCandidate)
{
    assert(endLayer - startLayer == 1);
    // Determine the shortest path between the candidates and the Euclidean distance between their respective positions.
    LoopsLib::GraphAlgs::WeightedShortestPath<MemoizedEdgeLength> wsp;
    const auto& startC = m_candidates[startLayer][candidate];
    const auto& endC = m_candidates[endLayer][endCandidate];
    NT shortestPathDist = 0;
    if (startC.edge == endC.edge && startC.offsetFromStart <= endC.offsetFromStart)
    {
        shortestPathDist = endC.offsetFromStart - startC.offsetFromStart;
    }
    else
    {
        // Compute upperbound for shortest path, based on speedbound
        const auto distUpperbound = m_speedBound * (m_trajectory[endLayer].second - m_trajectory[startLayer].second);

        std::vector<Graph_t::Edge*> pointerPath;
        // Check if there is a shortest path within the upperbound
        if(!wsp.computeShortestPath(m_graph, m_edgeLengths,
                                m_graph->edge(startC.edge)->m_sink->id(),
                                m_graph->edge(endC.edge)->m_source->id(), pointerPath, distUpperbound))
        {
            if(m_verbose) std::cout << "Could not find SP with upperbound " << distUpperbound << ": start " << startC.edge << "," << startC.offsetFromStart << " | end "
                << endC.edge << "," << endC.offsetFromStart << std::endl;
            return std::make_pair(false, 0);
        }

        // Compute shortest dist as path plus the distances along the matched edges
        shortestPathDist = endC.offsetFromStart + m_edgeLengths[startC.edge] - startC.offsetFromStart + wsp.
            lastSpDist();
    }
    // Compute probability as ratio of euclidean to shortest path dist (or the inverse if shortestpaths is smaller, which is unlikely, but possible)
    const NT euclideanDist = (m_trajectory[startLayer].first - m_trajectory[endLayer].first).length();
    return std::make_pair(true, euclideanDist >= shortestPathDist
                                    ? (shortestPathDist + 1e-12) / (euclideanDist + 1e-12)
                                    : euclideanDist / shortestPathDist);
}

LoopsAlgs::MapMatching::FastMapMatching::Vec2 LoopsAlgs::MapMatching::FastMapMatching::nearestOnSegment(
    const Vec2& queryPoint, const Vec2& s0, const Vec2& s1)
{
    auto diff = s1 - s0;
    if (diff.dot(queryPoint - s0) < 0) return s0;
    if (diff.dot(queryPoint - s1) > 0) return s1;
    return queryPoint.projectOn(s0, s1);
}

LoopsAlgs::MapMatching::FastMapMatching::Vec2 LoopsAlgs::MapMatching::FastMapMatching::nearestOnEdge(
    const Vec2& queryPoint, const LoopsLib::DS::EmbeddedGraph::EdgeIndex& e, LoopsLib::NT& offset) const
{
    auto pos = nearestOnSegment(queryPoint, m_graph->vertexLocation(m_graph->edge(e)->m_source),
                            m_graph->vertexLocation(m_graph->edge(e)->m_sink));
    offset = (pos - m_graph->vertexLocation(m_graph->edge(e)->m_source)).length();
    return pos;
}

void LoopsAlgs::MapMatching::FastMapMatching::reconstructPath(const std::vector<Candidate>& candidatePath,
    MapMatchedPath& out)
const
{
    MemoizedEdgeLength edgeLengths(m_graph);
    for (std::size_t i = 0; i < candidatePath.size() - 1; ++i)
    {
        const auto& curr = candidatePath[i];
        const auto& next = candidatePath[i + 1];
        if (curr.edge == next.edge)
        {
            // On the same edge in increasing order
            if (curr.offsetFromStart <= next.offsetFromStart)
            {
                if (out.empty() || out.back() != curr.edge) out.push_back(curr.edge);
            }
                // In reverse order, so need to find a path
            else
            {
                LoopsLib::GraphAlgs::WeightedShortestPath<MemoizedEdgeLength> wsp;
                std::vector<LoopsLib::DS::EmbeddedGraph::Edge*> pointerPath;
                std::vector<LoopsLib::DS::EmbeddedGraph::EdgeIndex> path;
                wsp.computeShortestPath(m_graph, edgeLengths, m_graph->edge(curr.edge)->m_sink->id(),
                                        m_graph->edge(next.edge)->m_source->id(), pointerPath);
                std::transform(pointerPath.begin(), pointerPath.end(), std::back_inserter(path),
                               [](auto* e) { return e->id(); });

                if (out.empty() || out.back() != curr.edge)
                {
                    if (path.front() != curr.edge) out.push_back(curr.edge);
                }
                out.insert(out.end(), path.begin(), path.end());
            }
        }
        else
        {
            std::vector<LoopsLib::DS::EmbeddedGraph::EdgeIndex> path;

            if(m_graph->edge(curr.edge)->m_sink->id() == m_graph->edge(next.edge)->m_source->id())
            {
                path = { (std::size_t)m_graph->edge(next.edge)->id() };
            }
            else
            {

                LoopsLib::GraphAlgs::WeightedShortestPath<MemoizedEdgeLength> wsp;
                std::vector<LoopsLib::DS::EmbeddedGraph::Edge*> pointerPath;
                wsp.computeShortestPath(m_graph, edgeLengths, m_graph->edge(curr.edge)->m_sink->id(),
                    m_graph->edge(next.edge)->m_source->id(), pointerPath);
                std::transform(pointerPath.begin(), pointerPath.end(), std::back_inserter(path),
                    [](auto* e) { return e->id(); });
            }

            if (out.empty() || out.back() != curr.edge)
            {
                if (path.front() != curr.edge) out.push_back(curr.edge);
            }
            out.insert(out.end(), path.begin(), path.end());
        }
    }
    if (out.empty() || candidatePath.back().edge != out.back()) out.push_back(candidatePath.back().edge);
}

LoopsAlgs::MapMatching::FastMapMatching::FastMapMatching(LoopsLib::DS::EmbeddedGraph* graph): m_graph(graph)
{
    m_index.construct(m_graph);
}

LoopsAlgs::MapMatching::FastMapMatching::FastMapMatching(LoopsLib::DS::EmbeddedGraph* graph, LoopsLib::NT radius,
                                                         LoopsLib::NT gpsError, int k): m_graph(graph),
                                                                                        m_radius(radius),
                                                                                        m_gpsError(gpsError),
                                                                                        m_k(k)
{
    m_index.construct(m_graph);
}

void LoopsAlgs::MapMatching::FastMapMatching::mapMatch(const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& trajectory, MapMatchedPath& out)
{
    EmissionProbProvider epProvider(m_gpsError, trajectory);

    std::vector<std::vector<Candidate>> candidates;
    for (const auto& measurement : trajectory)
    {
        //std::cout << "[FastMapMatching] ----------" << std::endl;
        auto nearest = m_index.kNearest(measurement.first.x(), measurement.first.y(), m_k, m_radius);
        candidates.emplace_back();
        auto& currCandidates = candidates.back();
        for (const auto& el : nearest)
        {
            LoopsLib::NT offset = 0;
            auto pos = nearestOnEdge(measurement.first, el, offset);
            currCandidates.push_back(Candidate{el, pos, offset});
            //std::cout << "[FastMapMatching] Candidate " << (measurement.first-pos).length() << ',' << "(" << pos.x() << "," << pos.y() << "),"<< el << std::endl;
        }
        // Early out
        /*if(currCandidates.empty())
        {
            std::cout << "[FastMapMatching] Empty candidate layer, searching with location " << measurement.first.x() << "," << measurement.first.y() << std::endl;
            auto nearest2 = m_index.kNearest(measurement.first.x(), measurement.first.y(), 1, 10*m_radius);
            if(nearest2.size()>0)
            {
                LoopsLib::NT offset = 0;
                auto pos = nearestOnEdge(measurement.first, nearest2[0], offset);
                std::cout << "\t\t Approx closest: " << (pos - measurement.first).length() << std::endl;
            }
            
            return;
        }*/
    }
    TransmissionProbProvider tpProvider(candidates, m_graph, trajectory, m_speedBound);

    // Compute all probabilities.
    m_hmmGraph.compute(epProvider, tpProvider, candidates.begin(), candidates.end(), [](decltype(candidates.begin()) it)
    {
        return std::make_pair(it->begin(), it->end());
    });
    // Reconstruct candidates path
    std::string failMessage;
    std::size_t startLayer;
    auto path = m_hmmGraph.reconstructPath(failMessage, startLayer);
    if (path.empty())
    {
        std::cout << "FastMapMatching failed: " << failMessage << std::endl;
        return;
    }
    // Reconstruct path in the roadnetwork via shortest paths.
    std::vector<Candidate> optimalPath;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        optimalPath.push_back(candidates[startLayer+i][path[i]]);
        if (i != 0) std::cout << ",";
        std::cout << candidates[startLayer + i][path[i]].edge;
    }
    std::cout << std::endl;
    std::cout << "[FastMapMatching] Matched " << path.size() << "/" << candidates.size() << std::endl;
    reconstructPath(optimalPath, out);
}

void LoopsAlgs::MapMatching::FastMapMatching::mapMatch(const Kernel::TimestampedTrajectory& trajectory,
    MapMatchedPath& out, int& offset, int& size)
{
    EmissionProbProvider epProvider(m_gpsError, trajectory);

    std::vector<std::vector<Candidate>> candidates;
    for (const auto& measurement : trajectory)
    {
        //std::cout << "[FastMapMatching] ----------" << std::endl;
        auto nearest = m_index.kNearest(measurement.first.x(), measurement.first.y(), m_k, m_radius);
        candidates.emplace_back();
        auto& currCandidates = candidates.back();
        for (const auto& el : nearest)
        {
            LoopsLib::NT candidateOffset = 0;
            auto pos = nearestOnEdge(measurement.first, el, candidateOffset);
            currCandidates.push_back(Candidate{ el, pos, candidateOffset });
            //std::cout << "[FastMapMatching] Candidate " << (measurement.first-pos).length() << ',' << "(" << pos.x() << "," << pos.y() << "),"<< el << std::endl;
        }
        if(m_verbose && candidates.empty())std::cout << "[FastMapMatching] Empty candidate layer, searching with location " << measurement.first.x() << "," << measurement.first.y() << std::endl;
        // Early out
        /*if(currCandidates.empty())
        {
            std::cout << "[FastMapMatching] Empty candidate layer, searching with location " << measurement.first.x() << "," << measurement.first.y() << std::endl;
            auto nearest2 = m_index.kNearest(measurement.first.x(), measurement.first.y(), 1, 10*m_radius);
            if(nearest2.size()>0)
            {
                LoopsLib::NT offset = 0;
                auto pos = nearestOnEdge(measurement.first, nearest2[0], offset);
                std::cout << "\t\t Approx closest: " << (pos - measurement.first).length() << std::endl;
            }

            return;
        }*/
    }
    TransmissionProbProvider tpProvider(candidates, m_graph, trajectory, m_speedBound);

    // Compute all probabilities.
    m_hmmGraph.compute(epProvider, tpProvider, candidates.begin(), candidates.end(), [](decltype(candidates.begin()) it)
    {
        return std::make_pair(it->begin(), it->end());
    });
    // Reconstruct candidates path
    std::string failMessage;
    std::size_t startLayer;
    auto path = m_hmmGraph.reconstructPath(failMessage, startLayer);
    if (path.empty())
    {
        std::cout << "FastMapMatching failed: " << failMessage << std::endl;
        return;
    }
    offset = startLayer;
    size = path.size();
    // Reconstruct path in the roadnetwork via shortest paths.
    std::vector<Candidate> optimalPath;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        optimalPath.push_back(candidates[startLayer + i][path[i]]);
        if (i != 0) std::cout << ",";
        std::cout << candidates[startLayer + i][path[i]].edge;
    }
    std::cout << std::endl;
    std::cout << "[FastMapMatching] Matched " << path.size() << "/" << candidates.size() << std::endl;
    reconstructPath(optimalPath, out);
}

void LoopsAlgs::MapMatching::FastMapMatching::mapMatchFromWGS84(const Kernel::TimestampedTrajectory& trajectory,
    MapMatchedPath& out, int& offset, int& size)
{
    OGRSpatialReference ref;
    ref.SetWellKnownGeogCS("WGS84");
    mapMatch(trajectory, ref, out, offset, size);
}

void LoopsAlgs::MapMatching::FastMapMatching::mapMatch(const Kernel::TimestampedTrajectory& trajectory,
    const OGRSpatialReference& trajectoryRef, MapMatchedPath& out)
{
    if(trajectoryRef.IsSame(m_graph->spatialRefPntr()))
    {
        mapMatch(trajectory, out);
    }
    else
    {
        std::cout << "[FastMapMatching] Converting trajectory to correct spatial reference" << std::endl;
        Kernel::TimestampedTrajectory converted = trajectory;

        LoopsLib::MovetkGeometryKernel().convertCRS(trajectoryRef, m_graph->spatialRef(), converted);
        mapMatch(converted, out);
    }
}

void LoopsAlgs::MapMatching::FastMapMatching::mapMatch(const Kernel::TimestampedTrajectory& trajectory,
    const OGRSpatialReference& trajectoryRef, MapMatchedPath& out, int& offset, int& size)
{
    if (trajectoryRef.IsSame(m_graph->spatialRefPntr()))
    {
        mapMatch(trajectory, out, offset, size);
    }
    else
    {
        std::cout << "[FastMapMatching] Converting trajectory to correct spatial reference" << std::endl;
        Kernel::TimestampedTrajectory converted = trajectory;

        LoopsLib::MovetkGeometryKernel().convertCRS(trajectoryRef, m_graph->spatialRef(), converted);
        mapMatch(converted, out, offset, size);
    }
}

void LoopsAlgs::MapMatching::FastMapMatching::mapMatchSet(const Kernel::TimestampedTrajectorySet& trajectorySet,
    Kernel::GraphTrajectorySet& out, int count)
{
    if(trajectorySet.m_ref.IsSame(m_graph->spatialRefPntr()))
    {
        std::size_t index = 0;
        std::size_t success = 0;
        for (; index < (count == -1 ? trajectorySet.size() : std::min(count, (int)trajectorySet.size())); ++index)
        {
            const auto& trajectory = trajectorySet.trajectories[index];
            Kernel::GraphTrajectory result;
            mapMatch(trajectory, result);
            std::cout << "[FastMatching] -- Matching " << trajectorySet.ids[index] << std::endl;
            if (!result.empty())
            {
                out.trajectories.emplace_back(std::move(result));
                out.ids.push_back(trajectorySet.ids[index]);
                ++success;
            }
            std::cout << "[FastMatching] Matched trajectories so far " << success << "/" << (index +1)<< std::endl;
        }
    }
    else
    {
        auto deleter = [](auto* el) {if (el)OGRCoordinateTransformation::DestroyCT(el); };
        std::unique_ptr<OGRCoordinateTransformation, decltype(deleter)> transform(nullptr, deleter);
        transform.reset(OGRCreateCoordinateTransformation(const_cast<OGRSpatialReference*>(&trajectorySet.m_ref), const_cast<OGRSpatialReference*>(m_graph->spatialRefPntr())));
        std::size_t index = 0;

        std::size_t success = 0;
        for (; index < (count == -1 ? trajectorySet.size() : std::min(count,(int)trajectorySet.size())); ++index)
        {
            const auto& trajectory = trajectorySet.trajectories[index];
            std::cout << "[FastMapMatching] Converting trajectory to correct CRS" << std::endl;
            Kernel::TimestampedTrajectory converted;
            movetk_core::MakePoint<Kernel> mkPoint;
            for (const auto& el : trajectory)
            {
                double x, y;
                x = el.first.x();
                y = el.first.y();
                transform->Transform(1, &x, &y);

                converted.push_back(std::make_pair(mkPoint({ x,y }), el.second));
            }
            std::cout << "[FastMatching] -- Matching " << trajectorySet.ids[index] << std::endl;
            Kernel::GraphTrajectory result;
            mapMatch(converted, result);
            if(!result.empty())
            {
                out.trajectories.emplace_back(std::move(result));
                out.ids.push_back(trajectorySet.ids[index]);
                ++success;
            }
            std::cout << "[FastMatching] Matched trajectories so far " << success << "/" << (index+1) << std::endl;
        }
    }
}

void LoopsAlgs::MapMatching::FastMapMatching::mapMatchSet(const Kernel::TimestampedTrajectorySet& trajectorySet,
    Kernel::GraphTrajectorySet& out, SetMatchResultDescription& resultDescription, int count)
{
    if (trajectorySet.m_ref.IsSame(m_graph->spatialRefPntr()))
    {
        std::size_t index = 0;
        std::size_t success = 0;
        for (; index < (count == -1 ? trajectorySet.size() : std::min(count, (int)trajectorySet.size())); ++index)
        {
            const auto& trajectory = trajectorySet.trajectories[index];
            Kernel::GraphTrajectory result;
            int offset, sz;
            mapMatch(trajectory, result,offset, sz);
            std::cout << "[FastMatching] -- Matching " << trajectorySet.ids[index] << std::endl;
            if (!result.empty())
            {
                out.trajectories.emplace_back(std::move(result));
                out.ids.push_back(trajectorySet.ids[index]);
                ++success;
                resultDescription.matchedTrajectories.push_back(out.ids.back());
                resultDescription.matchOffset.push_back(offset);
                resultDescription.matchSize.push_back(sz);
                resultDescription.inputSize.push_back(trajectory.size());
            }
            std::cout << "[FastMatching] Matched trajectories so far " << success << "/" << (index + 1) << std::endl;
        }
    }
    else
    {
        auto deleter = [](auto* el) {if (el)OGRCoordinateTransformation::DestroyCT(el); };
        std::unique_ptr<OGRCoordinateTransformation, decltype(deleter)> transform(nullptr, deleter);
        transform.reset(OGRCreateCoordinateTransformation(const_cast<OGRSpatialReference*>(&trajectorySet.m_ref), const_cast<OGRSpatialReference*>(m_graph->spatialRefPntr())));
        std::size_t index = 0;

        std::size_t success = 0;
        for (; index < (count == -1 ? trajectorySet.size() : std::min(count, (int)trajectorySet.size())); ++index)
        {
            const auto& trajectory = trajectorySet.trajectories[index];
            std::cout << "[FastMapMatching] Converting trajectory to correct CRS" << std::endl;
            Kernel::TimestampedTrajectory converted;
            movetk_core::MakePoint<Kernel> mkPoint;
            for (const auto& el : trajectory)
            {
                double x, y;
                x = el.first.x();
                y = el.first.y();
                transform->Transform(1, &x, &y);

                converted.push_back(std::make_pair(mkPoint({ x,y }), el.second));
            }
            std::cout << "[FastMatching] -- Matching " << trajectorySet.ids[index] << std::endl;
            Kernel::GraphTrajectory result;
            int offset, sz;
            mapMatch(converted, result, offset,sz);
            if (!result.empty())
            {
                out.trajectories.emplace_back(std::move(result));
                out.ids.push_back(trajectorySet.ids[index]);
                resultDescription.matchedTrajectories.push_back(out.ids.back());
                resultDescription.matchOffset.push_back(offset);
                resultDescription.matchSize.push_back(sz);
                resultDescription.inputSize.push_back(trajectory.size());
                ++success;
            }
            std::cout << "[FastMatching] Matched trajectories so far " << success << "/" << (index + 1) << std::endl;
        }
    }
}

void LoopsAlgs::MapMatching::FastMapMatching::setRadius(const LoopsLib::NT& radius)
{
    m_radius = radius;
}

LoopsLib::NT LoopsAlgs::MapMatching::FastMapMatching::radius() const
{
    return m_radius;
}

void LoopsAlgs::MapMatching::FastMapMatching::setK(int k)
{
    m_k = k;
}

void LoopsAlgs::MapMatching::FastMapMatching::setSpeedBound(const LoopsLib::NT& speedBound)
{
    m_speedBound = speedBound;
}

LoopsLib::NT LoopsAlgs::MapMatching::FastMapMatching::speedBound() const
{
    return m_speedBound;
}

void LoopsAlgs::MapMatching::FastMapMatching::setVerbose(const bool& verbose)
{
    m_verbose = verbose;
    m_hmmGraph.setVerbose(verbose);
}

bool LoopsAlgs::MapMatching::FastMapMatching::verbose() const
{
    return m_verbose;
}

int LoopsAlgs::MapMatching::FastMapMatching::k() const
{
    return m_k;
}

void LoopsAlgs::MapMatching::FastMapMatching::setGpsError(LoopsLib::NT error)
{
    m_gpsError = error;
}

LoopsLib::NT LoopsAlgs::MapMatching::FastMapMatching::gpsError() const
{
    return m_gpsError;
}
