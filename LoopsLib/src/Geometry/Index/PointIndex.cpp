#include <LoopsLib/Geometry/Index/PointIndex.h>
#include <LoopsLib/Geometry/BoostInterface.h>
#include "movetk/geom/GeometryInterface.h"
#include <LoopsLib/Helpers/Iterators.h>
using namespace LoopsLib::Geometry::Index;

void PointIndex::construct(const std::vector<LoopsLib::MovetkGeometryKernel::MovetkPoint>& points)
{
    long long id = 0;
    for (auto p : points)
    {
        m_tree.insert(std::make_pair(p, id));
        ++id;
    }
}

void PointIndex::clear()
{
    m_tree.clear();
}

bool PointIndex::isEmpty() const
{
    return m_tree.empty();
}

void PointIndex::containedIn(NT minX, NT minY, NT maxX, NT maxY, std::vector<long long>& out) const
{
    auto mkPoint = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();
    auto bl = mkPoint({ minX,minY });
    auto tr = mkPoint({ maxX,maxY });
    boost::geometry::model::box<specialize::MtkPoint> box(bl, tr);
    std::vector<value> vals;
    m_tree.query(bgi::covered_by(box), std::back_inserter(vals));
    out.reserve(out.size() + vals.size());
    for (auto val : vals)
    {
        out.push_back(val.second);
    }
}

void PointIndex::containedInDisk(NT x, NT y, NT radius, std::vector<long long>& out) const
{
    auto mkPoint = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();
    auto bl = mkPoint({ x-radius,y-radius});
    auto tr = mkPoint({ x + radius,y + radius });
    boost::geometry::model::box<specialize::MtkPoint> box(bl, tr);
    std::vector<value> vals;
    m_tree.query(bgi::covered_by(box), std::back_inserter(vals));
    out.reserve(out.size() + vals.size());
    for (auto val : vals)
    {
        if(std::hypot(val.first.m_x-x, val.first.m_y-y) <= radius)
        {
            out.push_back(val.second);
        }
    }
}

void PointIndex::containedInBuffer(const std::vector<Point>& polyline, NT bufferSize, std::vector<long long>& points) const
{
    if (polyline.size() <= 1) return;
    std::set<long long> pointsSet;
    containedInBuffer(polyline, bufferSize, pointsSet);
    points.reserve(pointsSet.size());
    points.insert(points.begin(), pointsSet.begin(), pointsSet.end());
}

void PointIndex::containedInBuffer(const std::vector<Point>& polyline, NT bufferSize, std::set<long long>& points) const
{
    if (polyline.size() <= 1) return;
    std::size_t ind = 0;
    for (; ind < polyline.size() - 1; ++ind)
    {
        std::vector<long long> temp;
        const auto& p0 = polyline[ind];
        const auto& p1 = polyline[ind+1];
        auto x = p0.x();
        auto y = p0.y();
        auto x2 = p1.x();
        auto y2 = p1.y();
        this->containedInDisk(x, y, bufferSize, temp);
        if (ind == polyline.size() - 2)
        {
            containedInDisk(x2, y2, bufferSize, temp);
        }
        auto len = std::hypot((x2 - x), (y2 - y));
        // 90 degrees rotated in CCW direction
        auto rotX = -(y2 - y);
        auto rotY = x2 - x;
        const auto scale = bufferSize / len;

        auto pnt = [](NT x, NT y) { return std::make_pair(x, y); };
        //
        containedInPoly({
            pnt(x + rotX * scale, y + rotY * scale),
            pnt(x - rotX * scale, y - rotY * scale),
            pnt(x2 - rotX * scale, y2 - rotY * scale),
            pnt(x2 + rotX * scale, y2 + rotY * scale),
            }, temp);
        points.insert(temp.begin(), temp.end());
    }
}

void PointIndex::containedInPoly(const std::vector<LoopsLib::MovetkGeometryKernel::MovetkPoint>& points, std::vector<long long>& out) const
{
    boost::geometry::model::polygon<specialize::MtkPoint, false, false> poly;
    poly.outer().assign(points.begin(), points.end());
    std::vector<value> vals;
    m_tree.query(bgi::covered_by(poly), std::back_inserter(vals));
    out.reserve(out.size() + vals.size());
    for (auto val : vals)
    {
        out.push_back(val.second);
    }
}

void PointIndex::containedInPoly(const std::vector<std::pair<LoopsLib::NT, LoopsLib::NT>>& points, std::vector<long long>& out) const
{
    auto mkpoint = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();

    boost::geometry::model::polygon<specialize::MtkPoint,false,false> poly;
    auto it = Helpers::Iterators::transform_iterator(points, [mkpoint](const std::pair<LoopsLib::NT, LoopsLib::NT>& pair) {return mkpoint({ pair.first, pair.second }); });
    for(const auto& el: points)
    {
        poly.outer().push_back(mkpoint({ el.first, el.second }));
    }
    
    //poly.outer().assign(it, it.associatedEnd());
    std::vector<value> vals;
    m_tree.query(bgi::covered_by(poly), std::back_inserter(vals));
    out.reserve(out.size() + vals.size());
    for (auto val : vals)
    {
        out.push_back(val.second);
    }
}

void PointIndex::containedInOrientedRect(const std::pair<NT, NT>& start, const std::pair<NT, NT>& end, NT width,
    std::vector<long long>& out) const
{
    auto dx = end.first - start.first;
    auto dy = end.second - start.second;
    auto len = std::hypot(dx, dy);
    auto rotX = -dy / len;
    auto rotY = dx / len;
    auto pnt = [](NT x, NT y) {return std::make_pair(x, y); };
    auto sumPnts = [](std::pair<NT,NT> p0, std::pair<NT,NT> p1, NT scale=1)
    {
        return std::make_pair(p0.first + scale * p1.first, p0.second + scale*p1.second);
    };
    auto rotDir = pnt(rotX, rotY);

    containedInPoly({
        sumPnts(start, rotDir, width),
        sumPnts(start, rotDir, -width),
        sumPnts(end, rotDir, -width),
        sumPnts(end, rotDir, width)
    }, out);
}

long long PointIndex::closest(const Point& point) const
{
    if (m_tree.empty()) return -1;

    std::vector<value> result;
    m_tree.query(bgi::nearest(point, 1), std::back_inserter(result));
    assert(result.size() != 0);
    return result[0].second;
}
