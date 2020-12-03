#ifndef GEOMETRY_INDEX_POINTINDEX_H
#define GEOMETRY_INDEX_POINTINDEX_H

#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Geometry/BoostInterface.h>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
namespace LoopsLib::Geometry::Index
{
    using NT = LoopsLib::NT;
    namespace bgi = boost::geometry::index;
    struct PointIndex
    {
        using Point = MovetkGeometryKernel::MovetkPoint;
        using value = std::pair<Point, long long>;
        using rtree = bgi::rtree<value, bgi::quadratic<4>>;

        rtree m_tree;
        void construct(const std::vector<MovetkGeometryKernel::MovetkPoint>& points);

        void clear();

        bool isEmpty() const;

        void containedIn(NT minX, NT minY, NT maxX, NT maxY, std::vector<long long>& out) const;
        void containedInDisk(NT x, NT y, NT radius, std::vector<long long>& out) const;

        /**
         * \brief Determines the points that lie within the buffer zone of the polyline, with the given buffersize.
         * The bufferzone is given by the Minkowski sum of a disk with radius 'bufferSize' and the polyline
         * \param polyline The polyline, given as a list of consecutive points
         * \param bufferSize The radius of the disk for determining the buffer zone
         * \param points Points that lie within the zone
         */
        void containedInBuffer(const std::vector<Point>& polyline, NT bufferSize, std::vector<long long>& points) const;

        void containedInBuffer(const std::vector<Point>& polyline, NT bufferSize, std::set<long long>& points) const;


        void containedInPoly(const std::vector<Point>& points, std::vector<long long>& out) const;

        /**
         * \brief 
         * \param points Polygon, specified in counterclockwise fashion
         * \param out 
         */
        void containedInPoly(const std::vector<std::pair<NT, NT>>& points, std::vector<long long>& out) const;

        void containedInOrientedRect(const std::pair<NT,NT>& start, const std::pair<NT,NT>& end, NT width, std::vector<long long>& out) const;

        long long closest(const Point& point) const;
    };
}
#endif