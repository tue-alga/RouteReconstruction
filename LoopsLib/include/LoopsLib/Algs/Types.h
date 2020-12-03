#ifndef LOOPS_TYPES_H
#define LOOPS_TYPES_H

#include <LoopsLib/DS/BaseGraph.h>
//#include <CGAL/Simple_cartesian.h>
#include <LoopsLib/Math/Vector.h>
#include <ogr_spatialref.h>
#include <memory>
namespace LoopsLib
{

    //// Default graph type. 
    //using GraphType = DS::BaseGraph;
    //// The scalar value to use.
    //using Scalar = double;
    //// Flow map
    //using FlowMap = std::vector<Scalar>;
    //using Kernel = CGAL::Simple_cartesian<double>;
    //using Point2 = Kernel::Point_2;

    template<typename T>
    bool isApprox(T t1, T t2, T thresh = 0.000001)
    {
        return std::abs(t1 - t2) < thresh;
    }

    namespace KernelDef
    {
        //==============================
        // Define the Number Type
        // For the CGAL backend,
        // One can choose from the
        // following number types
        using NT = long double;
        //typedef CGAL::Mpzf NT;
        //typedef CGAL::Gmpfr NT;
        //typedef CGAL::Gmpq NT;
        //==============================

        //==============================
        // Define the Dimensions
        constexpr size_t dimensions = 2;
        //==============================

    }

    //==============================
    // Define the Geometry Backend
    //    struct GeometryBackend
    //{
    //        using NT = KernelDef::NT;
    //        constexpr static size_t dim = KernelDef::dimensions;
    //        //using GeometryType = CGAL::Cartesian_d<NT> ; // define the geometry type
    //        using GeometryType = CGAL::Simple_cartesian<NT>; // define the geometry type
    //        using SphereTraits = CGAL::Min_sphere_of_points_d_traits_d<GeometryType, NT, KernelDef::dimensions>;
    //        using Wrapper_CGAL_Geometry = movetk_support::Wrapper_CGAL_Kernel<GeometryBackend> ; // the traits class
    //};
    //using GeometryBackend = movetk_support::CGALTraits<KernelDef::NT, KernelDef::dimensions>;
    //typedef movetk_support::CGALTraits<KernelDef::NT, KernelDef::dimensions> GeometryBackend;
    //Using the Geometry Backend define the Movetk Geometry Kernel
    //using MovetkGeometryKernel = movetk_core::MovetkGeometryKernel<typename GeometryBackend::Wrapper_CGAL_Geometry>;
    //typedef movetk_core::MovetkGeometryKernel<typename GeometryBackend::Wrapper_CGAL_Geometry> MovetkGeometryKernel;

    using NT = typename KernelDef::NT;
    struct MovetkGeometryKernel
    {
        using MovetkPoint = LoopsLib::Math::Vec2<NT>;
        using MovetkVector = LoopsLib::Math::Vec2<NT>;

        struct Segment
        {
            MovetkPoint m_p0, m_p1;
            Segment(const MovetkPoint& p0, const MovetkPoint& p1) :m_p0(p0), m_p1(p1) {}
            const MovetkPoint& operator[](std::size_t ind) const
            {
                return ind == 0 ? m_p0 : m_p1;
            }
            MovetkPoint& operator[](std::size_t ind)
            {
                return ind == 0 ? m_p0 : m_p1;
            }
            const Segment& get() const
            {
                return *this;
            }
            Segment& get()
            {
                return *this;
            }
            const MovetkPoint& vertex(std::size_t ind) const
            {
                return ind == 0 ? m_p0 : m_p1;
            }
            MovetkPoint& vertex(std::size_t ind)
            {
                return ind == 0 ? m_p0 : m_p1;
            }
        };
        using MovetkSegment = Segment;

        using NT = typename LoopsLib::NT;

        using Polyline = std::vector<MovetkPoint>;

        using Trajectory = Polyline;
        using TimestampedTrajectory = std::vector<std::pair<MovetkPoint, NT>>;
        using GraphTrajectory = std::vector<LoopsLib::DS::BaseGraph::EdgeIndex>;

        inline void convertCRS(const OGRSpatialReference& curr, const OGRSpatialReference& target, Trajectory& traj) const
        {
            auto deleter = [](auto* pntr) {if (pntr)OGRCoordinateTransformation::DestroyCT(pntr); };
            std::unique_ptr<OGRCoordinateTransformation, decltype(deleter)> transform(
                OGRCreateCoordinateTransformation(const_cast<OGRSpatialReference*>(&curr), const_cast<OGRSpatialReference*>(&target)),
                deleter
            );
            for (auto& el : traj)
            {
                double x = el.x(), y = el.y();
                // Loss of precision unfortunately...
                transform->Transform(1, &x, &y);
                el.m_x = x;
                el.m_y = y;
            }
        }
        inline void convertCRS(const OGRSpatialReference& curr, const OGRSpatialReference& target, TimestampedTrajectory& traj) const
        {
            auto deleter = [](auto* pntr) {if (pntr)OGRCoordinateTransformation::DestroyCT(pntr); };
            std::unique_ptr<OGRCoordinateTransformation, decltype(deleter)> transform(
                OGRCreateCoordinateTransformation(const_cast<OGRSpatialReference*>(&curr), const_cast<OGRSpatialReference*>(&target)),
                deleter
            );
            for (auto& el : traj)
            {
                double x = el.first.x(), y = el.first.y();
                // Loss of precision unfortunately...
                transform->Transform(1, &x, &y);
                MovetkPoint pnt;
                pnt.m_x = x;
                pnt.m_y = y;
                el = std::make_pair(pnt, el.second);
            }
        }

        template<typename Deleter>
        inline void convertCRS(const std::unique_ptr<OGRCoordinateTransformation, Deleter>& transform, Trajectory& traj) const
        {
            for (auto& el : traj)
            {
                double x = el.x(), y = el.y();
                // Loss of precision unfortunately...
                transform->Transform(1, &x, &y);
                el.m_x = x;
                el.m_y = y;
            }
        }
        template<typename Deleter>
        inline void convertCRS(const std::unique_ptr<OGRCoordinateTransformation, Deleter>& transform, TimestampedTrajectory& traj) const
        {
            for (auto& el : traj)
            {
                double x = el.first.x(), y = el.first.y();
                // Loss of precision unfortunately...
                transform->Transform(1, &x, &y);
                MovetkPoint pnt;
                pnt.m_x = x;
                pnt.m_y = y;
                el = std::make_pair(pnt, el.second);
            }
        }

        struct detail
        {
            template<typename Trajectory_t>
            struct TrajectorySet
            {
                // Spatial reference
                OGRSpatialReference m_ref{};

                std::vector<std::string> ids;
                std::vector<Trajectory_t> trajectories;

                std::size_t size() const
                {
                    return trajectories.size();
                }

                auto begin() const { return trajectories.begin(); }
                auto end() const { return trajectories.end(); }
                auto begin() { return trajectories.begin(); }
                auto end() { return trajectories.end(); }
            };
        };
        using TrajectorySet = detail::TrajectorySet<Trajectory>;
        using TimestampedTrajectorySet = detail::TrajectorySet<TimestampedTrajectory>;
        using GraphTrajectorySet = detail::TrajectorySet<GraphTrajectory>;
        ;
    };


    namespace Algs
    {
        // Edge path, possibly with degeneracies
        using BasisElement = std::vector<LoopsLib::DS::BaseGraph::Id_t>;
        // The field type
        using FieldType = std::vector<NT>;
    }

}
#endif