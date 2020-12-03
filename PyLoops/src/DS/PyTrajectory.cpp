#include <PyLoops/DS/PyTrajectory.h>
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/MapProviders.h>
#include "core/gps.hpp"
#include <pybind11/stl.h>
#include "cpl_conv.h"
#include <fstream>
#include <LoopsIO/TrajectorySetSerializer.h>
#include "LoopsLib/Algs/Processing/TrajectorySetFrechet.h"
#include "movetk/metric/Norm.h"


void PyLoops::ds::PyTrajectory::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;

    //Register graph class
    using Traj = FMM::CORE::Trajectory;
    py::class_<Traj>(mod, "Trajectory")
        .def(py::init<>())
    .def("addPoint", [](Traj& traj, LoopsLib::NT x, LoopsLib::NT y, LoopsLib::NT t)
    {
        traj.timestamps.push_back(t);
        traj.geom.add_point(x, y);
    })
        .def("getX", [](Traj& t, std::size_t ind)
    {
        return t.geom.get_x(ind);
    })
        .def("getY", [](Traj& t, std::size_t ind)
    {
        return t.geom.get_y(ind);
    })
        .def("getT", [](Traj& t, std::size_t ind)
    {
        return t.timestamps[ind];
    }).def("size", [](Traj& t)
    {
        return t.timestamps.size();
    }).def("getPoint",[](Traj& t, std::size_t ind)
    {
        return std::make_pair(t.geom.get_x(ind), t.geom.get_y(ind));
    });
}

void PyLoops::ds::TrajectoryList::registerPy(pybind11::module& mod)
{
    namespace py = pybind11;
    using TL = TrajectoryList;
    py::class_<TL>(mod, "TrajectoryList")
        .def(py::init<>())
        .def("addTrajectory", [](TL& tList, const FMM::CORE::Trajectory& traj)
    {
        tList.m_trajectories.push_back(traj);
    })
        .def("getTrajectory", [](TL& tList, std::size_t ind)
    {
        return tList.m_trajectories[ind];
    }).def("clear", [](TL& t) {t.m_trajectories.clear(); })
    .def("convertToGraphSpatRef", [](TL& t, const std::string& wellKnownGeog, LoopsLib::DS::EmbeddedGraph* graph)
    {
        OGRSpatialReference ref;
        ref.SetWellKnownGeogCS(wellKnownGeog.c_str());
        auto coordTrans = OGRCreateCoordinateTransformation(&ref, &graph->spatialRef());
        for(auto& traj : t.m_trajectories)
        {
            for(std::size_t i = 0 ; i < traj.timestamps.size(); ++i)
            {
                double x = traj.geom.get_x(i);
                double y = traj.geom.get_y(i);
                coordTrans->Transform(1, &x, &y);

                traj.geom.set_x(i,x);
                traj.geom.set_y(i,y);
            }
        }


        OGRCoordinateTransformation::DestroyCT(coordTrans);
    })
    .def("size", [](TL& t)
    {
        return t.m_trajectories.size();
    }).def("readFromWkbCsv",[](TL& tl, const std::string& filePath, const std::string& separator, int trajectoryColumn, bool hasHeader)
    {
        std::ifstream stream(filePath);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file " + filePath);
        }
        std::string line;
        std::stringstream lineStream;
        int currId = 0;
        int lineNum = 0;
        while (std::getline(stream, line))
        {
            if (hasHeader && lineNum == 0)
            {
                hasHeader = false;
                continue;
            }
            ++lineNum;

            lineStream.str(line);
            int col = 0;
            std::string part;
            while (std::getline(lineStream, part, separator[0]))
            {
                if (col != trajectoryColumn)
                {
                    ++col;
                    continue;
                }
                int byteCount;
                auto* data = CPLHexToBinary(part.data(), &byteCount);
                // Decode with GDAL
                OGRGeometry* geom;
                // TODO result of createFromWkb
                auto res = OGRGeometryFactory::createFromWkb(data, nullptr, &geom, byteCount);
                if (!geom)
                {
                    std::cout << "Loading geom failed from WKB:" << res << ", part:" << part << std::endl;
                    continue;
                }
                if (wkbFlatten(geom->getGeometryType()) != wkbLineString)
                {
                    std::cout << "Unknown geometry type" << std::endl;
                    CPLFree(data);
                    OGRGeometryFactory::destroyGeometry(geom);
                    continue;
                }
                OGRLineString* lineStr = geom->toLineString();
                auto* it = lineStr->getPointIterator();
                OGRPoint p;
                FMM::CORE::Trajectory traj{ "",0,currId, {}, {} };
                while (it->getNextPoint(&p))
                {
                    traj.geom.add_point(p.getX(), p.getY());
                    traj.timestamps.push_back(p.getM());
                }
                //boost::geometry::read_wkb(wkb.begin(), wkb.end(), l.get_geometry());
                OGRPointIterator::destroy(it);
                tl.m_trajectories.push_back(traj);
                ++currId;
            }
        }
    })
    .def("readFromWktCsv", [](TL& tl, const std::string& filePath, const std::string& separator, int trajectoryColumn, bool hasHeader)
    {
        std::ifstream stream(filePath);
        if (!stream.is_open())
        {
            throw std::runtime_error("Could not open file " + filePath);
        }
        std::string line;
        int currId = 0;
        int lineNum = 0;
        bool skipHeader = hasHeader;
        while (std::getline(stream, line))
        {
            if(skipHeader && lineNum == 0)
            {
                skipHeader = false;
                continue;
            }
            std::stringstream lineStream;
            ++lineNum;
            lineStream.str(line);
            int col = 0;
            std::string part;
            while (std::getline(lineStream, part, separator[0]))
            {
                if (col != trajectoryColumn)
                {
                    ++col;
                    continue;
                }
                const char* data = part.c_str();
                // Decode with GDAL
                OGRGeometry* geom;
                // TODO result of createFromWkb
                auto res = OGRGeometryFactory::createFromWkt(data, nullptr, &geom);
                if (!geom)
                {
                    std::cout << "Loading geom failed from WKT:" << res << ", part:" << part << std::endl;
                    continue;
                }
                if (wkbFlatten(geom->getGeometryType()) != wkbLineString)
                {
                    std::cout << "Unknown geometry type" << std::endl;
                    OGRGeometryFactory::destroyGeometry(geom);
                    continue;
                }
                OGRLineString* lineStr = geom->toLineString();
                auto* it = lineStr->getPointIterator();
                OGRPoint p;
                FMM::CORE::Trajectory traj{ "",0,currId, {}, {} };
                while (it->getNextPoint(&p))
                {
                    traj.geom.add_point(p.getX(), p.getY());
                    traj.timestamps.push_back(p.getM());
                }
                //boost::geometry::read_wkb(wkb.begin(), wkb.end(), l.get_geometry());
                OGRPointIterator::destroy(it);
                tl.m_trajectories.push_back(traj);
                ++currId;
            }
        }
    });
}
using StrongFrechet = movetk_algorithms::StrongFrechet<LoopsLib::MovetkGeometryKernel, movetk_support::squared_distance_d<LoopsLib::MovetkGeometryKernel, movetk_support::FiniteNorm<LoopsLib::MovetkGeometryKernel, 2>>>;

void PyLoops::ds::TrajectorySet::registerPy(pybind11::module& mod)
{
    LoopsIO::TrajectorySetSerializer::registerTypesOfTuple<LoopsIO::AllTrajectorySetProviders>();

    namespace py = pybind11;
    using TS = LoopsLib::MovetkGeometryKernel::TrajectorySet;
    py::class_<TS>(mod, "TrajectorySet")
        .def(py::init<>())
        .def_readonly("trajectoryIds", static_cast<const std::vector<std::string> TS::*>(&TS::ids))
        .def("convertToWellknownGeog", [](TS& ts, const std::string& geog)
    {
        using NT = LoopsLib::NT;
        OGRSpatialReference ref;
        ref.SetWellKnownGeogCS(geog.c_str());
        auto* trans = OGRCreateCoordinateTransformation(&ts.m_ref, &ref);
        for (auto& traj : ts.trajectories)
        {
            for (auto& pnt : traj)
            {
                double x = pnt.x(), y = pnt.y();
                trans->Transform(1, &x, &y);
                pnt = LoopsLib::MovetkGeometryKernel::MovetkPoint((NT)x, (NT)y);
            }
        }
        ts.m_ref = ref;
        OGRCoordinateTransformation::DestroyCT(trans);
    })
    .def("read",[](TS& rs, const std::string& file)
    {
        LoopsIO::TrajectorySetSerializer::read(file, rs);
    })
        .def("convertToCRSOfOther", [](TS& ts, const TS& otherSet)
    {
        using NT = LoopsLib::NT;
        auto* trans = OGRCreateCoordinateTransformation(&ts.m_ref, const_cast<OGRSpatialReference*>(&otherSet.m_ref));
        for (auto& traj : ts.trajectories)
        {
            for (auto& pnt : traj)
            {
                double x = pnt.x(), y = pnt.y();
                trans->Transform(1, &x, &y);
                pnt.m_x = x;
                pnt.m_y = y;
            }
        }
        ts.m_ref = otherSet.m_ref;
        OGRCoordinateTransformation::DestroyCT(trans);
    })
    .def("computeFrechetToOther", [](const TS& self, int index, const TS& other, int otherIndex)
    {
        StrongFrechet fr;
        fr.setTolerance(0.1);
        fr.setMode(StrongFrechet::Mode::DoubleAndSearch);
        LoopsLib::NT res = fr(self.trajectories[index].begin(), self.trajectories[index].end(), other.trajectories[otherIndex].begin(), other.trajectories[otherIndex].end());
        return static_cast<double>(res);
    })
    .def("write", [](const TS& rs, const std::string& file)
    {
        LoopsIO::TrajectorySetSerializer::write(file, rs);
    })
        .def("trajectory", [](TS& ts, int index)
    {
        return ts.trajectories[index];
    });
}

void PyLoops::ds::TimestampedTrajectorySet::registerPy(pybind11::module& mod)
{
    LoopsIO::TimestampedTrajectorySetSerializer::registerTypesOfTuple<LoopsIO::AllTimestampedTrajectorySetProviders>();

    using TS = LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet;
    namespace py = pybind11;
    py::class_<TS>(mod, "TimestampedTrajectorySet")
    .def(py::init<>())
    .def_readonly("trajectoryIds", static_cast<const std::vector<std::string> TS::*>(&TS::ids) )
        .def("read", [](TS& rs, const std::string& file)
    {
        LoopsIO::TimestampedTrajectorySetSerializer::read(file, rs);
    })
        .def("write", [](const TS& rs, const std::string& file)
    {
        LoopsIO::TimestampedTrajectorySetSerializer::write(file, rs);
    })
    .def("convertToTrajectorySet", [](TS& ts)
    {
        LoopsLib::MovetkGeometryKernel::TrajectorySet newTs;
        newTs.m_ref = ts.m_ref;
        newTs.ids = ts.ids;
        for(const auto& el: ts)
        {
            newTs.trajectories.push_back({});
            for(const auto& pnt: el)
            {
                newTs.trajectories.back().push_back(pnt.first);
            }
        }
        return newTs;
    })
    .def("convertToWellknownGeog", [](TS& ts, const std::string& geog)
    {
        using NT = LoopsLib::NT;
        OGRSpatialReference ref;
        ref.SetWellKnownGeogCS(geog.c_str());
        auto* trans = OGRCreateCoordinateTransformation(&ts.m_ref, &ref);
        for(auto& traj: ts.trajectories)
        {
            for(auto& pnt : traj)
            {
                double x = pnt.first.x(), y = pnt.first.y();
                trans->Transform(1, &x, &y);
                pnt = std::make_pair(LoopsLib::MovetkGeometryKernel::MovetkPoint((NT)x, (NT)y), pnt.second);
            }
        }
        ts.m_ref = ref;
        OGRCoordinateTransformation::DestroyCT(trans);
    })
    .def("trajectory", [](TS& ts, int index)
    {
        return ts.trajectories[index];
    });
}

void PyLoops::ds::GraphTrajectorySet::registerPy(pybind11::module& mod)
{
    LoopsIO::GraphTrajectorySetSerializer::registerTypesOfTuple<LoopsIO::AllGraphTrajectorySetProviders>();

    using TS = LoopsLib::MovetkGeometryKernel::GraphTrajectorySet;
    namespace py = pybind11;
    py::class_<TS>(mod, "GraphTrajectorySet")
        .def(py::init<>())
        .def_readonly("trajectoryIds", static_cast<const std::vector<std::string> TS::*>(&TS::ids))
        .def("read",[](TS& set, const std::string& filePath)
        {
            LoopsIO::GraphTrajectorySetSerializer::read(filePath, set);
        })
        .def("verifyPaths", [](const TS& ts, const LoopsLib::DS::EmbeddedGraph* graph)
        {
            std::size_t index = 0;
            for (const auto& traj : ts)
            {
                for (std::size_t i = 0; i < traj.size() - 1; ++i)
                {
                    auto first = graph->edge(traj[i])->m_sink->id();
                    auto second = graph->edge(traj[i + 1])->m_source->id();
                    if (graph->edge(traj[i])->m_sink->id() != graph->edge(traj[i+1])->m_source->id())
                    {
                        py::print("Trajectory ", index, " does not connect at edge ", i);
                        py::print("\t", first, " vs ", second);
                    }
                }
                ++index;
            }
        })
        .def("convertToTrajectorySet",[](const TS& ts, LoopsLib::DS::EmbeddedGraph* graph)
    {
            LoopsLib::MovetkGeometryKernel::TrajectorySet set;
            set.m_ref = graph->spatialRef();
            set.ids = ts.ids;
            for(std::size_t i = 0; i < ts.size();++i)
            {
                // Convert path to coordinates
                set.trajectories.push_back({});
                auto& curr = set.trajectories.back();
                curr.push_back(graph->vertexLocation(graph->edge(ts.trajectories[i][0])->m_source));
                for(const auto& eId: ts.trajectories[i])
                {
                    curr.push_back(graph->vertexLocation(graph->edge(eId)->m_sink));
                }
            }
            return set;
    })
        .def("write", [](const TS& set, const std::string& filePath)
        {
            LoopsIO::GraphTrajectorySetSerializer::write(filePath, set);
        })
        .def("trajectory", [](TS& ts, int index)
    {
        return ts.trajectories[index];
    });
}
