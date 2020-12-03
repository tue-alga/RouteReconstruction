#include <PyLoops/Bindings/ds/PyTrajectory.h>
#include <LoopsIO/GraphIO.h>
#include <LoopsIO/MapProviders.h>
#include "core/gps.hpp"
#include <pybind11/stl.h>
#include "core/gps.hpp"
#include "cpl_conv.h"
#include <fstream>

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
