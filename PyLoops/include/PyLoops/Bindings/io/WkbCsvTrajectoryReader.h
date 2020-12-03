#ifndef PYLOOPS_IO_WKBCSVTRAJECTORYREADER_H
#define PYLOOPS_IO_WKBCSVTRAJECTORYREADER_H
#include <pybind11/pybind11.h>
#include "core/gps.hpp"
#include "cpl_conv.h"

namespace PyLoops::io
{
    class PyWkbCsvTrajectoryReader
    {
        char m_separator = ';';
    public:
        void read(const std::string& filePath, int trajectoryColumn, std::vector<FMM::CORE::Trajectory>& trajectories)
        {
            std::ifstream stream(filePath);
            if(!stream.is_open())
            {
                throw std::runtime_error("Could not open file " + filePath);
            }
            std::string line;
            std::stringstream lineStream;
            int currId = 0;
            while(std::getline(stream, line))
            {
                lineStream.str(line);
                int col = 0;
                std::string part;
                while(std::getline(lineStream, part, m_separator))
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
                    OGRGeometryFactory::createFromWkb(data, nullptr, &geom, byteCount);
                    if(!geom)
                    {
                        std::cout << "Loading geom failed from WKB" << std::endl;
                        continue;
                    }
                    if(geom->getGeometryType() != wkbLineString)
                    {
                        std::cout << "Unknown geometry type" << std::endl;
                        CPLFree(data);
                        OGRGeometryFactory::destroyGeometry(geom);
                        continue;
                    }
                    OGRLineString* lineStr = geom->toLineString();
                    auto res = FMM::CORE::ogr2linestring(lineStr);
                    trajectories.push_back(FMM::CORE::Trajectory{ "",0,currId, res, {} });
                    ++currId;
                }
            }
        }

        static void registerPy(pybind11::module& mod)
        {
            pybind11::class_<PyWkbCsvTrajectoryReader>(mod, "WkbCsvTrajectoryReader")
                .def("read", &PyWkbCsvTrajectoryReader::read);
        }
    };
}
#endif 