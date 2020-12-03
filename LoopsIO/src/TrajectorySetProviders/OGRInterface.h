#ifndef LOOPSIO_TRAJECTORYSETPROVIDERS_OGRINTERFACE_H
#define LOOPSIO_TRAJECTORYSETPROVIDERS_OGRINTERFACE_H
#include "cpl_conv.h"
#include "ogrsf_frmts.h"
#include <functional>
#include <sstream>
#include <fstream>
#include <iostream>

namespace LoopsIO::TrajectorySetProviders
{

    class CsvLikeOgrTrajectoryReader
    {
    public:
        struct InterruptHooks
        {
            std::function<bool()> hasToStopCb;
            std::size_t linesPerCheck = 50;
        };
    private:
        char m_separator = ';';
        int m_trajectoryColumn;
        int m_idColumn = -1;
        InterruptHooks m_interrupt;
        int m_maxTrajCount = -1;
    public:
        CsvLikeOgrTrajectoryReader(char separator, int trajectoryColumn, int idColumn = -1) :m_separator(separator), m_trajectoryColumn(trajectoryColumn), m_idColumn(idColumn),
            m_interrupt({ []() {return false; } , 100 }) {}
        void setInterruptor(InterruptHooks interruptor)
        {
            m_interrupt = interruptor;
        }
        void setMaximumReadTrajectories(int value)
        {
            m_maxTrajCount = value;
        }

        template<typename Visitor, typename OgrToGeometryFunc>
        bool read(const std::string& filePath, Visitor& visitor, OgrToGeometryFunc&& reader, bool hasHeader = true)
        {
            std::ifstream stream(filePath);
            if (!stream.is_open())
            {
                throw std::runtime_error("Could not open file " + filePath);
            }
            std::string line;
            std::stringstream lineStream;
            int currId = 0;
            bool isFirstLine = true;
            std::size_t lineNum = 0;
            while (std::getline(stream, line) && (m_maxTrajCount < 0 || lineNum < m_maxTrajCount))
            {
                if (lineNum % m_interrupt.linesPerCheck == 0 && m_interrupt.hasToStopCb()) return false;
                ++lineNum;
                if (isFirstLine)
                {
                    isFirstLine = false;
                    if (hasHeader) continue;
                }
                visitor.visitNewTrajectory();
                lineStream.clear();
                lineStream.str(line);
                lineStream.seekg(0);
                int col = 0;
                std::string part;
                while (std::getline(lineStream, part, m_separator))
                {
                    ++col;
                    if (col - 1 == m_idColumn)
                    {
                        visitor.visitField("id", part);
                    }
                    if (col - 1 != m_trajectoryColumn)
                    {
                        continue;
                    }
                    //std::cout << "Data : " << part << std::endl;
                    int byteCount;
                    auto del = [](auto* pntr) { CPLFree(pntr); };
                    auto data = std::unique_ptr<GByte, decltype(del)>(CPLHexToBinary(part.data(), &byteCount), del);
                    // Decode with GDAL
                    OGRGeometry* geomHolder;
                    OGRErr error = reader(part, &geomHolder);
                    if (!geomHolder)
                    {
                        std::cout << "### Loading geom failed from WKB:\n"
                            << " Err code:" << error << ", part:" << part << ", part size: " << part.size() << std::endl;
                        continue;
                    }
                    auto delGeom = [](OGRGeometry* pntr) {OGRGeometryFactory::destroyGeometry(pntr); };
                    auto geom = std::unique_ptr<OGRGeometry, decltype(delGeom)>(geomHolder, delGeom);

                    if (wkbFlatten(geom->getGeometryType()) != wkbLineString) // Flatten because there is some bit pollution... See GDAL docs.
                    {
                        std::cout << "Unknown geometry type:" << geom->getGeometryName() << std::endl;
                        continue;
                    }
                    auto* lineStr = geom->toLineString();
                    // Assuming there is a point iterator
                    auto delPntIterator = [](OGRPointIterator* it) {OGRPointIterator::destroy(it); };
                    auto pntIt = std::unique_ptr<OGRPointIterator, decltype(delPntIterator)>(lineStr->getPointIterator(), delPntIterator);
                    OGRPoint pnt;
                    while (pntIt->getNextPoint(&pnt))
                    {
                        visitor.visitGeometry(pnt);
                    }
                    ++currId;
                }
            }
            return true;
        }
    };


    struct OgrTrajectoryVisitor
    {
        LoopsLib::MovetkGeometryKernel::TrajectorySet& out;
        OgrTrajectoryVisitor(LoopsLib::MovetkGeometryKernel::TrajectorySet& out) :out(out) {}

        void visitField(const std::string& name, const std::string& value)
        {
            if (name == "id")
            {
                out.ids.push_back(value);
            }
        }
        void visitNewTrajectory()
        {
            out.trajectories.emplace_back();
        }
        void visitGeometry(const OGRPoint& point)
        {
            out.trajectories.back().emplace_back(static_cast<LoopsLib::NT>(point.getX()), static_cast<LoopsLib::NT>(point.getY()));
        }
    };
    struct OgrTimestampedTrajectoryVisitor
    {
        LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& out;
        OgrTimestampedTrajectoryVisitor(LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& out) :out(out) {}

        void visitField(const std::string& name, const std::string& value)
        {
            if (name == "id")
            {
                out.ids.push_back(value);
            }
        }
        void visitNewTrajectory()
        {
            out.trajectories.emplace_back();
        }
        void visitGeometry(const OGRPoint& point)
        {
            out.trajectories.back().emplace_back(LoopsLib::MovetkGeometryKernel::MovetkPoint{ (LoopsLib::NT)point.getX(), (LoopsLib::NT)point.getY() }, (LoopsLib::NT)point.getM());
            ;
        }
    };
}

#endif