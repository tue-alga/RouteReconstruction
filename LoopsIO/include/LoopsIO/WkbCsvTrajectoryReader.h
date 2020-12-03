#ifndef LOOPSLIB_IO_WKBCSVTRAJECTORYREADER_H
#define LOOPSLIB_IO_WKBCSVTRAJECTORYREADER_H
#include "cpl_conv.h"
#include "ogrsf_frmts.h"
#include <fstream>
#include <sstream>
#include <movetk/geom/GeometryInterface.h>

namespace LoopsIO
{
    template<typename T, typename Deleter>
    struct ScopedPointer
    {
        T* m_pntr;
        Deleter m_deleter;
        bool m_doDelete = true;

        ScopedPointer(T* pntr, Deleter deleter)
            :m_pntr(pntr), m_deleter(deleter){}
        ScopedPointer(const ScopedPointer&) = delete;
        ScopedPointer(ScopedPointer&& other) noexcept
        {
            m_pntr = other.m_pntr;
            m_deleter = other.m_deleter;
            other.m_deleter = nullptr;
        }
        void untrack()
        {
            m_doDelete = false;
        }
        auto operator->()
        {
            return m_pntr;
        }
        T* raw()
        {
            return m_pntr;
        }
        // Don't allow copy
        ScopedPointer& operator=(const ScopedPointer& other) = delete;

        ScopedPointer& operator=(ScopedPointer&& other) noexcept
        {
            m_pntr = other.m_pntr;
            m_deleter = other.m_deleter;
            other.m_deleter = nullptr;
            return *this;
        }
        ~ScopedPointer()
        {
            if(m_pntr && m_doDelete) m_deleter(m_pntr);
            m_pntr = nullptr;
        }
    };

    /**
     * Archetype of visitors
     */
    struct Visitor
    {
        void visitField(const std::string& fielName, const std::string& fieldValue)
        {
        }
        void visitGeometry(const OGRPoint& point){}
        void visitNewTrajectory(){}
    };

    struct ColumnBasedVisitor
    {
        struct Trajectory
        {
            std::map<std::string, std::string> fields;
            std::vector<double> lat;
            std::vector<double> lon;
            std::vector<double> time;
        };
        std::vector<Trajectory> m_trajectories;
        Trajectory& curr()
        {
            return m_trajectories.back();
        }
        void visitField(const std::string& fieldName, const std::string& fieldValue)
        {
            m_trajectories.back().fields[fieldName] = fieldValue;
        }
        void visitGeometry(const OGRPoint& point)
        {
            // Assuming x and y are actually reasonable
            curr().lon.push_back(point.getX());
            curr().lat.push_back(point.getY());
            curr().time.push_back(point.getM());
            if(curr().time.size() >= 2)
            {
                if (curr().time.at(curr().time.size() - 1) - curr().time.at(curr().time.size() - 2) < 0) std::cout << "Negative time " << std::endl;
            }
        }
        void visitNewTrajectory()
        {
            m_trajectories.emplace_back();
        }
    };

    template<typename StringLikeType>
    struct string_split_iterator
    {
        const StringLikeType& m_string;
        char m_delim;
        std::size_t m_currentPos;
        std::size_t m_stringSize;

        string_split_iterator(const StringLikeType& string, char delim): m_string(string),
            m_delim(delim), m_currentPos(0)
        {
            auto end = m_currentPos;
            for(; end < m_string.size(); ++end)
            {
                if(m_string[end] == delim)
                {
                    m_stringSize = end - m_currentPos;
                }
            }
            if (end == m_string.size()) m_stringSize = m_string.size();
        }

        string_split_iterator end() const
        {
            string_split_iterator it(m_string, m_delim);
            it.m_currentPos = m_string.size();
            return it;
        }

        string_split_iterator& operator++()
        {
            m_currentPos += m_stringSize + 1;
            if (m_currentPos >= m_string.size()) {
                m_currentPos = m_string.size();
                m_stringSize = 0;
            }
            else
            {
                auto end = m_currentPos;
                for (; end < m_string.size(); ++end)
                {
                    if (m_string[end] == m_delim)
                    {
                        m_stringSize = end - m_currentPos;
                    }
                }
                if (end == m_string.size()) m_stringSize = m_string.size() - m_currentPos;
            }
            return *this;
        }
        std::string_view operator*() const
        {
            if (m_stringSize == 0) return std::string_view();
            return std::string_view(m_string.data() + m_currentPos, m_stringSize);
        }

        bool operator==(const string_split_iterator& other) const
        {
            return m_currentPos == other.m_currentPos;
        }
        bool operator!=(const string_split_iterator& other) const
        {
            return !(*this == other);
        }

    };

    class WkbCsvTrajectoryReader
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
        int m_idColumn=-1;
        InterruptHooks m_interrupt;
        int m_maxTrajCount = -1;
    public:
        WkbCsvTrajectoryReader(char separator, int trajectoryColumn, int idColumn=-1):m_separator(separator), m_trajectoryColumn(trajectoryColumn), m_idColumn(idColumn),
            m_interrupt({ []() {return false; } , 100 }) {}
        void setInterruptor(InterruptHooks interruptor)
        {
            m_interrupt = interruptor;
        }
        void setMaximumReadTrajectories(int value)
        {
            m_maxTrajCount = value;
        }
        
        template<typename Visitor>
        bool read(const std::string& filePath, Visitor& visitor, bool hasHeader = true)
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
                if(isFirstLine)
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
                    if(col-1 == m_idColumn)
                    {
                        visitor.visitField("id", part);
                    }
                    if (col-1 != m_trajectoryColumn)
                    {
                        continue;
                    }
                    //std::cout << "Data : " << part << std::endl;
                    int byteCount;
                    auto data = ScopedPointer(CPLHexToBinary(part.data(), &byteCount), CPLFree);
                    // Decode with GDAL
                    OGRGeometry* geomHolder;
                    // TODO result of createFromWkb
                    OGRErr error = OGRGeometryFactory::createFromWkb(data.raw(), nullptr, &geomHolder, byteCount);
                    if (!geomHolder)
                    {
                        std::cout << "### Loading geom failed from WKB:\n" 
                        << " Err code:" << error << ", part:" << part <<  ", part size: "<< part.size() << ", bytecount:" << byteCount << ", data:"<<data.raw()<<std::endl;
                        continue;
                    }
                    auto geom = ScopedPointer(geomHolder, OGRGeometryFactory::destroyGeometry);
                    if (wkbFlatten(geom->getGeometryType()) != wkbLineString) // Flatten because there is some bit pollution... See GDAL docs.
                    {
                        std::cout << "Unknown geometry type:" << geom->getGeometryName() << std::endl;
                        continue;
                    }
                    OGRLineString* lineStr = geom->toLineString();
                    auto pntIt = ScopedPointer(lineStr->getPointIterator(), OGRPointIterator::destroy);
                    OGRPoint pnt;
                    while(pntIt->getNextPoint(&pnt))
                    {
                        visitor.visitGeometry(pnt);
                    }
                    ++currId;
                }
            }
            return true;
        }
    };
    class WktCsvTrajectoryReader
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
        WktCsvTrajectoryReader(char separator, int trajectoryColumn, int idColumn = -1) :m_separator(separator), m_trajectoryColumn(trajectoryColumn), m_idColumn(idColumn),
            m_interrupt({ []() {return false; } , 100 }) {}
        void setInterruptor(InterruptHooks interruptor)
        {
            m_interrupt = interruptor;
        }
        void setMaximumReadTrajectories(int value)
        {
            m_maxTrajCount = value;
        }

        template<typename Visitor>
        bool read(const std::string& filePath, Visitor& visitor, bool hasHeader = true)
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
                // Reset stringstream, move to start
                lineStream.clear();
                lineStream.str(line);
                lineStream.seekg(0);

                // The current column
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
                    // Decode with GDAL
                    OGRGeometry* geomHolder;
                    // TODO result of createFromWkb
                    OGRErr error = OGRGeometryFactory::createFromWkt(part.c_str(), nullptr, &geomHolder);
                    if (!geomHolder)
                    {
                        std::cout << "### Loading geom failed from WKT:\n"
                            << " Err code:" << error << ", part:" << part << ", part size: " << part.size() << std::endl;
                        continue;
                    }
                    auto geom = ScopedPointer(geomHolder, OGRGeometryFactory::destroyGeometry);
                    if (wkbFlatten(geom->getGeometryType()) != wkbLineString) // Flatten because there is some bit pollution... See GDAL docs.
                    {
                        std::cout << "Unknown geometry type:" << geom->getGeometryName() << std::endl;
                        continue;
                    }
                    OGRLineString* lineStr = geom->toLineString();
                    auto pntIt = ScopedPointer(lineStr->getPointIterator(), OGRPointIterator::destroy);
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
}

#endif