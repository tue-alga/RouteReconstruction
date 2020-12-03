#ifndef LOOPSALGS_FRECHTE_FRECHETSETFINDING_H
#define LOOPSALGS_FRECHTE_FRECHETSETFINDING_H
#include <LoopsLib/Algs/Types.h>
namespace LoopsAlgs::Frechet
{
    class SpatialGrid
    {
        std::vector<std::vector<std::size_t>> m_elements;
        std::size_t m_rows, m_cols;
        std::pair<LoopsLib::NT, LoopsLib::NT> m_gridX;
        std::pair<LoopsLib::NT, LoopsLib::NT> m_gridY;
    public:
        using NT = LoopsLib::NT;
        using CellCoord = std::pair<std::size_t, std::size_t>;
        SpatialGrid(std::size_t rows, std::size_t cols):m_rows(rows), m_cols(cols)
        {
            m_elements.resize(m_rows*m_cols, {});
        }
        LoopsLib::NT cellWidth() const
        {
            return (m_gridX.second - m_gridX.first) / (LoopsLib::NT)m_cols;
        }
        LoopsLib::NT cellHeight() const
        {
            return (m_gridY.second - m_gridY.first) / (LoopsLib::NT)m_rows;
        }
        using CellCoord = std::pair<std::size_t, std::size_t>;

        std::pair<std::size_t, std::size_t> cellForLocation(const LoopsLib::MovetkGeometryKernel::MovetkPoint& pnt) const
        {
            const std::size_t x = (pnt.x() - m_gridX.first) / ((m_gridX.second - m_gridX.first) / (LoopsLib::NT)m_cols);
            const std::size_t y = (pnt.y() - m_gridY.first) / ((m_gridY.second - m_gridY.first) / (LoopsLib::NT)m_rows);
            return std::make_pair(x, y);
        }
        std::size_t cellIndex(const CellCoord& coord) const
        {
            return coord.first + coord.second * m_cols;
        }
        
        void insert(const LoopsLib::MovetkGeometryKernel::MovetkPoint& location, std::size_t id)
        {
            auto loc = cellIndex(cellForLocation(location));
            m_elements[loc].push_back(id);
        }
        void setBoundingBox(NT minX, NT maxX, NT minY, NT maxY)
        {
            m_gridX = std::make_pair(minX, maxX);
            m_gridY = std::make_pair(minY, maxY);
        }
    };

    class FrechetSetFinding
    {
    public:
        using Trajectory = LoopsLib::MovetkGeometryKernel::Trajectory;
    private:
        template<typename T>
        T clamp(T val, T minVal, T maxVal)
        {
            return std::min(std::max(minVal, val), maxVal);
        }

        std::size_t m_hashRows = 500;
        std::size_t m_hashCols = 500;
        std::vector<std::vector<std::size_t>> m_hashGrid;
        std::pair<LoopsLib::NT, LoopsLib::NT> m_gridX;
        std::pair<LoopsLib::NT, LoopsLib::NT> m_gridY;

        LoopsLib::NT cellWidth() const
        {
            return (m_gridX.second - m_gridX.first) / (LoopsLib::NT)m_hashCols;
        }
        LoopsLib::NT cellHeight() const
        {
            return (m_gridY.second - m_gridY.first) / (LoopsLib::NT)m_hashRows;
        }
        using CellCoord = std::pair<std::size_t, std::size_t>;

        std::pair<std::size_t, std::size_t> cellForCoordinate(const LoopsLib::MovetkGeometryKernel::MovetkPoint& pnt) const
        {
            const std::size_t x = (pnt.x() - m_gridX.first) / ((m_gridX.second - m_gridX.first) / (LoopsLib::NT)m_hashCols);
            const std::size_t y = (pnt.y() - m_gridY.first) / ((m_gridY.second - m_gridY.first) / (LoopsLib::NT)m_hashRows);
            return std::make_pair(x, y);
        }
        std::size_t cellIndex(const CellCoord& coord) const
        {
            return coord.first + coord.second * m_hashCols;
        }

        std::vector<std::size_t>& gridCell(const LoopsLib::MovetkGeometryKernel::MovetkPoint& pnt)
        {
            const std::size_t x = (pnt.x() - m_gridX.first) / ((m_gridX.second - m_gridX.first) / (LoopsLib::NT)m_hashCols);
            const std::size_t y = (pnt.y() - m_gridY.first) / ((m_gridY.second - m_gridY.first) / (LoopsLib::NT)m_hashRows);
            return m_hashGrid[clamp<std::size_t>(x,0, m_hashCols -1) + clamp<std::size_t>(y, 0, m_hashRows -1) * m_hashCols];
        }
        std::vector<std::size_t>& gridCellSafe(const LoopsLib::MovetkGeometryKernel::MovetkPoint& pnt)
        {
            if(pnt.x() < m_gridX.first || pnt.x() > m_gridX.second || pnt.y() < m_gridY.first || pnt.y() > m_gridY.second)
            {
                throw std::runtime_error("[FrechetSetFinding] Point is out of bounds in gridcell");
            }
            return gridCell(pnt);
        }

        /**
         * \brief Returns all cells within the square of diameter 2 epsilon, centered at the given point
         * \param point 
         * \param epsilon 
         * \return 
         */
        std::vector<std::size_t> cellsWithinDistance(const LoopsLib::MovetkGeometryKernel::MovetkPoint& point, LoopsLib::NT epsilon) const
        {
            std::vector<std::size_t> ret;
            auto minCell = cellForCoordinate(point - LoopsLib::MovetkGeometryKernel::MovetkPoint(-epsilon, -epsilon));
            auto maxCell = cellForCoordinate(point - LoopsLib::MovetkGeometryKernel::MovetkPoint(epsilon, epsilon));
            for(auto x = minCell.first; x <= maxCell.first; ++x)
            {
                for (auto y = minCell.second; y <= maxCell.second; ++y)
                {
                    ret.push_back(cellIndex(std::make_pair(x, y)));
                }
            }
            return ret;
        }

        bool isEqualTimeDistance(const Trajectory& t0, const Trajectory& t1, LoopsLib::NT epsilon)
        {
            const auto epsSq = epsilon * epsilon;

            auto lerpPos = [](const Trajectory& traj, std::size_t index, LoopsLib::NT factor)
            {
                return traj[index] + factor * (traj[index + 1] - traj[index]);
            };

            // Check first elements first
            if ((t0.front() - t1.front()).sqLength() > epsSq) return false;
            if ((t0.back() - t1.back()).sqLength() > epsSq) return false;

            const auto n = t0.size();
            const auto m = t1.size();
            // Simple case of equal size
            if(n == m)
            {
                for(auto i = 0; i < n; ++i)
                {
                    if ((t0[i] - t1[i]).sqLength() > epsSq) return false;
                }
                return true;
            }
            using NT = LoopsLib::NT;
            LoopsLib::NT ratio = m / static_cast<LoopsLib::NT>(n);
            // Otherwise, march the ray with coefficient  m / n through the freespace grid.
            LoopsLib::NT prevY = 0;
            for(auto i = 1; i < n; ++i)
            {
                const NT newY = i * ratio;
                for(std::size_t j = prevY+1; j < (std::size_t)newY; ++j)
                {
                    LoopsLib::NT t = (j - prevY) * n / (LoopsLib::NT)m;
                    if ((lerpPos(t0, i-1, t) - t1[j]).sqLength() > epsSq) return false;
                }
                if ((t0[i] - lerpPos(t1, (std::size_t)newY, newY - (std::size_t)newY)).sqLength() > epsSq) return false;
                prevY = newY;
            }
            return true;
        }
        const std::vector<LoopsLib::MovetkGeometryKernel::Trajectory>* m_trajectories = nullptr;

        // The simplification percentages to use
        std::vector<LoopsLib::NT> m_simplificationPercentages;
        // The simplifications, precomputed
        std::vector<LoopsLib::MovetkGeometryKernel::Trajectory> m_simplifications;
    public:
        FrechetSetFinding(int hashRows, int hashColumns):m_hashRows(hashRows), m_hashCols(hashColumns){}

        void setSimplificationPercentages(const std::vector<LoopsLib::NT>& percentages)
        {
            m_simplificationPercentages = percentages;
        }

        void precompute(const std::vector<LoopsLib::MovetkGeometryKernel::Trajectory>& trajectories)
        {
            m_trajectories = &trajectories;

            // Determine bounding box of the trajectories
            const auto lowest = std::numeric_limits<LoopsLib::NT>::lowest();
            const auto highest = std::numeric_limits<LoopsLib::NT>::max();

            m_gridX = std::make_pair(highest, lowest);
            m_gridY = std::make_pair(highest, lowest);
            for(const auto& el : trajectories)
            {
                for(const auto& pnt : el)
                {
                    m_gridX.first = std::min(m_gridX.first, pnt.m_x);
                    m_gridX.second = std::max(m_gridX.second, pnt.m_x);
                    m_gridY.first = std::min(m_gridY.first, pnt.m_y);
                    m_gridY.second = std::max(m_gridY.second, pnt.m_y);
                }
            }
            m_hashGrid.assign(m_hashCols*m_hashRows, {});
            // Add indices of trajectories to grid cells.
            for(auto i = 0; i < trajectories.size(); ++i)
            {
                const auto& el = trajectories[i];
                auto& startCell = gridCell(el.front());
                startCell.push_back(i);
                auto& endCell = gridCell(el.back());
                endCell.push_back(i);
            }
        }

        void compute(const Trajectory& trajectory, LoopsLib::NT epsilon, std::vector<std::size_t>& matches)
        {
            const auto epsSq = epsilon * epsilon;
            // Step 1) prune by spatial hashing
            auto cells = cellsWithinDistance(trajectory.front(), epsilon);
            std::set<std::size_t> startTrajectories, endTrajectories;
            for(const auto& c : cells)
            {
                for(const auto& tId: m_hashGrid[c])
                {
                    if((m_trajectories->at(tId).front()-trajectory.front()).sqLength() <= epsSq)
                    {
                        startTrajectories.insert(tId);
                    }
                }
            }
            auto endCells = cellsWithinDistance(trajectory.back(), epsilon);
            for (const auto& c : endCells)
            {
                for (const auto& tId : m_hashGrid[c])
                {
                    if ((m_trajectories->at(tId).back() - trajectory.back()).sqLength() <= epsSq)
                    {
                        endTrajectories.insert(tId);
                    }
                }
            }
            std::vector<std::size_t> candidates;
            auto candidatesEnd = std::set_intersection(startTrajectories.begin(), startTrajectories.end(), endTrajectories.begin(), endTrajectories.end(), candidates.begin());
            if (candidatesEnd == candidates.begin()) return;


            // Step 2 per trajectory) 
            // Step 2a decide on distance via simplifications
            // Step 2b decide with equal distance 
            // Step 2c decide via exact Frechet decision

        }
    };
}
#endif