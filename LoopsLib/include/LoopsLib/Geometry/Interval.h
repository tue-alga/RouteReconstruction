#ifndef GEOMETRY_INTERVAL_H
#define GEOMETRY_INTERVAL_H
#include <LoopsLib/Algs/Types.h>
namespace LoopsLib::Geometry
{
    using NT = MovetkGeometryKernel::NT;

    struct Interval
    {
        NT min = std::numeric_limits<NT>::max();
        NT max = std::numeric_limits<NT>::lowest();
        Interval(NT min, NT max);
        Interval();
        Interval& operator=(const Interval& other)
        {
            min = other.min;
            max = other.max;
            return *this;
        }
        Interval(const Interval& other) : min(other.min), max(other.max) {}

        /**
         * \brief Returns whether the interval is empty.
         * This is internally handled by setting max < min. This ensures
         * that interval [a,a] for some value a is not empty.
         * \return Is the interval empty.
         */
        bool isEmpty() const;

        bool contains(const NT& value) const
        {
            return min <= value && value <= max;
        }
        bool containsApprox(const NT& value, NT maxDist) const
        {
            return (min-maxDist) <= value && value <= max+maxDist;
        }

        /**
         * \brief Returns a new interval that is the intersection of the current
         * interval with the other interval
         * \param other The other interval
         * \return The intersection. Potentially empty.
         */
        Interval intersection(const Interval& other) const;

        bool intersectsWith(const Interval& other) const;
        bool intersectsWith(const NT& min, const NT& max) const;

        /**
         * \brief Intersects the current interval with the other interval and assigns
         * the intersection to this interval
         * \param other The other interval
         * \return Reference to self
         */
        Interval& intersectWith(const Interval& other);

        /**
         * \brief Scales the minimum and maximum of this interval by the given value
         * \param val The value
         * \return Reference to self
         */
        Interval& scale(const NT& val);

        /**
         * \brief Offsets the interval by the given amount
         * \param amount The amount
         * \return Reference to self
         */
        Interval& offset(const NT& amount);

        /**
         * \brief Makes the interval positive: ensures that the minimum is less than or equal to the maximum.
         * \return Reference to self
         */
        Interval& makePositive();
    };
}
namespace std
{
    inline std::ostream& operator<<(std::ostream& str, const LoopsLib::Geometry::Interval& inter)
    {
        str << "[" << inter.min << "," << inter.max << "]";
        return str;
    }
}
#endif