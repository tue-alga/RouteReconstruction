#include <LoopsLib/Geometry/Interval.h>
using namespace LoopsLib::Geometry;


Interval::Interval(NT min, NT max) : min(min), max(max)
{
}

Interval::Interval()
{
}

bool Interval::isEmpty() const
{
    return max < min;
}

Interval Interval::intersection(const Interval& other) const
{
    return Interval(std::max(min, other.min), std::min(max, other.max));
}

bool Interval::intersectsWith(const Interval& other) const
{
    return !(max < other.min || min > other.max);
}

bool Interval::intersectsWith(const NT& min, const NT& max) const
{
    return !(this->max < min || this->min > max);
}

Interval& Interval::intersectWith(const Interval& other)
{
    min = std::max(min, other.min);
    max = std::min(max, other.max);
    return *this;
}

Interval& Interval::scale(const NT& val)
{
    min *= val;
    max *= val;
    return *this;
}

Interval& Interval::offset(const NT& amount)
{
    min += amount;
    max += amount;
    return *this;
}

Interval& Interval::makePositive()
{
    if (max < min) std::swap(max, min);
    return *this;
}
