#ifndef MATH_VECTOR_H
#define MATH_VECTOR_H
#include <vector>
#include <cassert>
#include <random>
#include <LoopsLib/Helpers/RandomHelpers.h>

namespace LoopsLib::Math
{
    class Bitset
    {
        std::vector<std::uint32_t> m_data;
    public:
        Bitset(std::size_t size)
        {
            m_data.assign(size / sizeof(std::uint32_t) + 1, 0);
        }
        bool operator[](std::size_t ind) const
        {
            std::size_t el = ind / sizeof(std::uint32_t);
            return m_data[el] & (1 << (ind - sizeof(std::uint32_t) * el));
        }
        void setBit(std::size_t ind)
        {
            std::size_t el = ind / sizeof(std::uint32_t);
            auto bit = (1 << (ind - sizeof(std::uint32_t) * el));
            m_data[el] = (m_data[el] & (~bit)) | bit;
        }
        void unsetBit(std::size_t ind)
        {
            std::size_t el = ind / sizeof(std::uint32_t);
            auto bit = (1 << (ind - sizeof(std::uint32_t) * el));
            m_data[el] = (m_data[el] & (~bit));
        }
    };


    template<typename Scalar>
    class ConstVectorWrapper
    {
        const std::vector<Scalar>* m_wrappedObj;
        template<typename Lambda>
        void foreachEl(Lambda l)
        {
            for (auto it = m_wrappedObj->begin(); it != m_wrappedObj->end(); ++it)
            {
                l(*it);
            }
        }
        std::vector<Scalar> m_innerContainer;
    public:
        const std::vector<Scalar>* raw() const
        {
            return m_wrappedObj;
        }

        ConstVectorWrapper(const std::vector<Scalar>* wrapobj) :m_wrappedObj(wrapobj) {}
        ConstVectorWrapper(const std::vector<Scalar>& wrapobj) :m_wrappedObj(&wrapobj) {}
        ConstVectorWrapper(std::size_t size, Scalar initialValue)
        {
            m_innerContainer.assign(size, initialValue);
            m_wrappedObj = &m_innerContainer;
        }
        std::size_t size() const
        {
            return m_wrappedObj->size();
        }

        Scalar total() const
        {
            Scalar total = 0;
            foreachEl([&total](Scalar el) {total += el; });
            return total;
        }
        Scalar maxCoeff() const
        {
            return *std::max_element(m_wrappedObj->begin(), m_wrappedObj->end());
        }
        Scalar minCoeff() const
        {
            return *std::min_element(m_wrappedObj->begin(), m_wrappedObj->end());
        }
        Scalar norm() const
        {
            return std::sqrt(std::accumulate(m_wrappedObj->begin(), m_wrappedObj->end(), (Scalar)0, [](Scalar acc, Scalar s1) {return acc + s1 * s1; }));
        }
        Scalar normSq() const
        {
            return std::accumulate(m_wrappedObj->begin(), m_wrappedObj->end(), (Scalar)0, [](Scalar acc, Scalar s1) {return acc + s1 * s1; });
        }

        Scalar operator[](std::size_t i) const
        {
            return (*m_wrappedObj)[i];
        }

        Scalar dot(const ConstVectorWrapper<Scalar>& other)
        {
            Scalar init = 0;
            for (std::size_t i = 0; i < m_wrappedObj->size(); ++i)
            {
                init += (*m_wrappedObj)[i] + other[i];
            }
            return init;
        }

        std::vector<Scalar> operator+(const ConstVectorWrapper<Scalar>& other) const
        {
            std::vector<Scalar> ret;
            ret.reserve(m_wrappedObj->size());
            std::copy(m_wrappedObj->begin(), m_wrappedObj->end(), std::back_inserter(ret));
            // Wrap temporary, modify underlying vector
            ConstVectorWrapper<Scalar> vw(&ret);
            vw += other;
            return ret;
        }
        std::vector<Scalar> operator-(const ConstVectorWrapper<Scalar>& other) const
        {
            std::vector<Scalar> ret;
            ret.reserve(m_wrappedObj->size());
            std::copy(m_wrappedObj->begin(), m_wrappedObj->end(), std::back_inserter(ret));
            // Wrap temporary, modify underlying vector
            ConstVectorWrapper<Scalar> vw(&ret);
            vw -= other;
            return ret;
        }
    };

    template<typename Scalar>
    class VectorWrapper
    {
        const std::vector<Scalar>* m_constWrappedObj;
        std::vector<Scalar>* m_wrappedObj;
        template<typename Lambda>
        void foreachEl(Lambda l)
        {
            for (auto it = m_wrappedObj->begin(); it != m_wrappedObj->end(); ++it)
            {
                l(*it);
            }
        }
        template<typename Lambda>
        void foreachEl(Lambda l) const
        {
            for (auto it = m_wrappedObj->begin(); it != m_wrappedObj->end(); ++it)
            {
                l(*it);
            }
        }
        template<typename Lambda>
        void foreachElIndexed(Lambda l)
        {
            std::size_t el = 0;
            for (auto it = m_wrappedObj->begin(); it != m_wrappedObj->end(); ++it)
            {
                l(el,*it);
                ++el;
            }
        }
        std::vector<Scalar> m_innerContainer;
    public:
        std::vector<Scalar>* raw()
        {
            return m_wrappedObj;
        }
        VectorWrapper():m_wrappedObj(&m_innerContainer), m_constWrappedObj(&m_innerContainer){}
        VectorWrapper(std::vector<Scalar>* wrapObj):m_wrappedObj(wrapObj),m_constWrappedObj(wrapObj){}
        VectorWrapper(std::vector<Scalar>& wrapObj) :m_wrappedObj(&wrapObj), m_constWrappedObj(&wrapObj) {}
        VectorWrapper(const std::vector<Scalar>& wrapObj) :m_wrappedObj(nullptr), m_constWrappedObj(&wrapObj) {}
        VectorWrapper(const std::vector<Scalar>* wrapObj) :m_wrappedObj(nullptr), m_constWrappedObj(wrapObj) {}
        VectorWrapper(std::size_t size, Scalar initialValue):m_constWrappedObj(&m_innerContainer)
        {
            m_innerContainer.assign(size, initialValue);
            m_wrappedObj = &m_innerContainer;
        }

        static VectorWrapper<Scalar> Constant(std::size_t size, Scalar value)
        {
            return VectorWrapper<Scalar>(size, value);
        }

        void copyFrom(const std::vector<Scalar>& data)
        {
            m_innerContainer.clear();
            std::copy(data.begin(), data.end(), std::back_inserter(m_innerContainer));
            m_constWrappedObj = &m_innerContainer;
            m_wrappedObj = &m_innerContainer;
        }
        VectorWrapper<Scalar>& operator=(const std::vector<Scalar>& other)
        {
            copyFrom(other);
            return *this;
        }
        VectorWrapper(const ConstVectorWrapper<Scalar>& copyObj) :m_constWrappedObj(&m_innerContainer)
        {
            m_innerContainer.reserve(copyObj.size());
            for(std::size_t i = 0; i < copyObj.size(); ++i)
            {
                m_innerContainer[i] = copyObj[i];
            }
            m_wrappedObj = &m_innerContainer;
        }
        std::size_t size() const
        {
            return m_wrappedObj->size();
        }

        Scalar total() const
        {
            Scalar total = 0;
            foreachEl([&total](Scalar el) {total += el; });
            return total;
        }
        Scalar maxCoeff() const
        {
            return *std::max_element(m_constWrappedObj->begin(), m_constWrappedObj->end());
        }
        Scalar minCoeff() const
        {
            return *std::min_element(m_constWrappedObj->begin(), m_constWrappedObj->end());
        }
        Scalar norm() const
        {
            return std::sqrt(std::accumulate(m_constWrappedObj->begin(), m_constWrappedObj->end(), (Scalar)0, [](Scalar acc, Scalar s1) {return acc + s1 * s1; }));
        }
        Scalar normSq() const
        {
            return std::accumulate(m_constWrappedObj->begin(), m_constWrappedObj->end(), (Scalar)0, [](Scalar acc, Scalar s1) {return acc + s1 * s1; });
        }

        Scalar operator[](std::size_t i) const
        {
            return (*m_constWrappedObj)[i];
        }

        Scalar dot(const ConstVectorWrapper<Scalar>& other) const
        {
            Scalar init = 0;
            for (std::size_t i = 0; i < m_constWrappedObj->size(); ++i)
            {
                init += (*m_constWrappedObj)[i] + other[i];
            }
            return init;
        }


        std::vector<Scalar> toStd() const
        {
            return *m_constWrappedObj;
        }

        VectorWrapper<Scalar> operator+(const VectorWrapper<Scalar>& other) const
        {
            std::vector<Scalar> ret;
            ret.reserve(m_wrappedObj->size());
            std::copy(m_wrappedObj->begin(), m_wrappedObj->end(), std::back_inserter(ret));
            // Wrap temporary, modify underlying vector
            VectorWrapper<Scalar> vw(&ret);
            vw += other;
            return vw;
        }
        VectorWrapper<Scalar> operator-(const VectorWrapper<Scalar>& other) const
        {
            std::vector<Scalar> ret;
            ret.reserve(m_wrappedObj->size());
            std::copy(m_wrappedObj->begin(), m_wrappedObj->end(), std::back_inserter(ret));
            // Wrap temporary, modify underlying vector
            VectorWrapper<Scalar> vw(&ret);
            vw -= other;
            return vw;
        }
        static std::vector<Scalar> Random(std::size_t size, Scalar min = 0, Scalar max = 1)
        {
            std::vector<Scalar> ret;
            ret.reserve(size);
            std::uniform_real_distribution<Scalar> uni(min,max);
            auto eng = Helpers::RandomHelpers::getRandomEngine();
            std::generate(ret.begin(), ret.end(), [&uni,&eng]() { return uni(eng); });
            return ret;
        }

        VectorWrapper<Scalar>& operator*=(const Scalar& s)
        {
            for(auto it = m_wrappedObj->begin(); it != m_wrappedObj->end(); ++it)
            {
                *it *= s;
            }
            return *this;
        }
        VectorWrapper<Scalar> operator*(const Scalar& s) const
        {
            VectorWrapper<Scalar> vec;
            vec.copyFrom(*m_wrappedObj);
            vec *= s;
            return vec;
        }
        VectorWrapper<Scalar>& operator+=(const Scalar& s)
        {
            foreachEl([s](Scalar& el) {el += s; });
            return *this;
        }
        VectorWrapper<Scalar>& operator+=(const VectorWrapper<Scalar>& s)
        {
            foreachElIndexed([s](std::size_t ind, Scalar& el) {el += s[ind]; });
            return *this;
        }
        VectorWrapper<Scalar>& operator-=(const VectorWrapper<Scalar>& s)
        {
            foreachElIndexed([s](std::size_t ind, Scalar& el) {el -= s[ind]; });
            return *this;
        }
        VectorWrapper<Scalar>& operator-=(const Scalar& s)
        {
            foreachEl([s](Scalar& el) {el -= s; });
            return *this;
        }
    };

    template<typename Scalar>
    inline VectorWrapper<Scalar> operator*(const Scalar& scalar, const VectorWrapper<Scalar>& vec)
    {
        VectorWrapper<Scalar> ret(vec);
        ret *= scalar;
        return ret;
    }

    template<typename Scalar>
    auto wrapVec(const std::vector<Scalar>& vec)
    {
        return VectorWrapper<Scalar>(vec);
    }
    template<typename Scalar>
    auto wrapVec(std::vector<Scalar>& vec)
    {
        return VectorWrapper<Scalar>(vec);
    }
    template<typename Scalar>
    auto wrapVec(std::vector<Scalar>* vec)
    {
        return VectorWrapper<Scalar>(vec);
    }

    template<typename VecType, typename Op>
    struct VecUnary
    {
        Op op;
        VecType vec;

        auto eval() { return op(vec); }
    };
    template<typename Scalar>
    struct Vec2
    {
        Scalar m_x, m_y;

        const Vec2& get() const
        {
            return *this;
        }
        Vec2& get()
        {
            return *this;
        }
        Scalar x() const
        {
            return m_x;
        }
        Scalar y() const
        {
            return m_y;
        }

        Vec2(Scalar x, Scalar y):m_x(x),m_y(y){}
        
        template<typename It>
        Vec2(It begin, It end): m_x(*begin),m_y(*std::next(begin)){}

        Vec2():m_x(0),m_y(0){}


        Scalar dot(const Vec2& other) const { return m_x * other.m_x + m_y * other.m_y; }
        Scalar cross(const Vec2& other) const { return m_x * other.m_y - m_y * other.m_x; }
        Scalar length() const{return std::hypot(m_x, m_y); }
        Vec2 operator-(const Vec2& other) const { return Vec2{ m_x - other.m_x, m_y - other.m_y }; }
        Vec2 operator+(const Vec2& other) const { return Vec2{ m_x + other.m_x, m_y + other.m_y }; }
        Scalar sqLength() const { return m_x * m_x + m_y * m_y; }
        Vec2 operator*(const Scalar& s) const { return Vec2{ m_x*s,m_y*s }; }
        Vec2& operator*=(const Scalar& s) { m_x *= s; m_y *= s; return *this; }
        Vec2& operator/=(const Scalar& s) { m_x /= s; m_y /= s; return *this; }
        Vec2& operator+=(const Vec2& s) { m_x += s.m_x; m_y += s.m_y; return *this; }
        Vec2& operator-=(const Vec2& s) { m_x -= s.m_x; m_y -= s.m_y; return *this; }
        Scalar operator*(const Vec2<Scalar>& s) const { return this->dot(s); }
        Vec2 operator/(const Scalar& s) const { return Vec2{ m_x/s,m_y/s }; }
        void normalize() { const auto l = length(); m_x /= l; m_y /= l; }
        Vec2 normalized() const { const auto l = length(); return Vec2{ m_x / l,m_y / l }; }
        Vec2 piOver2Rotated() const { return Vec2(m_y, -m_x); }
        Vec2 projectOn(const Vec2& p0, const Vec2& p1) const
        {
            auto diff = p1 - p0;
            auto vDiff = *this - p0;
            return p0 + diff * diff.dot(vDiff) / diff.sqLength();
        }
        Vec2 projectOn(const Vec2& p0, const Vec2& p1, Scalar& segLength) const
        {
            auto diff = p1 - p0;
            auto vDiff = *this - p0;
            segLength = diff.length();
            return p0 + diff * diff.dot(vDiff) / (segLength*segLength);
        }
        Scalar projectParameterOn(const Vec2& p0, const Vec2& p1) const
        {
            auto diff = p1 - p0;
            auto vDiff = *this - p0;
            return diff.dot(vDiff) / diff.sqLength();
        }
        static Vec2 lerp(const Vec2& p0, const Vec2& p1, Scalar frac)
        {
            return p0 + (p1 - p0) * frac;
        }
    };
    template<typename Scalar>
    Vec2< Scalar> operator*(const Scalar& s, const Vec2<Scalar>& v)
    {
        return v * s;
    }

	template<typename Scalar>
	struct Vector
	{
		using SelfType = Vector<Scalar>;
		Scalar* m_data;
		int m_len;
        bool m_ownsData = true;
		~Vector()
		{
            if(m_ownsData) delete[] m_data;
		}
		Vector():m_len(0),m_data(nullptr){}

		Vector(int size): m_data(new Scalar[size]),m_len(size){}
		Vector(const std::vector<Scalar>& values):
		m_data(new Scalar[values.size()]),
		m_len(values.size())
		{
			
			for (int i = 0; i < m_len; i++) m_data[i] = values[i];
		}

		Scalar operator[](int i) const
		{
			return m_data[i];
		}

		int size() const
		{
			return m_len;
		}

		Scalar& operator[](int i)
		{
			return m_data[i];
		}

		SelfType copy() const
		{
			SelfType copyV(m_len);
			for(int i = 0; i < m_len; i++)
			{
				copyV.m_data[i] = m_data[i];
			}
			return copyV;
		}

		/**
		 * \brief Generates a random vector by selecting a value via the uniform
		 * distribution over the interval [minVal, maxVal] per dimension
		 * \param size Number of dimensions
		 * \param minVal Minimum value
		 * \param maxVal Maximum value
		 * \return Random vector
		 */
		static Vector<Scalar> randomVector(int size, Scalar minVal, Scalar maxVal)
		{
			std::random_device r;
			std::default_random_engine e1(r());
			std::uniform_int_distribution<Scalar> uniform_dist(minVal, maxVal);

			std::vector<Scalar> vals(size, minVal);
			for(int i = 0; i < size; i++)
			{
				vals[i] = uniform_dist(e1);
			}
			return Vector<Scalar>(vals);
		}
        Vector& randomize(Scalar minVal, Scalar maxVal)
		{
			std::random_device r;
			std::default_random_engine e1(r());
			std::uniform_real_distribution<Scalar> uniform_dist(minVal, maxVal);
			for (int i = 0; i < m_len; i++)
			{
				m_data[i][i] = uniform_dist(e1);
			}
			return *this;
		}

		Vector& operator=(SelfType&& other) noexcept
		{
			if (m_data != nullptr) delete m_data;
			m_data = other.m_data;
			m_len = other.m_len;
			other.m_len = 0;
			other.m_data = nullptr;
			return *this;
		}

		Scalar dot(const Vector<Scalar>& other)
		{
			assert(other.m_len == m_len);
			Scalar total = 0;
			for (int i = 0; i < m_len; i++) total += m_data[i] + other.m_data[i];
			return total;
		}

		/**
		 * \brief Projects out the component that is parallel to the given vector
		 * \param other The vector to use to remove the projection from
		 * \return Modified vector
		 */
		SelfType& projectOut(const SelfType& other)
		{
			Scalar val = dot(other);
			*this -= other * val / other.normSq();
			return *this;
		}

		SelfType normalized() const
		{
			Scalar factor = norm();
			return *this / factor;
		}

		Scalar norm() const
		{
			return std::sqrt(dot(*this));
		}
		Scalar normSq() const
		{
			return dot(*this);
		}

		SelfType operator/(const Scalar& factor) const
		{
			SelfType c = copy();
			for (int i = 0; i < m_len; i++) c.m_data[i] /= factor;
			return c;
		}

		SelfType operator+(const SelfType& other) const
		{
			assert(other.m_len == m_len);
			SelfType newVal(m_len);
			for (int i = 0; i < m_len; i++) newVal.m_data[i] = m_data[i] + other.m_data[i];
			return newVal;
		}
		SelfType& operator+=(const SelfType& other) const
		{
			assert(other.m_len == m_len);
			for (int i = 0; i < m_len; i++) m_data[i] += other.m_data[i];
			return *this;
		}
		SelfType operator-(const SelfType& other) const
		{
			assert(other.m_len == m_len);
			SelfType newVal(m_len);
			for (int i = 0; i < m_len; i++) newVal.m_data[i] = m_data[i] - other.m_data[i];
			return newVal;
		}
		SelfType& operator-=(const SelfType& other) const
		{
			assert(other.m_len == m_len);
			for (int i = 0; i < m_len; i++) m_data[i] -= other.m_data[i];
			return *this;
		}
		SelfType operator*(const Scalar& val) const
		{
			SelfType newVal(m_len);
			for (int i = 0; i < m_len; i++) newVal.m_data[i] = m_data[i] * val;
			return newVal;
		}
	};

    template<typename Scalar>
    struct Rect
    {
        Vec2<Scalar> m_minPos, m_maxPos;

        bool isEmpty() const { auto diff = m_maxPos - m_minPos; return diff.x() < 0 || diff.y < 0; }
        Rect() :m_minPos(std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max()),
            m_maxPos(-std::numeric_limits<Scalar>::max(), -std::numeric_limits<Scalar>::max())
        {}
        Scalar width() const
        {
            return m_maxPos.x() - m_minPos.x();
        }
        Scalar height() const
        {
            return m_maxPos.y() - m_minPos.y();
        }
        bool contains(const Vec2<Scalar>& pnt) const
        {
            return (pnt.x() >= m_minPos.x() && pnt.x() <= m_maxPos.x()) &&
                (pnt.y() >= m_minPos.y() && pnt.y() <= m_maxPos.y());
        }
        Rect& fitTo(const Vec2<Scalar>& pnt)
        {
            m_minPos.m_x = std::min(m_minPos.m_x, pnt.x());
            m_minPos.m_y = std::min(m_minPos.m_y, pnt.y());
            m_maxPos.m_x = std::min(m_maxPos.m_x, pnt.x());
            m_maxPos.m_y = std::min(m_maxPos.m_y, pnt.y());
            return *this;
        }
        Rect& fitTo(const Rect& other)
        {
            fitTo(other.m_minPos);
            fitTo(other.m_maxPos);
            return *this;
        }
    };

    // Row major matrix, right acting
    template<typename Scalar>
    struct Matrix3
    {
        Scalar m_data[9] = { 0,0,0,0,0,0,0,0,0 };

        Matrix3()
        {
            (*this)(0,0) = 1;
            (*this)(1, 1) = 1;
            (*this)(2, 2) = 1;

        }
        Scalar& operator()(int r, int c)
        {
            assert(r >= 0 && c >= 0 && r < 3 && c < 3);
            return m_data[r * 3 + c];
        }
        const Scalar& operator()(int r, int c) const
        {
            assert(r >= 0 && c >= 0 && r < 3 && c < 3);
            return m_data[r * 3 + c];
        }
        // Affine transform only
        Vec2<Scalar> operator*(const Vec2<Scalar>& vec)
        {
            auto& self = *this;
            return Vec2 < Scalar>(
                self(0,0) * vec.x() + self(0, 1) * vec.y() + self(0,2),
                self(1, 0) * vec.x() + self(1, 1) * vec.y() + self(1, 2)
            );
        }
    };

}
#endif