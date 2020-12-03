#ifndef HELPERS_REDUCE_H
#define HELPERS_REDUCE_H
#include <vector>

namespace LoopsLib::Helpers{

    template<typename ValueType>
    ValueType norm(const std::vector<ValueType>& val)
    {
        return std::sqrt(std::accumulate(val.begin(), val.end(), 0, [](ValueType acc, ValueType v0) {return acc + v0 * v0; }));
    }
    template<typename ValueType>
    ValueType minCoeff(const std::vector<ValueType>& val)
    {
        return *std::min_element(val.begin(), val.end());
    }
    template<typename ValueType>
    ValueType maxCoeff(const std::vector<ValueType>& val)
    {
        return *std::max_element(val.begin(), val.end());
    }

	template<typename In, typename Out, typename Proc>
	Out reduce(const std::vector<In>& v, Proc processor, Out init)
	{
		Out o = init;
		for (int i = 0; i < v.size(); i++)
		{
			processor(v[i], i, o);
		}
		return o;
	}

	template<typename In>
	int argmax(const std::vector<In>& v)
	{
		In m = v[0];
		int ind = 0;
		for(int i = 1; i < v.size(); i++)
		{
			if(v[i] > m)
			{
				m = v[i];
				ind = i;
			}
		}
		return ind;
	}
	/**
	 * \brief 
	 * \tparam In Input type
	 * \tparam Comparison Comparison object type
	 * \param v The vector of values
	 * \param compare Callable object representing the 'less than' operator.
	 * \return The index of the maximum value
	 */
	template<typename In, typename Comparison>
	int argmax(const std::vector<In>& v, Comparison compare)
	{
		int ind = 0;
		for (int i = 1; i < v.size(); i++)
		{
			if (compare(v[ind],v[i]))
			{
				ind = i;
			}
		}
		return ind;
	}
	/**
	 * \brief
	 * \tparam In Input type
	 * \tparam Comparison Comparison object type
	 * \param v The vector of values
	 * \param compare Callable object representing the 'less than' operator.
	 * \return The index of the minimum value
	 */
	template<typename In, typename Comparison>
	int argmin(const std::vector<In>& v, Comparison compare)
	{
		int ind = 0;
		for (int i = 1; i < v.size(); i++)
		{
			if (compare(v[i], v[ind]))
			{
				ind = i;
			}
		}
		return ind;
	}
}
#endif
