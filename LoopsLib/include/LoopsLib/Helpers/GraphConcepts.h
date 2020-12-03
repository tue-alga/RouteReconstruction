#ifndef GRAPH_CONCEPTS_H
#define GRAPH_CONCEPTS_H
#include <vector>
#include <functional>


template<typename ValueType>
struct IteratorWrapper
{
    virtual ~IteratorWrapper() = default;
    virtual bool atEnd() = 0;
    virtual bool next(ValueType& out) = 0;
};

template<typename ValueType, typename State>
struct LambdaIteratorWrapper : public IteratorWrapper<ValueType>
{
    State m_state;
    std::function<bool(State&)> m_endCheck;
    std::function<bool(State&, ValueType&)> m_next;

    LambdaIteratorWrapper(State initial,std::function<bool(State&)> endCheck, std::function<bool(State&, ValueType&)> next):
    m_state(initial), m_endCheck(endCheck), m_next(next){}

    bool atEnd() override
    {
        return m_endCheck(m_state);
    }

    bool next(ValueType& out) override
    {
        return m_next(m_state, out);
    }
};

template<typename ValueType, typename State>
class IterableCollection
{
    State m_state;
public:

};

template<typename ValueType,typename Converter>
class ConvertedIterable
{
	class iterator;
	friend class ConvertedIterable<ValueType,Converter>::iterator;
	std::vector<ValueType>& m_toIterate;
	Converter m_conv;
public:
	using OutputType = decltype(m_conv.operator()(std::declval<ValueType>()));
	ConvertedIterable(std::vector<ValueType>& toIterate, Converter conv) : m_toIterate(toIterate),m_conv(conv)
	{}
	class iterator
	{
		friend class ConvertedIterable;
		int m_index;
		ConvertedIterable<ValueType, Converter>* m_parent;
		iterator(ConvertedIterable<ValueType,Converter>* parent, int index):m_index(index), m_parent(parent){}
	public:
		OutputType operator*(){
			return m_parent->m_conv(m_parent->m_toIterate[m_index]);
		}
		iterator& operator++()
		{
			++m_index;
			return *this;
		}
		bool operator!=(const iterator& other) const
		{
			return m_index == other.m_index;
		}
	};
	iterator begin()
	{
		return iterator(this, 0);
	}
	iterator end()
	{
		return iterator(this, m_toIterate.size());
	}
};
#endif