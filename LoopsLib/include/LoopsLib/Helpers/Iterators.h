#ifndef HELPERS_ITERATORS_H
#define HELPERS_ITERATORS_H
#include <type_traits>
namespace LoopsLib::Helpers::Iterators
{
#define LOOPS_COPY_IT_TYPES(typeName ) \
    using difference_type = typename typeName::difference_type;\
    using value_type = typename typeName::value_type;\
    using reference = typename typeName::reference;\
    using pointer = typename typeName::pointer;\
    using iterator_category = typename typeName::iterator_category;
#define LOOPS_COPY_IT_TYPES_EXCEPT_VALUES(typeName ) \
    using difference_type = typename typeName::difference_type;\
    using iterator_category = typename typeName::iterator_category;

    template<typename It, typename EndIt>
    struct PairIterable
    {
        std::pair<It, EndIt> m_iters;
        PairIterable(const std::pair<It, EndIt>& iters):m_iters(iters){}
        It begin()
        {
            return m_iters.first;
        }
        EndIt end()
        {
            return m_iters.second;
        }
    };

    namespace detail
    {
        //Src: https://stackoverflow.com/questions/7943525/is-it-possible-to-figure-out-the-parameter-type-and-return-type-of-a-lambda/7943765
        template <typename T>
        struct function_traits
            : public function_traits<decltype(&T::operator())>
        {};
        // For generic types, directly use the result of the signature of its 'operator()'

        template <typename ClassType, typename ReturnType, typename... Args>
        struct function_traits<ReturnType(ClassType::*)(Args...) const>
            // we specialize for pointers to member function
        {
            static constexpr std::size_t arity = sizeof...(Args);
            //enum { arity = sizeof...(Args) };
            // arity is the number of arguments.

            using result_type = ReturnType;

            template <size_t i>
            struct arg
            {
                using type = typename std::tuple_element<i, std::tuple<Args...>>::type ;
                // the i-th argument is equivalent to the i-th tuple element of a tuple
                // composed of those arguments.
            };
        };
        template <typename ClassType, typename ReturnType, typename... Args>
        struct function_traits<ReturnType(ClassType::*)(Args...)>
            // we specialize for pointers to member function
        {
            static constexpr std::size_t arity = sizeof...(Args);
            //enum { arity = sizeof...(Args) };
            // arity is the number of arguments.

            using result_type = ReturnType;

            template <size_t i>
            struct arg
            {
                using type = typename std::tuple_element<i, std::tuple<Args...>>::type;
                // the i-th argument is equivalent to the i-th tuple element of a tuple
                // composed of those arguments.
            };
        };
    }

    template<typename BaseType>
    struct filter_end_iterator
    {
        using internal_it = decltype(std::declval<const BaseType&>().begin());
        internal_it m_currentIt;
        filter_end_iterator(const BaseType& container):m_currentIt(container.end()){}
    };
    template<typename BaseType,typename Lambda>
    struct filter_iterator
    {
        const BaseType& m_container;
        using internal_it = decltype(std::declval<const BaseType&>().begin());
        std::size_t m_current = 0;
        internal_it m_currentIt;
        Lambda m_filter;

        filter_iterator(const BaseType& container, Lambda filter):
        m_container(container),
        m_filter(filter),
        m_currentIt(container.begin())
        {
            while (m_currentIt != m_container.end() && !m_filter(m_current, *m_currentIt))
            {
                ++m_current;
                ++m_currentIt;
            }
        }

        filter_iterator associatedEnd()
        {
            filter_iterator it(m_container, m_filter);
            it.m_currentIt = m_container.end();
            return it;
        }

        static filter_iterator end(const BaseType& container)
        {
            filter_iterator it(container);
            it.m_current = container.end();
            return it;
        }

        auto operator*()const
        {
            return *m_currentIt;
        }

        bool operator!=(const filter_iterator& other) const
        {
            return m_currentIt != other.m_currentIt;
        }
        bool operator!=(const filter_end_iterator<BaseType>& other) const
        {
            return m_currentIt != other.m_currentIt;
        }

        filter_iterator& operator++()
        {
            ++m_current;
            ++m_currentIt;
            while(m_currentIt != m_container.end() && !m_filter(m_current, *m_currentIt))
            {
                ++m_current;
                ++m_currentIt;
            }
            return *this;
        }
    };

    template<typename Container>
    struct circular_it
    {
        using InnerIt = decltype(std::declval<Container&>().begin());
        InnerIt m_it;
        InnerIt m_begin;
        InnerIt m_end;
        InnerIt m_start;
        std::size_t m_circCount = 0;
        using difference_type = typename InnerIt::difference_type;
        using value_type = typename InnerIt::value_type;
        using reference = typename InnerIt::reference;
        using pointer = typename InnerIt::pointer;
        using iterator_category = typename InnerIt::iterator_category;
    public:
        circular_it(Container& c, InnerIt start, std::size_t cycleCount = 0): m_it(start),m_start(start),m_begin(c.begin()),m_end(c.end()),m_circCount(cycleCount){}
        circular_it& operator++()
        {
            ++m_it;
            if (m_it == m_end) m_it = m_begin;
            if (m_it == m_start) m_circCount++;
            return *this;
        }
        difference_type operator-(const circular_it& other) const
        {
            auto diff = m_end - m_it + m_circCount*(m_end-m_begin);
            auto diffOther = other.m_end - other.m_it + other.m_circCount * (other.m_end - other.m_begin);
            return diffOther - diff;

        }
        bool operator==(const circular_it& other) const
        {
            return m_it == other.m_it && m_circCount == other.m_circCount;
        }
        bool operator!=(const circular_it& other) const
        {
            return m_it != other.m_it || m_circCount != other.m_circCount;
        }
        auto operator*()
        {
            return *m_it;
        }
        auto operator->()
        {
            return m_it.operator->();
        }
    };
    template<typename Container>
    struct indexed_iterator
    {
        using InnerIt = decltype(std::declval<Container&>().begin());
        InnerIt m_it;
        using difference_type = typename InnerIt::difference_type;
        using value_type = typename InnerIt::value_type;
        using reference = typename InnerIt::reference;
        using pointer = typename InnerIt::pointer;
        using iterator_category = typename InnerIt::iterator_category;
    public:
        indexed_iterator(Container& c, std::size_t index) : m_it(index == c.size() ? c.end(): c.begin() + index) {}
        indexed_iterator& operator++()
        {
            ++m_it;
            return *this;
        }
        bool operator==(const indexed_iterator& other) const
        {
            return m_it == other.m_it;
        }
        bool operator!=(const indexed_iterator& other) const
        {
            return m_it != other.m_it;
        }
        auto operator*()
        {
            return *m_it;
        }
        auto operator->()
        {
            return m_it.operator->();
        }
    };
    template<typename ValueType>
    struct value_iterator
    {
        using difference_type = std::size_t;
        using value_type = ValueType;
        using reference = ValueType&;
        using pointer = ValueType*;
        using iterator_category = std::forward_iterator_tag;
        ValueType m_value, m_step;
    public:
        value_iterator(ValueType value, ValueType step = 1.0) : m_value(value),m_step(step) {}
        value_iterator& operator++()
        {
            m_value += m_step;
            return *this;
        }
        bool operator!=(const value_iterator& other) const
        {
            if (other.m_value > m_value) {
                return other.m_value - m_value >= m_step;
            }
            return m_value - other.m_value >= m_step;
        }
        auto operator*()
        {
            return m_value;
        }
    };

    template<typename It1, typename It2>
    class consecutive_iterator
    {
    public:
        using difference_type = std::size_t;
        using value_type = std::remove_cv_t<std::remove_pointer_t<std::decay_t<decltype(*std::declval<It1>())>>>;
        using reference = value_type& ;
        using pointer = value_type * ;
        using iterator_category = std::forward_iterator_tag;
        using inner_it1 = It1;
        using inner_it2 = It2;
    private:
        It1 m_start1, m_end1;
        It2 m_start2, m_end2;
    public:
        consecutive_iterator(){}
        consecutive_iterator(It1 start1, It1 end1, It2 start2, It2 end2) :
            m_start1(start1),
            m_end1(end1),
            m_start2(start2),
            m_end2(end2)
        {}
        consecutive_iterator(std::pair<It1,It1> range1, std::pair<It2, It2> range2) :
            m_start1(range1.first),
            m_end1(range1.second),
            m_start2(range2.first),
            m_end2(range2.second)
        {}
        // Associated end iterator
        consecutive_iterator end() const
        {
            return consecutive_iterator(m_end1, m_end1, m_end2, m_end2);
        }
        consecutive_iterator& operator++()
        {
            if(m_start1 == m_end1)
            {
                ++m_start2;
            }
            else
            {
                ++m_start1;
            }
            return *this;
        }
        bool operator!=(const consecutive_iterator& other) const
        {
            return m_start1 != other.m_start1 || m_start2 != other.m_start2;
        }
        auto operator*()
        {
            if (m_start1 == m_end1) return *m_start2;
            return *m_start1;
        }
    };

    template<typename Iterator>
    struct Iterable
    {
        Iterator m_start, m_end;
    public:
        Iterable(Iterator start, Iterator end):m_start(start),m_end(end){}
        Iterator begin()
        {
            return m_start;
        }
        Iterator end()
        {
            return m_end;
        }
    };
    template<typename Container>
    struct IndexedIterable
    {
        indexed_iterator<Container> m_start, m_end;
    public:
        IndexedIterable(Container& c, std::size_t start, std::size_t end,bool inclusive=false) :
        m_start(indexed_iterator<Container>(c,start)), 
        m_end(indexed_iterator<Container>(c, inclusive ? end + 1 : end)) 
        {}
        indexed_iterator<Container> begin()
        {
            return m_start;
        }
        indexed_iterator<Container> end()
        {
            return m_end;
        }
    };

    template<typename ValueType>
    struct ValueIterable
    {
        value_iterator<ValueType> m_start, m_end;
    public:
        ValueIterable(ValueType start, ValueType end, bool isInclusive = false, ValueType step = 1) :
            m_start(value_iterator<ValueType>(start,step)),
            m_end(value_iterator<ValueType>(end + (isInclusive ? step : 0),step))
        {}
        ValueIterable(std::pair<ValueType,ValueType> startEnd, bool isInclusive=false, ValueType step = 1) :
            m_start(value_iterator<ValueType>(startEnd.first, step)),
            m_end(value_iterator<ValueType>(startEnd.second +(isInclusive? step : 0), step))
        {}
        value_iterator<ValueType>  begin()
        {
            return m_start;
        }
        value_iterator<ValueType>  end()
        {
            return m_end;
        }
    };

    template<typename Container, typename Func>
    class transform_iterator
    {
    public:
        using CIt = decltype(std::declval<const Container&>().begin());
        using OutType = std::decay_t<typename detail::function_traits<Func>::result_type>;
    private:
        CIt m_curr;
        CIt m_end;
        Func m_func;
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = OutType;
        using difference_type = typename CIt::difference_type;
        using pointer = OutType * ;
        using reference = OutType & ;

        transform_iterator(CIt it, Func f): m_curr(it),m_end(it),m_func(f){}
        transform_iterator(const Container& c, Func func) : m_curr(c.begin()), m_end(c.end()),m_func(func) {}

        transform_iterator associatedEnd() const
        {
            transform_iterator<Container, Func> tIt(m_end,m_func);
            return tIt;
        }

        value_type operator*() const
        {
            return m_func(*m_curr);
        }
        transform_iterator& operator++()
        {
            ++m_curr;
            return *this;
        }
        transform_iterator& operator+=(int i)
        {
            m_curr += i;
            return *this;
        }
        bool operator!=(const transform_iterator& other) const
        {
            return m_curr != other.m_curr;
        }
        bool operator==(const transform_iterator& other) const
        {
            return m_curr == other.m_curr;
        }
    };

#define COPY_TYPE_DEF(targetType, typeName) using typeName = typename targetType::typeName

    template<typename Container, typename Func>
    class transformed_back_inserter_iterator
    {
        using back_insert_iterator_t = decltype(std::back_inserter(std::declval<Container&>()));
        back_insert_iterator_t m_it;
        Func m_func;
    public:
        COPY_TYPE_DEF(back_insert_iterator_t, iterator_category);
        COPY_TYPE_DEF(back_insert_iterator_t, value_type);
        COPY_TYPE_DEF(back_insert_iterator_t, difference_type);
        COPY_TYPE_DEF(back_insert_iterator_t, pointer);
        COPY_TYPE_DEF(back_insert_iterator_t, reference);
        COPY_TYPE_DEF(back_insert_iterator_t, container_type);

        using InputType = std::decay_t<typename detail::function_traits<Func>::template arg<0>::type>;
        using OutType = std::decay_t<typename detail::function_traits<Func>::result_type>;

        static_assert(std::is_convertible_v<OutType, std::decay_t<decltype(std::declval<Container&>()[0])>>,"Argument of lambda not same as ");

        transformed_back_inserter_iterator(Container& c, Func func): m_it(std::back_inserter(c)), m_func(func){}

        transformed_back_inserter_iterator& operator*()
        {
            return *this;
        }
        transformed_back_inserter_iterator& operator++()
        {
            return *this;
        }
        transformed_back_inserter_iterator& operator++(int i)
        {
            return *this;
        }
        transformed_back_inserter_iterator& operator=(const InputType& in)
        {
            m_it = m_func(in);
            return *this;
        }
    };

}
#endif