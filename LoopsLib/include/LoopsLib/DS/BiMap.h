#ifndef DS_BIMAP_H
#define DS_BIMAP_H
#include <map>
#include <unordered_map>
namespace LoopsLib::DS
{
    template<typename U, typename V, template<typename S,typename T> typename MapContainer>
    class BaseBiMap
    {
        MapContainer<U, V> m_forward;
        MapContainer<V, U> m_backward;
    public:
        BaseBiMap(){}
        BaseBiMap(const std::initializer_list<std::pair<U,V>>& initValues)
        {
            for(const auto& el : initValues)
            {
                insert(el.first, el.second);
            }
        }
        std::size_t size() const
        {
            return m_forward.size();
        }
        void insert(const U& u, const V& v)
        {
            m_forward[u] = v;
            m_backward[v] = u;
        }
        V forward(const U& u) const
        {
            return m_forward.at(u);
        }
        U backward(const V& v) const
        {
            return m_backward.at(v);
        }
        auto forwardBegin()
        {
            return m_forward.begin();
        }
        auto forwardBegin() const
        {
            return m_forward.begin();
        }
        auto forwardEnd()
        {
            return m_forward.end();
        }
        auto forwardEnd() const
        {
            return m_forward.end();
        }
        auto backwardBegin()
        {
            return m_backward.begin();
        }
        auto backwardBegin() const
        {
            return m_backward.begin();
        }
        auto backwardEnd()
        {
            return m_backward.end();
        }
        auto backwardEnd() const
        {
            return m_backward.end();
        }
        void erase(const U& u, const V& v)
        {
            m_forward.erase(u);
            m_backward.erase(v);
        }
        void eraseViaForward(const U& u)
        {
            auto v = m_forward[u];
            m_forward.erase(u);
            m_backward.erase(v);
        }
        void eraseViaBackward(const V& v)
        {
            auto u = m_backward[v];
            m_forward.erase(u);
            m_backward.erase(v);
        }
        bool containsForward(const U& u) const
        {
            return m_forward.find(u) != m_forward.end();
        }
        bool containsBackward(const V& v) const
        {
            return m_backward.find(v) != m_backward.end();
        }
    };
    namespace detail
    {
        template<typename U, typename V>
        using UMap = std::unordered_map < U, V>;
        template<typename U, typename V>
        using Map = std::map< U, V>;
    }
    template<typename U, typename V>
    using BiUnorderedMap = BaseBiMap<U, V, detail::UMap>;

    template<typename U, typename V>
    using BiMap = BaseBiMap<U, V, detail::Map>;

}
#endif