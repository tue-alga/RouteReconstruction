#ifndef GUIMESSAGES_BYTESBUFFER_H
#define GUIMESSAGES_BYTESBUFFER_H
#include <tuple>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <set>
#include <cassert>
#include <LoopsLib/Algs/Types.h>

namespace GuiMessages
{

    struct BytesRange
    {
        char* m_begin = nullptr;
        std::size_t m_capacity = 0;
        // Number of filled bytes
        std::size_t m_filled = 0;
        // Is the byte buffer owned by this object
        bool m_isOwned = false;

        ~BytesRange();

        void cleanup();

        /**
         * \brief Create an owned buffer of the given length
         * \param len The length
         */
        void create(std::size_t len);

        std::pair<char*, std::size_t> toPair() const;

        void copyFrom(void* src);

        template<typename T>
        void copyFrom(const T* src)
        {
            if (!m_isOwned) throw std::runtime_error("Copying into non-owned bytes range");
            if(m_capacity < sizeof(T)) throw std::runtime_error("Buffer too small");
            memcpy(m_begin, src, sizeof(T));
            m_filled = sizeof(T);
        }

        void copyTo(void* dest) const;
    };

    template<typename T>
    void to_binary(const T& t, BytesRange& range)
    {
        range.copyFrom(&t);
    }
    template<typename T>
    void from_binary(const BytesRange& data, T& t)
    {
        if(data.m_filled != sizeof(T))
        {
            throw std::runtime_error("Difference in bytes for from_binary: expected " + std::to_string(sizeof(T)) + " vs " + std::to_string(data.m_filled));
        }
        assert(data.m_filled == sizeof(T));
        data.copyTo(&t);
    }


    class BytesBuffer
    {
        char* m_data;
        // Current capacity
        std::size_t m_capacity;
        // Location to write next byte
        std::size_t m_head;
        // Location to read next byte
        std::size_t m_readHead;
        // Buffer for intermediate parsing.
        BytesRange m_scratchBuffer;
    private:
        void tryAllocate(std::size_t toAdd);
    public:

        BytesBuffer(std::size_t len = 64);

        ~BytesBuffer();

        /**
         * \brief Total size of filled buffer in bytes
         * \return The number of filled bytes
         */
        std::size_t size() const;

        /**
         * \brief Remaining bytes to read
         * \return The number of bytes to read remaining
         */
        std::size_t readSize() const;

        std::size_t capacity() const;

        BytesRange readableRange() const;

        BytesRange& scratchBuf();

        void seek(std::size_t pos);

        std::size_t tell() const;

        void copyRaw(const char* data, std::size_t len);

        void writeRawAt(std::size_t offset, void* data, std::size_t len);

        BytesRange getDataAndProgress(std::size_t len);

        /**
         * \brief Resets the read head of the buffer to the start
         */
        void resetReadHead();

        void copyRaw(const BytesRange& data);

        template<typename T>
        bool canReadSimpleType() const
        {
            return m_head >= sizeof(T);
        }

        bool canReadBytes(std::size_t len) const;
        template<typename T>
        friend BytesBuffer& operator>>(BytesBuffer& buff, T& arg);
        template<typename T>
        friend BytesBuffer& operator<<(BytesBuffer& buff, const T& arg);
    };

    // Specializations/overloads

    template<typename T>
    BytesBuffer& operator>>(BytesBuffer& buff, T& arg)
    {
        auto data = buff.getDataAndProgress(sizeof(T));
        from_binary(data, arg);
        return buff;
    }
    template<typename T>
    BytesBuffer& operator<<(BytesBuffer& buff, const T& arg)
    {
        static_assert(sizeof(T) < 64);
        to_binary(arg, buff.scratchBuf());
        buff.copyRaw(buff.scratchBuf());
        return buff;
    }
    template<>
    inline BytesBuffer& operator<<<std::pair<char*, std::size_t>>(BytesBuffer& buff, const std::pair<char*, std::size_t>& data)
    {
        buff.copyRaw(data.first, data.second);
        return buff;
    }
    template<typename T>
    inline BytesBuffer& operator<<(BytesBuffer& buff, const std::vector<T>& data)
    {
        buff << data.size();
        for (const auto& el : data)
        {
            buff << el;
        }
        return buff;
    }
    template<typename T>
    inline BytesBuffer& operator>>(BytesBuffer& buff, std::vector<T>& data)
    {
        std::size_t size;
        buff >> size;
        data.reserve(size);
        for(auto i = 0; i < size; ++i)
        {
            data.push_back({});
            buff >> data.back();
        }
        return buff;
    }
    template<>
    inline BytesBuffer& operator<<<std::string>(BytesBuffer& buff, const std::string& data)
    {
        buff << data.size();
        buff.copyRaw(data.data(), data.size());
        return buff;
    }
    template<>
    inline BytesBuffer& operator>><std::string>(BytesBuffer& buff, std::string& data)
    {
        std::size_t size;
        buff >> size;
        if(size > 0)
        {
            auto range = buff.getDataAndProgress(size);
            // Should copy
            data = std::string(range.m_begin, range.m_filled);
        }
        return buff;
    }

    template<typename U, typename V>
    inline BytesBuffer& operator<<(BytesBuffer& buff, const std::map<U, V>& data)
    {
        buff << data.size();
        for(const auto& el: data)
        {
            buff << el.first << el.second;
        }
        return buff;
    }
    template<typename U, typename V>
    inline BytesBuffer& operator>>(BytesBuffer& buff, std::map<U, V>& data)
    {
        std::size_t size;
        buff >> size;
        for(auto i = 0; i < size; ++i)
        {
            U u;
            V v;
            buff >> u >> v;
            data[u] = v;
        }
        return buff;
    }
    template<typename U, typename V>
    inline BytesBuffer& operator<<(BytesBuffer& buff, const std::unordered_map<U, V>& data)
    {
        buff << data.size();
        buff.copyRaw(data.data(), data.size());
        return buff;
    }
    template<typename U, typename V>
    inline BytesBuffer& operator>>(BytesBuffer& buff, std::unordered_map<U, V>& data)
    {
        std::size_t size;
        buff >> size;
        for (auto i = 0; i < size; ++i)
        {
            U u;
            V v;
            buff >> u >> v;
            data[u] = v;
        }
        return buff;
    }
    template<typename U>
    inline BytesBuffer& operator<<(BytesBuffer& buff, const std::set<U>& data)
    {
        buff << data.size();
        for (const auto& el : data)
        {
            buff << el;
        }
        return buff;
    }
    template<typename U>
    inline BytesBuffer& operator>>(BytesBuffer& buff, std::set<U>& data)
    {
        std::size_t size;
        buff >> size;
        for (auto i = 0; i < size; ++i)
        {
            U u;
            buff >> u;
            data.insert(u);
        }
        return buff;
    }
    template<typename U>
    inline BytesBuffer& operator<<(BytesBuffer& buff, const std::unordered_set<U>& data)
    {
        buff << data.size();
        for (const auto& el : data)
        {
            buff << el;
        }
        return buff;
    }
    template<typename U>
    inline BytesBuffer& operator>>(BytesBuffer& buff, std::unordered_set<U>& data)
    {
        std::size_t size;
        buff >> size;
        for (auto i = 0; i < size; ++i)
        {
            U u;
            buff >> u;
            data.insert(u);
        }
        return buff;
    }
    // LoopsLib specific
    template<typename U, typename V>
    inline BytesBuffer& operator>>(BytesBuffer& buff, std::pair<U,V>& pnt)
    {
        buff >> pnt.first;
        buff >> pnt.second;
        return buff;
    }
    template<typename U, typename V>
    inline BytesBuffer& operator<<(BytesBuffer& buff, const std::pair<U, V>& pnt)
    {
        buff << pnt.first;
        buff << pnt.second;
        return buff;
    }

    // LoopsLib specific
    template<>
    inline BytesBuffer& operator>><LoopsLib::MovetkGeometryKernel::MovetkPoint>(BytesBuffer& buff, LoopsLib::MovetkGeometryKernel::MovetkPoint& pnt)
    {
        buff >> pnt.m_x;
        buff >> pnt.m_y;
        return buff;
    }
    template<>
    inline BytesBuffer& operator<<<LoopsLib::MovetkGeometryKernel::MovetkPoint>(BytesBuffer& buff, const LoopsLib::MovetkGeometryKernel::MovetkPoint& pnt)
    {
        buff << pnt.m_x;
        buff << pnt.m_y;
        return buff;
    }
}
#endif