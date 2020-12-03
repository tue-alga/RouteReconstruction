#ifndef LOOPSIO_READBUFFER_H
#define LOOPSIO_READBUFFER_H
#include <string_view>
#ifdef WIN32
#include <charconv>
#else
#include <sstream>
namespace std
{
    struct fromCharsResult
    {
        std::errc ec = std::errc::argument_list_too_long;
    };
    template<typename T>
    fromCharsResult from_chars(const char* begin, const char* end, T& target)
    {
        std::stringstream ss;
        ss.str(std::string(begin, (end - begin) / sizeof(char)));
        ss >> target;
        return fromCharsResult{};
    }
}
#endif

namespace LoopsIO::Helpers
{

    class ReadBuffer
    {
        // The buffer data
        char* m_data;
        // Capacity of the buffer
        std::size_t m_capacity;
        // Current head of the buffer, the location to insert the next character
        std::size_t m_head = 0;
        // The current readhead, the index of the next character to read.
        std::size_t m_readHead = 0;
        bool m_lastReadSuccess = true;
    public:
        ReadBuffer(std::size_t capacity);

        /**
         * \brief Returns the character relative to the current read location
         * \param loc The location of the character
         * \return The character
         */
        char charAt(std::size_t loc) const;

        template<typename Func>
        std::size_t foreachChar(Func f)
        {
            auto curr = m_readHead;
            for (; curr < m_head; ++curr)
            {
                if (f(m_data[curr], curr)) break;
            }
            return curr;
        }

        /**
         * \brief Reallocates the buffer to be able to hold newCapacity number of characters.
         * Any present characters will be retained.
         * \param newCapacity The new capacity.
         */
        void reallocate(std::size_t newCapacity);

        /**
         * \brief Returns whether the buffer contains no readable characters.
         * \return Is the buffer empty
         */
        bool empty() const;

        template<typename DelimFunc>
        std::string_view substr(DelimFunc fun, bool extract = false)
        {
            auto curr = m_readHead;
            for (; curr < m_head; ++curr)
            {
                if (fun(m_data[curr])) break;
            }
            if (curr == m_head) return {};
            auto size = curr - m_readHead;
            auto start = m_readHead;
            if (extract) m_readHead = curr;
            return std::string_view(m_data + start, size);

        }

        std::string_view substr(std::size_t length) const;

        std::string_view substr(char delim, bool extract = false);

        /**
         * \brief Advances the read pointer by the given number of locations
         * \param num Number of locations to advance
         */
        void advance(std::size_t num = 1);

        /**
         * \brief Clears the buffer
         */
        void clear();

        /**
         * \brief Copies all readable characters to the beginning of the buffer
         */
        void refit();

        template<typename T, typename DelimFunc>
        ReadBuffer& read(T& target, DelimFunc fn)
        {
            m_lastReadSuccess = false;
            std::size_t curr = m_readHead;
            for (; curr < m_head; ++curr)
            {
                if (fn(m_data[curr])) break;
            }
            if (curr == m_head)
            {
                return *this;
            }
            m_lastReadSuccess = true;
            auto res = std::from_chars(m_data + m_readHead, m_data + curr, target);
            if (res.ec == std::errc::invalid_argument)
            {
                m_lastReadSuccess = false;
            }
            else
            {
                m_readHead = curr;
            }
            return *this;
        }

        bool skipJustOver(char stopChar);

        bool skipTo(char stopChar);

        /**
         * \brief Returns whether the buffer contains the given character
         * \param character The character
         * \param offset Offset relative to read position to start the search
         * \return Does the buffer contain the character
         */
        bool contains(char character, std::size_t offset = 0) const;

        bool readChunk(std::istream& stream, std::size_t size = 0);

        void seek(std::size_t pos);

        std::size_t tell() const;

        std::size_t size() const;

        std::size_t capacity() const;

        ReadBuffer& skipWs();

        ~ReadBuffer();
    };
}
#endif