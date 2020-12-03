#include <LoopsIO/Helpers/ReadBuffer.h>
#include <cassert>
#include <algorithm>
#include <istream>
#include <cctype>
#include <locale>
#include <cstring>

LoopsIO::Helpers::ReadBuffer::ReadBuffer(std::size_t capacity): m_data(new char[capacity]), m_capacity(capacity)
{
}

char LoopsIO::Helpers::ReadBuffer::charAt(std::size_t loc) const
{
    return m_data[m_readHead + loc];
}

void LoopsIO::Helpers::ReadBuffer::reallocate(std::size_t newCapacity)
{
    char* newData = new char[newCapacity];
    std::memmove(newData, m_data + m_readHead, size());
    delete[] m_data;
    m_data = newData;
    m_head = size();
    m_readHead = 0;
    m_capacity = newCapacity;
}

bool LoopsIO::Helpers::ReadBuffer::empty() const
{
    return m_head == m_readHead;
}

std::string_view LoopsIO::Helpers::ReadBuffer::substr(std::size_t length) const
{
    assert(m_readHead + length <= m_head);
    return std::string_view(m_data + m_readHead, length);
}

std::string_view LoopsIO::Helpers::ReadBuffer::substr(char delim, bool extract)
{
    auto curr = m_readHead;
    for (; curr < m_head; ++curr)
    {
        if (m_data[curr] == delim) break;
    }
    if (curr == m_head) return {};
    auto size = curr - m_readHead;
    auto start = m_readHead;
    if (extract) m_readHead = curr;
    return std::string_view(m_data + start, size);
}

void LoopsIO::Helpers::ReadBuffer::advance(std::size_t num)
{
    m_readHead += num;
    if (m_readHead > m_head) m_readHead = m_head;
}

void LoopsIO::Helpers::ReadBuffer::clear()
{
    m_head = 0;
    m_readHead = 0;
}

void LoopsIO::Helpers::ReadBuffer::refit()
{
    std::memmove(m_data, m_data + m_readHead, m_head - m_readHead);
    m_head -= m_readHead;
    m_readHead = 0;
}

bool LoopsIO::Helpers::ReadBuffer::skipJustOver(char stopChar)
{
    std::size_t curr = foreachChar([&curr, stopChar](char c, std::size_t head)
    {
        curr = head;
        return c == stopChar;
    });
    if (curr != m_head)
    {
        m_readHead = curr + 1;
        m_readHead = std::min(m_readHead, m_head);
        return true;
    }
    return false;
}

bool LoopsIO::Helpers::ReadBuffer::skipTo(char stopChar)
{
    std::size_t curr = foreachChar([stopChar](char c, std::size_t head)
    {
        return c == stopChar;
    });
    if (curr != m_head)
    {
        m_readHead = curr;
        return true;
    }
    return false;
}

bool LoopsIO::Helpers::ReadBuffer::contains(char character, std::size_t offset) const
{
    auto curr = m_readHead + offset;
    for (; curr < m_head; ++curr)
    {
        if (m_data[curr] == character) return true;
    }
    return false;
}

bool LoopsIO::Helpers::ReadBuffer::readChunk(std::istream& stream, std::size_t size)
{
    if (m_capacity == m_head) return false;

    if (size == 0)
    {
        auto read = stream.read(m_data + m_head, m_capacity - m_head).gcount();
        m_head += read;
    }
    else
    {
        auto read = stream.read(m_data + m_head, std::min(m_capacity - m_head, size)).gcount();
        m_head += read;
    }
    return true;
}

void LoopsIO::Helpers::ReadBuffer::seek(std::size_t pos)
{
    m_readHead = std::min(m_head, pos);
}

std::size_t LoopsIO::Helpers::ReadBuffer::tell() const
{
    return m_readHead;
}

std::size_t LoopsIO::Helpers::ReadBuffer::size() const
{
    return m_head - m_readHead;
}

std::size_t LoopsIO::Helpers::ReadBuffer::capacity() const
{
    return m_capacity;
}

LoopsIO::Helpers::ReadBuffer& LoopsIO::Helpers::ReadBuffer::skipWs()
{
    while (std::isspace(m_data[m_readHead]) && m_readHead < m_head)
    {
        ++m_readHead;
    }
    return *this;
}

LoopsIO::Helpers::ReadBuffer::~ReadBuffer()
{
    delete[] m_data;
}
