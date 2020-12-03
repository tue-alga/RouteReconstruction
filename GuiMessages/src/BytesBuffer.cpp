#include <GuiMessages/BytesBuffer.h>

GuiMessages::BytesRange::~BytesRange()
{
    if (m_isOwned) delete[] m_begin;
}

void GuiMessages::BytesRange::cleanup()
{
    if (m_isOwned) delete[] m_begin;
    m_isOwned = false;
    m_begin = nullptr;
    m_capacity = 0;
}

void GuiMessages::BytesRange::create(std::size_t len)
{
    if (m_isOwned) delete[] m_begin;
    m_begin = new char[len];
    m_capacity = len;
    m_isOwned = true;
    m_filled = 0;
}

std::pair<char*, std::size_t> GuiMessages::BytesRange::toPair() const
{
    return std::make_pair(m_begin, m_capacity);
}

void GuiMessages::BytesRange::copyFrom(void* src)
{
    memcpy(m_begin, src, m_capacity);
    m_filled = m_capacity;
}

void GuiMessages::BytesRange::copyTo(void* dest) const
{
    memcpy(dest, m_begin, m_filled);
}

void GuiMessages::BytesBuffer::tryAllocate(std::size_t toAdd)
{
    if (m_head + toAdd < m_capacity) return;

    while(m_head + toAdd > m_capacity)
    {
        m_capacity *= 2;
    }
    char* newData = new char[m_capacity];
    memcpy(newData, m_data, m_head);
    delete[] m_data;
    m_data = newData;
}

GuiMessages::BytesBuffer::BytesBuffer(std::size_t len): m_capacity(len), m_data(new char[len]), m_head(0), m_readHead(0)
{
    // Should be sufficient for basic types
    m_scratchBuffer.create(64);
}

GuiMessages::BytesBuffer::~BytesBuffer()
{
    delete[] m_data;
}

std::size_t GuiMessages::BytesBuffer::size() const
{
    return m_head;
}

std::size_t GuiMessages::BytesBuffer::readSize() const
{
    return m_head - m_readHead;
}

std::size_t GuiMessages::BytesBuffer::capacity() const
{
    return m_capacity;
}

GuiMessages::BytesRange GuiMessages::BytesBuffer::readableRange() const
{
    return BytesRange{m_data, m_head, m_head, false};
}

GuiMessages::BytesRange& GuiMessages::BytesBuffer::scratchBuf()
{
    return m_scratchBuffer;
}

void GuiMessages::BytesBuffer::seek(std::size_t pos)
{
    if (pos > m_head) throw std::out_of_range("Buffer out of bounds!");
    m_readHead = pos;
}

std::size_t GuiMessages::BytesBuffer::tell() const
{
    return m_readHead;
}

void GuiMessages::BytesBuffer::copyRaw(const char* data, std::size_t len)
{
    if (len == 0)return;
    tryAllocate(len);
    memcpy(m_data + m_head, data, len);
    m_head += len;
}

void GuiMessages::BytesBuffer::writeRawAt(std::size_t offset, void* data, std::size_t len)
{
    tryAllocate(offset + len - m_head);
    memcpy(m_data + offset, data, len);
}

GuiMessages::BytesRange GuiMessages::BytesBuffer::getDataAndProgress(std::size_t len)
{
    if (m_readHead + len > m_head)
    {
        throw std::runtime_error(
            "Reading out of bounds, " + std::to_string(len) + " while at " + std::to_string(m_readHead) + " with sz " +
            std::to_string(m_head));
    }
    BytesRange br{m_data + m_readHead, len, len, false};
    m_readHead += len;
    return br;
}

void GuiMessages::BytesBuffer::resetReadHead()
{
    m_readHead = 0;
}

void GuiMessages::BytesBuffer::copyRaw(const BytesRange& data)
{
    if (data.m_filled == 0)return;
    tryAllocate(data.m_filled);
    data.copyTo(m_data + m_head);
    m_head += data.m_filled;
}

bool GuiMessages::BytesBuffer::canReadBytes(std::size_t len) const
{
    return m_head >= len;
}
