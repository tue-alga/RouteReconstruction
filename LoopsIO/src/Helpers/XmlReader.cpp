#include <LoopsIO/Helpers/XmlReader.h>
#include <cctype>
bool LoopsIO::Helpers::XmlReader::xmlStart(char c)
{
    return c == '<';
}

bool LoopsIO::Helpers::XmlReader::xmlEnd(char c)
{
    return c == '>';
}

bool LoopsIO::Helpers::XmlReader::tagEnd(char c)
{
    return std::isspace(c) || c == '>' || c == '/';
}

bool LoopsIO::Helpers::XmlReader::XmlAttribute::operator==(const char* otherS) const
{
    return m_value == otherS;
}

LoopsIO::Helpers::XmlReader::XmlReader(std::istream& str): str(str), m_buf(1024)
{
}

bool LoopsIO::Helpers::XmlReader::isClose() const
{
    return m_isClose;
}

bool LoopsIO::Helpers::XmlReader::isInlineCloseTag() const
{
    if (m_currentTagElements.size() == 0) return false;
    return m_currentTagElements.back() == '/';
}

std::unordered_map<std::string_view, LoopsIO::Helpers::XmlReader::XmlAttribute> LoopsIO::Helpers::XmlReader::attributes()
{
    if (m_isClose) return {};
    std::unordered_map<std::string_view, XmlAttribute> attrs;
    auto pos = m_buf.tell();
    // Need atleast a key, an = and two " for an attribute
    while (m_buf.skipWs().tell() - pos + 4 < m_tagSize)
    {
        auto key = m_buf.substr('=', true);
        m_buf.skipTo('"');
        m_buf.advance();
        XmlAttribute val{m_buf.substr('"', true)};
        m_buf.advance();
        attrs[key] = val;
    }
    m_buf.seek(pos);

    return attrs;
}

LoopsIO::Helpers::XmlReader::operator bool() const
{
    return str || !m_buf.empty();
}

LoopsIO::Helpers::XmlReader& LoopsIO::Helpers::XmlReader::readNextElement()
{
    if (m_activeTag) parseToEnd();

    if (!m_buf.skipTo('<'))
    {
        m_buf.refit();
        if (!m_buf.readChunk(str))
        {
            m_buf.clear();
            return *this;
        }
        m_buf.skipTo('<');
    }
    if (!m_buf.contains('>'))
    {
        m_buf.refit();
        if (!m_buf.readChunk(str))
        {
            m_buf.clear();
            return *this;
        }
    }

    m_buf.advance(1);
    m_isClose = m_buf.charAt(0) == '/';
    if (m_isClose) m_buf.advance(1);

    m_currentTag = m_buf.substr(tagEnd, true);
    m_currentTagElements = m_buf.substr(xmlEnd);
    m_tagSize = m_currentTagElements.size();
    m_activeTag = true;
    return *this;
}

LoopsIO::Helpers::XmlReader& LoopsIO::Helpers::XmlReader::parseToEnd()
{
    //m_currentTagElements.size();
    m_buf.skipTo('>');
    m_activeTag = false;
    return *this;
}

const std::string_view& LoopsIO::Helpers::XmlReader::tagName() const
{
    return m_currentTag;
}
