#include <LoopsIO/Helpers/InStreamWrapper.h>
#include <cctype>
LoopsIO::Helpers::InStreamWrapper::InStreamWrapper(std::istream& str): str(str)
{
    m_partial.rdbuf()->pubsetbuf(m_indirectBuff, 1028);
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::clearIntervalBuffer()
{
    m_partial.clear();
    m_partial.rdbuf()->pubseekpos(0);
    //m_partial.seekg(0);
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::ignore(char c)
{
    str.ignore(std::numeric_limits<std::streamsize>::max(), c);
    return *this;
}

LoopsIO::Helpers::InStreamWrapper::operator bool() const
{
    return (bool)str;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::ignoreWs()
{
    char c = str.peek();
    while (str && std::isspace(c))
    {
        str.get();
        c = str.peek();
    }
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readJsonField(std::string& name)
{
    str.ignore(std::numeric_limits<std::streamsize>::max(), '"');
    std::getline(str, name, '"');
    str.ignore(std::numeric_limits<std::streamsize>::max(), ':');
    return *this;
}

bool LoopsIO::Helpers::InStreamWrapper::nextIsXmlOpen()
{
    auto pos = str.tellg();
    ignore('<');
    ignoreWs();
    char c = str.get();
    str.seekg(pos);
    return '/' != c;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readXmlStart(std::string& tagName)
{
    ignore('<');
    clearIntervalBuffer();
    char c = str.peek();
    while (c != '>' && !std::isspace(c) && str)
    {
        m_partial << c;
        str.get();
        c = str.peek();
    }
    tagName = m_partial.str();
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readXmlAttributes(std::map<std::string, std::string>& attrs,
                                                                      const std::set<std::string>& attrsToRead)
{
    while (true)
    {
        std::string attrName;
        std::string attrValue;
        readXmlAttribute(attrName, attrValue);
        if (attrName.empty()) break;
        if (attrsToRead.find(attrName) != attrsToRead.end()) attrs[attrName] = attrValue;
    }
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readXmlElementCloseTag(std::string& tagName)
{
    ignore('<');
    ignoreWs();
    char c = str.get();
    //assert(c == '<');
    //c = str.get();
    assert(c == '/');
    std::getline(str, tagName, '>');
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readXmlInlineClose()
{
    ignoreWs();
    char c = str.get();
    assert(c == '/');
    c = str.get();
    assert(c == '>');
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readLine(const std::string& target)
{
    std::string line;
    std::getline(str, line);
    while (str && line.empty())
    {
        std::getline(str, line);
    }
    assert(line == target);
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readLineTo(std::string& target)
{
    std::getline(str, target);
    return *this;
}

LoopsIO::Helpers::InStreamWrapper& LoopsIO::Helpers::InStreamWrapper::readLineTo(std::stringstream& target)
{
    std::string line;
    std::getline(str, line);
    target.str(line);
    target.seekg(0);
    return *this;
}
