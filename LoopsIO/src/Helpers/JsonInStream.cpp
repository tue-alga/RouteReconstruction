#include <LoopsIO/Helpers/JsonInStream.h>
#include <cassert>
#include <string>

LoopsIO::Helpers::JsonInStream::JsonInStream(std::istream& str): str(str)
{
}

std::istream::pos_type LoopsIO::Helpers::JsonInStream::pos() const
{
    return str.tellg();
}

LoopsIO::Helpers::JsonInStream::Token LoopsIO::Helpers::JsonInStream::moveToPos(std::streamsize pos)
{
    str.seekg(pos);
    return nextToken();
}

LoopsIO::Helpers::JsonInStream& LoopsIO::Helpers::JsonInStream::skipToJustPast(char c)
{
    str.ignore(std::numeric_limits<std::streamsize>::max(), c);
    return *this;
}

LoopsIO::Helpers::JsonInStream& LoopsIO::Helpers::JsonInStream::parseFieldName(std::string& fieldName)
{
    assert(m_lastToken == Token::FieldName);
    str.ignore(std::numeric_limits<std::streamsize>::max(), '"');
    std::getline(str, fieldName, '"');
    str.ignore(std::numeric_limits<std::streamsize>::max(), ':');
    return *this;
}

LoopsIO::Helpers::JsonInStream::Token LoopsIO::Helpers::JsonInStream::skipLastToken()
{
    switch (m_lastToken)
    {
    case Token::StringValue:
        skipToJustPast('"');
        skipToJustPast('"');
        break;
    case Token::FieldName:
        skipToJustPast('"');
        skipToJustPast('"');
        break;
    case Token::Comma:
        skipToJustPast(',');
        break;
    case Token::FieldSep:
        skipToJustPast(':');
        break;
    case Token::ArrayBegin:
        skipToJustPast('[');
        break;
    case Token::ArrayEnd:
        skipToJustPast(']');
        break;
    case Token::ObjectBegin:
        skipToJustPast('{');
        break;
    case Token::ObjectEnd:
        skipToJustPast('}');
        break;
    case Token::Value:
        {
            char c = str.peek();
            while (c != ']' && c != '}' && c != ',')
            {
                str.get();
                c = str.peek();
            }
        }
        break;
    default:
        break;
    }
    return nextToken();
}

LoopsIO::Helpers::JsonInStream::Token LoopsIO::Helpers::JsonInStream::nextToken()
{
    str >> std::ws;
    char c = str.peek();
    if (!str)
    {
        return Token::Eof;
    }
    Token retToken = Token::Value;
    if (c == ',')
    {
        retToken = Token::Comma;
    }
    else if (c == ':')
    {
        retToken = Token::FieldSep;
    }
    else if (c == '{')
    {
        retToken = Token::ObjectBegin;
        m_structure.push(retToken);
    }
    else if (c == '}')
    {
        assert(m_structure.top() == Token::ObjectBegin);
        retToken = Token::ObjectEnd;
        m_structure.pop();
    }
    else if (c == '[')
    {
        retToken = Token::ArrayBegin;
        m_structure.push(retToken);
    }
    else if (c == ']')
    {
        retToken = Token::ArrayEnd;
        assert(m_structure.top() == Token::ArrayBegin);
        m_structure.pop();
    }
    else if (c == '"')
    {
        if (m_lastToken != Token::FieldName)
        {
            retToken = Token::FieldName;
            assert(m_structure.top() == Token::ObjectBegin);
        }
        else
        {
            retToken = Token::StringValue;
        }
    }
    return retToken;
}
