#ifndef LOOPSIO_JSONINSTREAM_H
#define LOOPSIO_JSONINSTREAM_H
#include <istream>
#include <stack>
#include <vector>
#include <cassert>
#include <string>
#include <iterator>

namespace LoopsIO::Helpers
{

    class JsonInStream
    {
    public:
        /**
         * \brief All tokens in a JSON file
         */
        enum class Token
        {
            ObjectBegin,
            ObjectEnd,
            ArrayBegin,
            ArrayEnd,
            FieldName,
            StringValue,
            Value,
            Eof,
            Sof,
            Comma,
            FieldSep,
            Unknown
        };
    private:
        std::istream& str;
        Token m_lastToken = Token::Sof;
        std::stack<Token> m_structure;
        bool m_inObject = false;
        bool m_inArray = false;
    public:
        JsonInStream(std::istream& str);

        std::istream::pos_type pos() const;

        Token moveToPos(std::streamsize pos);

        JsonInStream& skipToJustPast(char c);

        JsonInStream& parseFieldName(std::string& fieldName);

        template<typename T>
        JsonInStream& parseValue(T& out)
        {
            assert(m_lastToken == Token::FieldName || m_structure.top() == Token::ArrayBegin || m_lastToken == Token::Value);
            if constexpr (std::is_same_v<T, std::string>) {
                skipToJustPast('"');
                std::getline(str, out, '"');
            }
            else
            {
                str >> out;
            }
            return *this;
        }
        template<typename T>
        JsonInStream& parseArray(std::vector<T>& output)
        {
            assert(m_lastToken == Token::ArrayBegin);
            Token t = skipLastToken();
            while (t != Token::ArrayEnd)
            {
                if (t == Token::Comma)
                {
                    t = skipLastToken();
                }
                assert(t == Token::Value);
                T val;
                str >> val;
                output.push_back(val);
            }
            return *this;
        }
        template<typename Container>
        JsonInStream& parseArray(Container& output)
        {
            auto insertIt = std::inserter(output, output.begin());
            using It = decltype(output.begin());
            using T = std::decay_t<typename It::value_type>;
            assert(m_lastToken == Token::ArrayBegin);
            Token t = skipLastToken();
            while (t != Token::ArrayEnd)
            {
                if (t == Token::Comma)
                {
                    t = skipLastToken();
                }
                assert(t == Token::Value);
                T val;
                str >> val;
                *insertIt = val;
            }
            return *this;
        }

        /**
         * \brief Skips the data belonging to the last token.
         * \return The next token
         */
        Token skipLastToken();

        /**
         * \brief Returns the next token
         * \return The new token
         */
        Token nextToken();
    };

}
#endif 