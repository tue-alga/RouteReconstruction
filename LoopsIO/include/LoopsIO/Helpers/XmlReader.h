#ifndef LOOPSIO_XMLREADER_H
#define LOOPSIO_XMLREADER_H
#include <istream>
#include <LoopsIO/Helpers/ReadBuffer.h>
#include <unordered_map>

namespace LoopsIO::Helpers
{
    class XmlReader
    {
        std::istream& str;
        ReadBuffer m_buf;
        std::string_view m_currentTag;
        std::string_view m_currentTagElements;
        std::size_t m_tagSize = 0;
        bool m_activeTag = false;
        bool m_isClose = false;
        static bool xmlStart(char c);
        static bool xmlEnd(char c);
        static bool tagEnd(char c);
    public:
        struct XmlAttribute
        {
            // The value 
            std::string_view m_value;

            bool operator==(const char* otherS) const;

            template<typename T>
            T value()
            {
                return XmlReader::fromChars<T>(m_value);
            }
        };

        XmlReader(std::istream& str);

        template<typename T>
        static void fromChars(const std::string_view& view, T& target)
        {
            std::from_chars(view.data(), view.data() + view.size(), target);
        }
        template<typename T>
        static T fromChars(const std::string_view& view)
        {
            T target;
            std::from_chars(view.data(), view.data() + view.size(), target);
            return target;
        }

        bool isClose() const;

        bool isInlineCloseTag() const;

        std::unordered_map<std::string_view, XmlAttribute> attributes();

        operator bool() const;

        XmlReader& readNextElement();

        XmlReader& parseToEnd();

        const std::string_view& tagName() const;
    };

}
#endif