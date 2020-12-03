#ifndef LOOPSIO_OUTSTREAMWRAPPER_H
#define LOOPSIO_OUTSTREAMWRAPPER_H
#include <string>
#include <ostream>

namespace LoopsIO::Helpers
{
    struct OutStreamWrapper
    {
        std::ostream& str;
        OutStreamWrapper(std::ostream& str);

        OutStreamWrapper& flush()
        {
            str.flush();
            return *this;
        }

        template<typename Iterable>
        OutStreamWrapper& writeJoined(const Iterable& it, const std::string& joiner)
        {
            bool first = true;
            for (const auto& el : it)
            {
                if (first) first = false;
                else str << joiner.c_str();
                *this << el;
            }
            return *this;
        }
        template<typename Iterable, typename Lambda>
        OutStreamWrapper& writeJoined(const Iterable& it, Lambda valueFunc, const std::string& joiner)
        {
            bool first = true;
            for (const auto& el : it)
            {
                if (first) first = false;
                else str << joiner.c_str();
                str << valueFunc(el);
            }
            return *this;
        }
        template<typename Iterable, typename Lambda>
        OutStreamWrapper& writeFuncJoined(const Iterable& it, Lambda outStreamFunc, const std::string& joiner)
        {
            bool first = true;
            for (const auto& el : it)
            {
                if (first) first = false;
                else str << joiner.c_str();
                outStreamFunc(el, *this);
            }
            return *this;
        }
        template<typename Value>
        OutStreamWrapper& operator<<(const Value& val)
        {
            if constexpr (std::is_invocable_r<OutStreamWrapper&, Value, OutStreamWrapper&>::value) {
                return val(*this);
            }
            else if constexpr (std::is_same_v<std::string, Value>) {
                str << val.c_str();
                return *this;
            }
            else
            {
                str << val;
                return *this;
            }
        }
        template<typename Value>
        OutStreamWrapper& write(const Value& val)
        {
            return *this << val;
        }
        template<typename Value>
        OutStreamWrapper& writeLn(const Value& val)
        {
            return *this << val << '\n';
        }
        template<typename Lambda>
        OutStreamWrapper& apply(const Lambda& val)
        {
            return *this << val;
        }

        /**
         * \brief Writes a newline to the output
         * \return This for chaining
         */
        OutStreamWrapper& nl();

        /**
         * \brief Writes an opening tag for XML that is not closed at the end
         * \param tag
         * \return
         */
        OutStreamWrapper& writeOpenXml(const std::string& tag);

        template<typename T>
        OutStreamWrapper& doubleQuote(const T& val)
        {
            str << '"' << val << '"';
            return *this;
        }
        template<typename T>
        OutStreamWrapper& writeXmlAttribute(const std::string& attr, const T& val)
        {
            str << attr << '=';
            return doubleQuote(val);
        }
        template<typename T>
        OutStreamWrapper& writeJsonAttribute(const std::string& attr, const T& val)
        {
            if constexpr (std::is_same_v<std::string, T>) {
                return doubleQuote(attr).write(':').doubleQuote(val);
            }
            else
            {
                return doubleQuote(attr).write(':').write(val);
            }
        }

        /**
         * \brief Writes a closing XML tag
         * \param tag The tag name for the closing tag
         * \return This object for chaining
         */
        OutStreamWrapper& writeCloseXml(const std::string& tag);

        /**
         * \brief Adds a tab to the output
         * \return This for chaining
         */
        OutStreamWrapper& tab();

        OutStreamWrapper& space();
    };
}
#endif