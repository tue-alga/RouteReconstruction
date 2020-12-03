#ifndef LOOPSIO_INSTREAMWRAPPER_H
#define LOOPSIO_INSTREAMWRAPPER_H
#include <istream>
#include <sstream>
#include <cassert>
#include <set>
#include <map>
#include <vector>
#include <functional>

namespace LoopsIO::Helpers
{
    namespace detail
    {
        template <typename T> struct LambdaArgs : LambdaArgs<decltype(&T::operator())> {};

        template <typename C, typename ... Args>
        struct LambdaArgs<void(C::*)(Args...) const>
        {
            using type = void(Args...);
            using ArgTuple = std::tuple<Args...>;
            using ReturnType = void;
            using FuncType = std::function<type>;
        };
        template <typename C, typename RetType, typename ... Args>
        struct LambdaArgs<RetType(C::*)(Args...) const>
        {
            using type = RetType(Args...);
            using ArgTuple = std::tuple<Args...>;
            using ReturnType = RetType;
            using FuncType = std::function<type>;
        };
    }
    class InStreamWrapper
    {
        std::istream& str;
        char m_indirectBuff[1028];
        std::stringstream m_partial;


    public:
        InStreamWrapper(std::istream& str);

        InStreamWrapper& clearIntervalBuffer();

        InStreamWrapper& ignore(char c);

        template<typename T>
        InStreamWrapper& read(T& val)
        {
            str >> val;
            return *this;
        }
        template<typename Lambda>
        InStreamWrapper& readApply(const Lambda& func)
        {
            using T = std::decay_t<std::tuple_element_t<0, typename detail::LambdaArgs<Lambda>::ArgTuple>>;
            T val;
            str >> val;
            func(val);
            return *this;
        }
        template<typename Lambda>
        InStreamWrapper& readMultipleTimes(std::size_t times, const Lambda& readFunc)
        {
            for (std::size_t i = 0; i < times; ++i)
            {
                readFunc(i, *this);
            }
            return *this;
        }
        template<typename T>
        T read()
        {
            T t;
            read(t);
            return t;
        }

        operator bool() const;

        template<typename Vector>
        InStreamWrapper& readVector(std::size_t size, Vector& target, const std::string& sep)
        {
            using T = std::decay_t<decltype(target[0])>;
            auto it = std::back_inserter(target);
            T curr;
            str >> curr;
            *it = curr;
            char* data = new char[sep.size()];
            std::string_view view(data, sep.size());
            for (auto i = size * 0 + 1; i < size; ++i)
            {
                str.read(data, sep.size());
                assert(sep == view);
                str >> curr;
                *it = curr;
            }
            delete[] data;
            return *this;
        }
        template<typename Vector, typename Converter>
        InStreamWrapper& readVector(std::size_t size, Vector& target, Converter lambdaConverter, const std::string& sep)
        {
            using Read_t = std::decay_t<std::tuple_element_t<0, typename detail::LambdaArgs<Converter>::ArgTuple>>;
            auto it = std::back_inserter(target);
            Read_t curr;
            str >> curr;
            *it = lambdaConverter(curr);
            char* data = new char[sep.size()];
            std::string_view view(data, sep.size());
            for (auto i = size * 0 + 1; i < size; ++i)
            {
                str.read(data, sep.size());
                assert(sep == view);
                str >> curr;
                *it = lambdaConverter(curr);
            }
            delete[] data;
            return *this;
        }

        InStreamWrapper& ignoreWs();

        InStreamWrapper& readJsonField(std::string& name);

        bool nextIsXmlOpen();

        InStreamWrapper& readXmlStart(std::string& tagName);

        InStreamWrapper& readXmlAttributes(std::map<std::string, std::string>& attrs,
                                           const std::set<std::string>& attrsToRead);

        InStreamWrapper& readXmlElementCloseTag(std::string& tagName);

        InStreamWrapper& readXmlInlineClose();

        template<typename T>
        InStreamWrapper& readXmlAttribute(std::string& attrName, T& value)
        {
            ignoreWs();
            const auto startPos = str.tellg();
            char c = str.get();
            std::vector<char> read;
            read.reserve(128);
            while (c != '=' && c != '>' && str)
            {
                read.push_back(c);
                c = str.get();
            }
            if (c == '=')
            {
                attrName = std::string(read.begin(), read.end());
                str.ignore(1, '"');
                std::stringstream ss;
                std::string temp;
                std::getline(str, temp, '"');
                ss.str(temp);
                ss >> value;
            }
            else
            {
                for (auto ci : read) str.putback(ci);
            }
            return *this;
        }

        InStreamWrapper& readLine(const std::string& target);

        InStreamWrapper& readLineTo(std::string& target);

        InStreamWrapper& readLineTo(std::stringstream& target);
    };
}
#endif