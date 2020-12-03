#ifndef IO_IOHELPERS_H
#define IO_IOHELPERS_H
#include <ostream>
#include <type_traits>
#include <Eigen/Eigen>
#include <string>
#include "LoopsLib/DS/BiMap.h"
#include "LoopsLib/Math/Vector.h"
#include "LoopsLib/Algs/Types.h"
#include <QString>
#include <qregexp.h>
#include <qstringlist.h>

namespace IO::IOHelpers
{
    namespace detail
    {
        // Deduction of lambda argument values and return type
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
    inline std::string extension(const std::string& filePath)
    {
        auto pos = filePath.find_last_of('.');
        if (pos == std::string::npos) return "";
        return filePath.substr(pos + 1);
    }

    inline void ipeMatrixFromString(const QString& string, LoopsLib::Math::Matrix3<LoopsLib::KernelDef::NT>& output)
    {
        auto parts = string.split(QRegExp("\\s+"));
        std::vector<long double> vals;
        for (const auto& el : parts)
        {
            vals.push_back(el.toDouble());
        }

        for (int i = 0; i < 6; ++i)
        {
            auto r = i % 2;
            auto c = (i - r) / 2;
            output(r, c) = vals[i];
        }
    }
    template<typename U>
    struct StringConvert
    {
        static U fromString(const std::string& str)
        {
            
        }
        static std::string toString(const U& u){}
    };
#define TO_STR_CONV(type)template<> inline std::string StringConvert<type>::toString(const type& in)
#define FROM_STR_CONV(type)template<> inline type StringConvert<type>::fromString(const std::string& in)
    template<>
    inline std::string StringConvert<std::string>::fromString(const std::string& in) { return in; }
    template<>
    inline std::string StringConvert<std::string>::toString(const std::string& in) { return in; }
    TO_STR_CONV(double) { return std::to_string(in); }
    FROM_STR_CONV(double) { return std::stod(in); }
    TO_STR_CONV(long double) { return std::to_string(in); }
    FROM_STR_CONV(long double) { return std::stold(in); }
    TO_STR_CONV(float) { return std::to_string(in); }
    FROM_STR_CONV(float) { return static_cast<float>(std::stod(in)); }
    TO_STR_CONV(long long) { return std::to_string(in); }
    FROM_STR_CONV(long long) { return std::stoll(in); }
    TO_STR_CONV(int) { return std::to_string(in); }
    FROM_STR_CONV(int) { return std::stoi(in); }
    TO_STR_CONV(bool) { return std::to_string(in); }
    FROM_STR_CONV(bool) { return in == "true" || in == "1";}


    /**
     * \brief Simplistic implementation of Uri. Contains a path (no protocol spec) and arguments appended to it.
     */
    class Uri
    {
        std::map<std::string, std::string> m_args;
        std::string m_path;
    public:
        Uri();
        Uri(const std::string& path, const std::map<std::string, std::string>& args);

        std::string extension() const;

        std::string encoded(char argSeparator = '&', char startOfArgs = '?');

        void encodeToStream(std::ostream& str, char argSeparator = '&', char startOfArgs = '?');

        const std::map<std::string, std::string>& args() const;

        template<typename T>
        T argValue(const std::string& name) const
        {
            auto str = m_args.at(name);
            return StringConvert<T>::fromString(str);
        }
        template<typename T>
        void addArg(const std::string& name, const T& value)
        {
            m_args[name] = StringConvert<T>::toString(value);
        }

        void removeArg(const std::string& name);

        bool hasArg(const std::string& name) const;

        static Uri decode(const std::string& uriStr, char argSeparator = '&', char startOfArgs = '?');

        bool operator==(const Uri& other) const;

        bool hasArgs(const std::vector<std::string>& args) const;

        bool hasArgs(const std::vector<std::string>& args, std::vector<std::string>& missing) const;
    };

    struct UrlEncoder
    {
        using ArgsMap = std::map<std::string, std::string>;
        static LoopsLib::DS::BiMap<char, std::string> m_encoderMap;

        static void addArgs(std::string& uri, const std::map<std::string, std::string>& args);

        static void extractPath(const std::string& uri, std::string& path);
        static void extractArgs(const std::string& uri, std::map<std::string, std::string>& args, char argSeparator = '&', char startArgs = '?');

        static bool isSameUri(const std::string& uri1, const std::string& uri2);

        static void encodeStringToStream(const std::string& string, std::ostream& out);

        static void encodeArgsToStream(const std::map<std::string, std::string>& args, std::ostream& out);

        static void decodeStringToStream(const std::string_view& string, std::ostream& out);

        static void decodeStringToStream(const std::string& string, std::ostream& out);

        static std::string encodeUrl(const std::string& path,
                                            const std::map<std::string, std::string>& arguments, char argSeparator = '&', char startArgs = '?');
        static void encodeUrlToStream(const std::string& path,
            const std::map<std::string, std::string>& arguments,std::ostream& str, char argSeparator = '&', char startArgs = '?');
        static std::string encodeUrl(const std::string& path,
            const std::map<std::string, std::string>& arguments, const std::vector<std::string>& argSelection, char argSeparator = '&', char startArgs = '?');

        static void decodeUrl(const std::string& url, std::string& path,
                                     std::map<std::string, std::string>& arguments, char argSeparator='&',char startArgs='?');
        static std::pair<std::string, ArgsMap> decodeUrl(const std::string& url, char argSeparator = '&', char startArgs = '?');

        static bool hasMissingArguments(const std::string& filePath, const std::vector<std::string>& requiredArgs,
                                        std::vector<std::string>& missingArgs);
    };

}
#endif