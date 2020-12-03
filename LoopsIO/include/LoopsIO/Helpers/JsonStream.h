#ifndef LOOPSIO_JSONSTREAM_H
#define LOOPSIO_JSONSTREAM_H
#include <ostream>
#include <Eigen/Eigen>
#include <stack>

namespace LoopsIO::Helpers
{
    namespace mod
    {
        enum class JsonStructure
        {
            Field,
            Array,
            Object,
            Value
        };
        struct JsonObject
        {
        };
        struct JsonObjectEnd{};
        struct JsonArray{};
        struct JsonArrayEnd{};
        struct JsonField
        {
            std::string name;
        };
    }

    struct JsonStream
    {
        std::ostream& str;

        std::stack<std::pair<mod::JsonStructure,int>> m_structure;
        bool topIs(mod::JsonStructure type)
        {
            return m_structure.top().first == type;
        }
    public:

        JsonStream(std::ostream& str);

        JsonStream& tab(int count = 1);

        JsonStream& field(const std::string& name);

        JsonStream& fieldSep();

        JsonStream& nl();

        JsonStream& operator<<(mod::JsonObject)
        {
            if(m_structure.empty() || topIs(mod::JsonStructure::Array) || topIs(mod::JsonStructure::Field))
            {
                str << "{";
                m_structure.push({ mod::JsonStructure::Object,0 });
            }
            return *this;
        }
        JsonStream& operator<<(mod::JsonField field)
        {
            if (m_structure.top().second > 0) str << ',' << '\n';
            str << '"' << field.name << '"' << ':';
            return *this;
        }

        template<typename T>
        JsonStream& value(const T& val)
        {
            if constexpr (std::is_same_v<T, std::string>) {
                str << '"' << val << '"';
            }
            else {
                str << val;
            }
            return *this;
        }


        JsonStream& beginObject();

        JsonStream& endObject();

        JsonStream& beginArray();

        JsonStream& comma();

        JsonStream& endArray();

        JsonStream& commaNl();

        template<typename Iterable>
        JsonStream& commaSeparated(const Iterable& iterable)
        {
            bool first = true;
            for (const auto& val : iterable)
            {
                if (!first) str << ',';
                else first = false;
                str << val;
            }
            return *this;
        }
        template<typename Iterable, typename Func>
        JsonStream& commaSeparated(const Iterable& iterable, Func func)
        {
            bool first = true;
            for (const auto& val : iterable)
            {
                if (!first) str << ',';
                else first = false;
                str << func(val);
            }
            return *this;
        }
        template<typename Iterable>
        JsonStream& array(const Iterable& iterable)
        {
            str << '[';
            commaSeparated(iterable);
            str << ']';
            return *this;
        }
        template<typename Iterable, typename Func>
        JsonStream& array(const Iterable& iterable, Func func)
        {
            str << '[';
            commaSeparated(iterable, func);
            str << ']';
            return *this;
        }

        JsonStream& eigenArray(const Eigen::VectorXd& vec);

        template<typename T>
        JsonStream& writeValueField(const std::string& name, const T& value)
        {
            return field(name).fieldSep().value(value);
        }
    };
}
#endif