#ifndef LOOPSIO_FIELDSERIALIZATION_BLUEPRINT_H
#define LOOPSIO_FIELDSERIALIZATION_BLUEPRINT_H
#include "SerializableField.h"
#include <map>
#include <tuple>
namespace LoopsIO::FieldSerialization
{
    struct object_name_tag
    {
        std::string name;
    };
    struct field_tag
    {
        std::string name;
    };

    template<typename Cls, typename...Fields>
    struct BluePrint
    {
        std::string m_name;
        std::tuple<Fields...> m_fields;
        BluePrint(const std::string& name, Fields...fields): m_name(name), m_fields(std::make_tuple(fields...)){}

        template<typename OutputStream, std::size_t I>
        void writeField(OutputStream& stream, Cls* toWrite)
        {
            auto& fieldSerializer = std::get<I>(m_fields);
            fieldSerializer.setTarget(toWrite);
            stream << field_tag{ fieldSerializer.name() } << fieldSerializer.get();
        }
        template<typename OutputStream, std::size_t...Is>
        void writeAllFields(OutputStream& stream, std::index_sequence<Is...>)
        {
            (writeField(stream, Is), ...);
        }

        template<typename OutputStream>
        void write(OutputStream& stream, const BluePrint<Cls, Fields...>& bp, Cls* toWrite)
        {
            stream << object_name_tag{ m_name };
            writeAllFields(stream, std::index_sequence_for<Fields...>{});
        }
        template<typename InputStream>
        void read(InputStream& stream, const BluePrint<Cls,Fields...>& bp, Cls* toRead)
        {
            
        }
    };
}
#endif