#ifndef LOOPSIO_FIELDSERIALIZATION_SERIALIZABLEFIELD_H
#define LOOPSIO_FIELDSERIALIZATION_SERIALIZABLEFIELD_H
#include <string>
#include <type_traits>
#include <typeindex>

namespace LoopsIO::FieldSerialization
{
    class ISerializableField
    {
    protected:
        std::string m_name;
    public:
        ISerializableField(const std::string& name):m_name(name){}
        virtual  ~ISerializableField() = default;
        std::string name() const
        {
            return m_name;
        }
    };
    struct FieldName
    {
        std::string name;
    };
    template<typename Cls, typename DataType>
    class TypedSerializableField : public ISerializableField
    {
    public:
        explicit TypedSerializableField(const std::string& name)
            : ISerializableField(name)
        {
        }

        std::type_index typeIndex() const
        {
            return typeid(DataType);
        }
        virtual void set(const DataType& data) = 0;
        virtual DataType get() = 0;
    };
    template<typename Archive, typename Cls, typename DataType>
    inline Archive& operator&(Archive& archive, TypedSerializableField<Cls,DataType>& serializer)
    {
        return archive & FieldName{ serializer.name() } &serializer.get();
    }
    template<typename Archive, typename Cls, typename DataType>
    inline Archive& operator&(Archive& archive, const TypedSerializableField<Cls, DataType>& serializer)
    {
        DataType type;
        auto& res = archive & FieldName{ serializer.name() } &type;
        serializer.set(type);
        return res;
    }
    template<typename Cls, typename DataType>
    class MemberSerializer : public TypedSerializableField<Cls,DataType>
    {
        DataType Cls::*Member;
        Cls* m_target = nullptr;
    public:
        MemberSerializer(const std::string& name, DataType Cls::* member)
            : TypedSerializableField<Cls, DataType>(name),
              Member(member)
        {
        }

        void setTarget(Cls* target)
        {
            m_target = target;
        }

        void set(const DataType& data) override
        {
            m_target->*Member = data;
        }
        DataType get() override
        {
            return m_target->*Member;
        }
    };
    template<typename Cls, typename DataType>
    class PropertySerializer : public TypedSerializableField<Cls, DataType>
    {
        DataType(Cls::*Getter)();
        void(Cls::*Setter)(const DataType&);
        Cls* m_target = nullptr;
    public:

        PropertySerializer(const std::string& name, DataType( Cls::* getter)(), void( Cls::* setter)(const DataType&))
            : TypedSerializableField<Cls, DataType>(name),
              Getter(getter),
              Setter(setter)
        {
        }

        void setTarget(Cls* target)
        {
            m_target = target;
        }

        void set(const DataType& data) override
        {
            std::invoke(Setter, *m_target, data);
        }
        DataType get() override
        {
            return std::invoke(Getter, *m_target);
        }
    };

    template<typename Cls>
    struct SerializableField
    {

    };
}
#endif