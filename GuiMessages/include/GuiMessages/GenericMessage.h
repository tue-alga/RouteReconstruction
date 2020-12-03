#ifndef GUIMESSAGES_GENERICMESSAGE_H
#define GUIMESSAGES_GENERICMESSAGE_H
#include "BaseMessage.h"

namespace GuiMessages{
    
    template<typename MsgType>
    class GenericMessage : public BaseMessage
    {
        MsgType m_msg;
        template<typename MemTuple, std::size_t...Is>
        void assignFromBuff(MemTuple&& tup, BytesBuffer& buff, std::index_sequence<Is...>)
        {
            using exp = int[];
            (void)exp {
                (buff >> std::invoke(std::get<Is>(tup), m_msg), 0)...
            };
        }
        template<typename MemTuple>
        void assignFromBuff(MemTuple&& tup, BytesBuffer& buff)
        {
            assignFromBuff(tup, buff, std::make_index_sequence<std::tuple_size_v<MemTuple>>{});
        }
        template<typename MemTuple, std::size_t...Is>
        void assignToBuff(MemTuple&& tup, BytesBuffer& buff, std::index_sequence<Is...>) const
        {
            using exp = int[];
            (void)exp {
                (buff << std::invoke(std::get<Is>(tup), m_msg), 0)...
            };
        }
        template<typename MemTuple, std::size_t I>
        void assignToBuff(MemTuple&& tup, BytesBuffer& buff) const
        {
            assignToBuff(tup, buff, std::make_index_sequence<std::tuple_size_v<MemTuple>>{});
        }
    public:
        const MsgType& msg() const
        {
            return m_msg;
        }
        MsgType& msg()
        {
            return m_msg;
        }

        std::string type() const override
        {
            return MsgType::type();
        }
        void encode(BytesBuffer& buff) const override
        {
            auto memTuple = MsgType::members();
            assignToBuff(memTuple, buff);
        }
        void decode(BytesBuffer& buff) override
        {
            auto memTuple = MsgType::members();
            assignFromBuff(memTuple, buff);
        }
    };
}
#endif