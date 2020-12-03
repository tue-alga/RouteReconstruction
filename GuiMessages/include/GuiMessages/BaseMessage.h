#ifndef GUIMESSAGES_BASEMESSAGE_H
#define GUIMESSAGES_BASEMESSAGE_H
#include <tuple>
#include <string>
#include <unordered_map>
#include <functional>
#include "BytesBuffer.h"

namespace GuiMessages
{
    class BaseMessage
    {
    public:
        virtual ~BaseMessage() = default;

        virtual std::string type() const = 0;

        virtual void encode(BytesBuffer& buff) const = 0;

        virtual void decode(BytesBuffer& buff) = 0;
    };


    class MessageFactory
    {
        std::unordered_map<std::string, std::function<BaseMessage*()>> m_messageConstructMap;
        MessageFactory(){}
        template<typename T>
        struct Holder
        {
            using type = T;
        };
        template<typename TupType, std::size_t...Is>
        void registerMessageTypesFromTuple(Holder<TupType>,std::index_sequence<Is...>)
        {
            (registerMessageType<std::tuple_element_t<Is, TupType>>(), ...);
        }
    public:
        static MessageFactory& inst();

        void registerMessageTypeConstruct(const std::string& typeName, std::function<BaseMessage*()> constructor);

        template<typename T>
        void registerMessageType()
        {
            registerMessageTypeConstruct(T().type(), []() {return new T(); });
        }
        template<typename TupleType>
        void registerMessageTypesFromTuple()
        {
            registerMessageTypesFromTuple(Holder<TupleType>{}, std::make_index_sequence<std::tuple_size_v<TupleType>>{});
        }

        /**
         * \brief Fully encodes the message, to be sent over a connection
         * Includes the header containing payload size and message type
         * \param message The message to encode
         * \param buff The output buffer
         */
        void encodeMessage(const BaseMessage& message, BytesBuffer& buff);

        /**
         * \brief Checks if the message in the given buffer is complete.
         * \param messageBegin The (partial) message
         * \return Is the message complete
         */
        bool isFullMessage(BytesBuffer& messageBegin) const;

        /**
         * \brief Creates a message from the given byte buffer. The buffer should contain
         * a fully encoded message including header
         * \param buff The buffer
         * \return The message
         */
        BaseMessage* messageFromBytes(BytesBuffer& buff);

        /**
         * \brief Creates a message from a buffer, where the buffer should not contain a header
         * or the readhead should be past the header
         * \param messagetype The message type to create
         * \param buff The input buffer
         * \return The created message
         */
        BaseMessage* createFromBytes(const std::string& messagetype, BytesBuffer& buff);
    };
}
#endif