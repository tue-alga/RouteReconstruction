#include <GuiMessages/BaseMessage.h>

GuiMessages::MessageFactory& GuiMessages::MessageFactory::inst()
{
    static MessageFactory mf;
    return mf;
}

void GuiMessages::MessageFactory::registerMessageTypeConstruct(const std::string& typeName,
                                                               std::function<BaseMessage*()> constructor)
{
    if (m_messageConstructMap.find(typeName) != m_messageConstructMap.end())
    {
        throw std::runtime_error("Duplicate message constructor registered: " + typeName);
    }
    m_messageConstructMap[typeName] = constructor;
}

void GuiMessages::MessageFactory::encodeMessage(const BaseMessage& message, BytesBuffer& buff)
{
    std::size_t totalSize = 0;
    buff << totalSize;
    buff << message.type();
    message.encode(buff);
    totalSize = buff.size();
    buff.writeRawAt(0, (void*)&totalSize, sizeof(std::size_t));
}

bool GuiMessages::MessageFactory::isFullMessage(BytesBuffer& messageBegin) const
{
    if (!messageBegin.canReadSimpleType<std::size_t>()) return false;
    std::size_t pos = messageBegin.tell();
    std::size_t sz;
    messageBegin >> sz;
    messageBegin.seek(pos);
    return messageBegin.size() >= sz;
}

GuiMessages::BaseMessage* GuiMessages::MessageFactory::messageFromBytes(BytesBuffer& buff)
{
    std::size_t payloadSize;
    buff >> payloadSize;
    std::string typeName;
    buff >> typeName;
    return createFromBytes(typeName, buff);
}

GuiMessages::BaseMessage* GuiMessages::MessageFactory::createFromBytes(const std::string& messagetype,
                                                                       BytesBuffer& buff)
{
    if (m_messageConstructMap.find(messagetype) == m_messageConstructMap.end())
    {
        throw std::invalid_argument("No message with typename '" + messagetype + "' exists.");
    }

    auto* msg = m_messageConstructMap[messagetype]();
    msg->decode(buff);
    return msg;
}
