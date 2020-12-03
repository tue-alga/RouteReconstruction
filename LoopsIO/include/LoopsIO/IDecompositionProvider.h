#ifndef IO_IDECOMPOSITIONPROVIDER_H
#define IO_IDECOMPOSITIONPROVIDER_H
#include <LoopsLib/Helpers/DecompositionObject.h>
namespace LoopsIO
{
    class IDecompositionProvider
    {
        std::string m_name;
        std::string m_filter;
        std::string m_extension;
    public:
        IDecompositionProvider(const std::string& name, const std::string& filter, const std::string& ext):
        m_name(name), m_filter(filter), m_extension(ext)
        {}
        std::string name() const
        {
            return m_name;
        }
        std::string ext() const
        {
            return m_extension;
        }
        std::string fileFilter() const
        {
            return m_filter;
        }
        virtual ~IDecompositionProvider(){}
        virtual void read(const std::string& filePath, LoopsLib::Helpers::DecompositionObject& decompObj,
            std::function<bool(const std::string&)> dependentMapCb, std::function<bool(const std::string&)> dependentFieldCb) = 0;
        virtual void write(const std::string& filePath, const LoopsLib::Helpers::DecompositionObject& decompObj) = 0;
    };
}
#endif