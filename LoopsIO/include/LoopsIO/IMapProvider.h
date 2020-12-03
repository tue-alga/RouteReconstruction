#ifndef IO_MAPPROVIDERS_IMAPPROVIDER_H
#define IO_MAPPROVIDERS_IMAPPROVIDER_H
#include <string>
#include <LoopsLib/Helpers/DecompositionObject.h>
namespace LoopsIO
{
    class IMapProvider
    {
    protected:
        std::string m_fileFilter, m_extension;
    public:
        using Point = LoopsLib::MovetkGeometryKernel::MovetkPoint;
        IMapProvider(const std::string& fileFilter, const std::string& extension) : m_fileFilter(fileFilter),m_extension(extension) {}

        std::string fileFilter() const
        {
            return m_fileFilter;
        }
        std::string extension() const
        {
            return m_extension;
        }

        virtual std::vector<std::string> requiredParameters() const
        {
            return {};
        }

        virtual  ~IMapProvider(){}
        virtual void write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& decompObj) = 0;

        virtual void read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj) = 0;
    };
}
#endif