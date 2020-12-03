#ifndef LOOPS_IO_IPEMAPPROVIDER_H
#define LOOPS_IO_IPEMAPPROVIDER_H
#include "LoopsIO/IMapProvider.h"

namespace LoopsIO::MapProviders
{
    class IpeMapProvider : public IMapProvider
    {
    public:
        IpeMapProvider();

        void write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& decompObj) override;
        void read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj) override;
        std::vector<std::string> requiredParameters() const override;
    };
}
#endif