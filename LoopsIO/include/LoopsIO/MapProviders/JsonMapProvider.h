#ifndef LOOPSIO_MAPPROVIDERS_JSONMAPPROVIDER_H
#define LOOPSIO_MAPPROVIDERS_JSONMAPPROVIDER_H
#include <LoopsIO/IMapProvider.h>

namespace LoopsIO::MapProviders
{
    class JsonMapProvider : public IMapProvider
    {
    public:
        explicit JsonMapProvider();

        void write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& decompObj) override;
        void read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj) override;
    };
}
#endif