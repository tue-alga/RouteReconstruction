#ifndef LOOPSIO_MAPPROVIDERS_MAPTXTMAPPROVIDER_H
#define LOOPSIO_MAPPROVIDERS_MAPTXTMAPPROVIDER_H
#include <LoopsIO/IMapProvider.h>

namespace LoopsIO::MapProviders
{
    class MapTxtMapProvider : public IMapProvider
    {
    public:
        explicit MapTxtMapProvider()
            : IMapProvider("Map txt (*.maptxt)","maptxt")
        {
        }

        void write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& decompObj) override;
        void read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj) override;
    };
}
#endif