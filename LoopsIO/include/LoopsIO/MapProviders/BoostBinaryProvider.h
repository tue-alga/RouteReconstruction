#ifndef LOOPS_IO_BOOSTBINARYPROVIDER_H
#define LOOPS_IO_BOOSTBINARYPROVIDER_H
#include <LoopsIO/IMapProvider.h>

namespace LoopsIO::MapProviders
{
    class BoostBinaryProvider : public IMapProvider
    {
        static const int WITH_LAYERS = 1;
    public:
        explicit BoostBinaryProvider();

        void write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& decompObj) override;
        void read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj) override;
    };
}
#endif