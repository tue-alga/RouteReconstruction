#ifndef LOOPSIO_MAPPROVIDERS_HDF5MAPPROVIDER_H
#define LOOPSIO_MAPPROVIDERS_HDF5MAPPROVIDER_H
#include <LoopsIO/IMapProvider.h>

namespace LoopsIO::MapProviders
{
    class Hdf5MapProvider : public IMapProvider
    {
    public:
        explicit Hdf5MapProvider()
            : IMapProvider("HDF5 file (*.h5)","h5")
        {
        }

        void write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& decompObj) override;
        void read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj) override;
    };
}
#endif