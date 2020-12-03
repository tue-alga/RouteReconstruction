#ifndef LOOPSIO_FIELDPROVIDERS_IPEFIELDPROVIDER_H
#define LOOPSIO_FIELDPROVIDERS_IPEFIELDPROVIDER_H
#include <LoopsIO/IFieldProvider.h>

namespace LoopsIO::FieldProviders
{
    class IpeFieldProvider : public IFieldProvider
    {
    public:
        IpeFieldProvider();

        void read(const std::string& uri, LoopsLib::Models::FlowField& flowField) override;
        void write(const std::string& filePath, const LoopsLib::Models::FlowField& decompObj) override;
        std::vector<std::string> requiredArguments() const override;
    };
}
#endif