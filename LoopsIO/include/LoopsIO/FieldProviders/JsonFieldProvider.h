#ifndef IO_FIELDPROVIDERS_JSONFIELDPROVIDER_H
#define IO_FIELDPROVIDERS_JSONFIELDPROVIDER_H
#include <LoopsIO/IFieldProvider.h>

namespace LoopsIO::FieldProviders
{
    class JsonFieldProvider : public IFieldProvider
    {
    public:
        JsonFieldProvider();

        void read(const std::string& filePath, LoopsLib::Models::FlowField& decompObj) override;
        void write(const std::string& filePath, const LoopsLib::Models::FlowField& flowField) override;
    };
}
#endif