#ifndef LOOPSIO_FIELDPROVIDERS_TXTFIELDPROVIDER_H
#define LOOPSIO_FIELDPROVIDERS_TXTFIELDPROVIDER_H
#include <LoopsIO/IFieldProvider.h>

namespace LoopsIO::FieldProviders
{
    class TxtFieldProvider : public IFieldProvider
    {
    public:
        TxtFieldProvider()
            : IFieldProvider("FieldTxt (*.fieldtxt)","fieldtxt")
        {
        }

        void read(const std::string& filePath, LoopsLib::Models::FlowField& flowField) override;
        void write(const std::string& filePath, const LoopsLib::Models::FlowField& decompObj) override;
    };
}
#endif