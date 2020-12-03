#ifndef LOOPSIO_PROBLEMINSTANCEPROVIDERS_BOOSTPROVIDERS_H
#define LOOPSIO_PROBLEMINSTANCEPROVIDERS_BOOSTPROVIDERS_H
#include "LoopsIO/ISerializer.h"
#include <LoopsLib/Models/ProblemInstance.h>

namespace LoopsIO::ProblemInstanceProviders
{
    class BoostTxtProvider : public TypedSerializer<LoopsLib::Models::ProblemInstance>
    {
    public:
        BoostTxtProvider();

        void read(const std::string& fileName, LoopsLib::Models::ProblemInstance& out) override;

        void write(const std::string& fileName, const LoopsLib::Models::ProblemInstance& out) override;
    };
    class BoostBinProvider : public TypedSerializer<LoopsLib::Models::ProblemInstance>
    {
    public:
        BoostBinProvider();

        void read(const std::string& fileName, LoopsLib::Models::ProblemInstance& out) override;

        void write(const std::string& fileName, const LoopsLib::Models::ProblemInstance& out) override;
    };
}
#endif