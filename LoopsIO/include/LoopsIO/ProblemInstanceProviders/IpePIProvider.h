#ifndef LOOPSIO_PROBLEMINSTANCEPROVIDERS_IPEPIPROVIDER_H
#define LOOPSIO_PROBLEMINSTANCEPROVIDERS_IPEPIPROVIDER_H
#include "LoopsIO/ISerializer.h"
#include <LoopsLib/Models/ProblemInstance.h>
#include <LoopsIO/Serializers.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray> 


namespace LoopsIO::ProblemInstanceProviders
{
    class IpePIProvider : public TypedSerializer<LoopsLib::Models::ProblemInstance>
    {
    public:
        IpePIProvider()
            : TypedSerializer<LoopsLib::Models::ProblemInstance>("ipe", "Ipe (*.ipe)", "ipe")
        {
        }

        void read(const std::string& fileName, LoopsLib::Models::ProblemInstance& out) override;

        virtual void write(const std::string& fileName, const LoopsLib::Models::ProblemInstance& out) override;

        std::vector<std::string> requiredArguments() const override;
        std::map<std::string, std::string> argumentDefaults() const override;
    };
}
#endif