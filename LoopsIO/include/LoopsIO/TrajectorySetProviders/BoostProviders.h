#ifndef LOOPSIO_TRAJECTORYSETPROVIDERS_BOOSTPROVIDERS_H
#define LOOPSIO_TRAJECTORYSETPROVIDERS_BOOSTPROVIDERS_H
#include <LoopsIO/ISerializer.h>
namespace LoopsIO::TrajectorySetProviders
{
    class BoostBinGraphTrajectorySetProvider : public TypedSerializer<LoopsLib::MovetkGeometryKernel::GraphTrajectorySet>
    {
    public:
        BoostBinGraphTrajectorySetProvider();

        void read(const std::string& fileName,
            LoopsLib::MovetkGeometryKernel::GraphTrajectorySet& out) override;

        virtual void write(const std::string& fileName,
                           const LoopsLib::MovetkGeometryKernel::GraphTrajectorySet& out) override;
    };
    class BoostTxtGraphTrajectorySetProvider : public TypedSerializer<LoopsLib::MovetkGeometryKernel::GraphTrajectorySet>
    {
    public:
        BoostTxtGraphTrajectorySetProvider();

        void read(const std::string& fileName,
            LoopsLib::MovetkGeometryKernel::GraphTrajectorySet& out) override;

        virtual void write(const std::string& fileName,
            const LoopsLib::MovetkGeometryKernel::GraphTrajectorySet& out) override;
    };
}
#endif