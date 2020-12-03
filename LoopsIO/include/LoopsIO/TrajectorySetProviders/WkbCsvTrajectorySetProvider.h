#ifndef LOOPSIO_TRAJECTORYSETPROVIDERS_WKBCSVTRAJECTORYSETPROVIDER_H
#define LOOPSIO_TRAJECTORYSETPROVIDERS_WKBCSVTRAJECTORYSETPROVIDER_H
#include <LoopsIO/ISerializer.h>
namespace LoopsIO::TrajectorySetProviders
{
    class WkbCsvTrajectorySetProvider : public TypedSerializer<LoopsLib::MovetkGeometryKernel::TrajectorySet>
    {
    public:
        WkbCsvTrajectorySetProvider()
            : TypedSerializer<LoopsLib::MovetkGeometryKernel::TrajectorySet>("Wkb Csv trajectory reader", "WKB Csv(*.wkbcsv)", "wkbcsv")
        {
        }

        void read(const std::string& fileName,
            LoopsLib::MovetkGeometryKernel::TrajectorySet& out)
        override;
        void write(const std::string& fileName,
            const LoopsLib::MovetkGeometryKernel::TrajectorySet&
            in) override;
        std::vector<std::string> requiredArguments() const override;
    };
    class WkbCsvTimestampedTrajectorySetProvider : public TypedSerializer<LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet>
    {
    public:
        WkbCsvTimestampedTrajectorySetProvider()
            : TypedSerializer<LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet>("Wkb Csv timestamped trajectory reader", "WKB Csv(*.wkbcsv)", "wkbcsv")
        {
        }

        void read(const std::string& fileName,
            LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& out) override;
        void write(const std::string& fileName,
            const LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet& out) override;
        std::vector<std::string> requiredArguments() const override;
    };
}
#endif