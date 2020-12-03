#ifndef LOOPSIO_TRAJECTORYSETPROVIDERS_WKTCSVTRAJECTORYSETPROVIDER_H
#define LOOPSIO_TRAJECTORYSETPROVIDERS_WKTCSVTRAJECTORYSETPROVIDER_H
#include <LoopsIO/ISerializer.h>
namespace LoopsIO::TrajectorySetProviders
{
    class WktCsvTrajectorySetProvider : public TypedSerializer<LoopsLib::MovetkGeometryKernel::TrajectorySet>
    {
    public:
        WktCsvTrajectorySetProvider()
            : TypedSerializer<LoopsLib::MovetkGeometryKernel::TrajectorySet>("Wkt Csv timestamped trajectory reader", "WKT Csv(*.wktcsv)", "wktcsv")
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
    class WktCsvTimestampedTrajectorySetProvider : public TypedSerializer<LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet>
    {
    public:
        WktCsvTimestampedTrajectorySetProvider()
            : TypedSerializer<LoopsLib::MovetkGeometryKernel::TimestampedTrajectorySet>("Wkt Csv timestamped trajectory reader", "WKT Csv(*.wktcsv)", "wktcsv")
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