#ifndef LOOPSLIB_MODELS_TRAJECTORYTYPES_H
#define LOOPSLIB_MODELS_TRAJECTORYTYPES_H
//#include <movetk/geom/GeometryInterface>
#include <LoopsLib/Algs/Types.h>
#include <tuple>
namespace LoopsLib::Models
{
    template<typename DatumDataTuple, typename TypeTagsTuple, typename TrajectoryData>
    struct Trajectory
    {
        
    };
    namespace Tags
    {
        struct XCoord_t;
        struct YCoord_t;
        struct Time_t;
        struct Velocity_t;
        struct VelocityX_t;
        struct VelocityY_t;
        struct AccelerationX_t;
        struct AccelerationY_t;
        struct Heading_t;
    }
    // Todo: based on kernel
    namespace DatumTuples
    {
        //P = location, T = time, V = velocity, H heading
        using P2 = std::tuple<NT, NT>;
        using P2_t = std::tuple<Tags::XCoord_t, Tags::YCoord_t>;
        
        using P2T = std::tuple<NT, NT, std::size_t>;
        using P2T_t = std::tuple<Tags::XCoord_t, Tags::YCoord_t,Tags::Time_t>;

        using P2V1T = std::tuple<NT, NT, NT, std::size_t>;
        using P2V1T_t = std::tuple<Tags::XCoord_t, Tags::YCoord_t, Tags::Velocity_t, Tags::Time_t>;

        using P2V1HT = std::tuple<NT, NT, NT, std::size_t>;
        using P2V1HT_t = std::tuple<Tags::XCoord_t, Tags::YCoord_t, Tags::Velocity_t, Tags::Heading_t, Tags::Time_t>;

        using P2V2T = std::tuple<NT, NT, NT, NT, std::size_t>;
        using P2V2T_t = std::tuple<Tags::XCoord_t, Tags::YCoord_t,Tags::VelocityX_t, Tags::VelocityY_t, Tags::Time_t>;
    }
}
#endif