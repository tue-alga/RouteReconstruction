#ifndef LOOPSALGS_MEMORY_GETSTATS_H
#define LOOPSALGS_MEMORY_GETSTATS_H
#include <cstdlib>
namespace LoopsAlgs::Memory
{

    std::size_t getPeakRSS();

    std::size_t getCurrentRSS();
}
#endif