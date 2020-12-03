#ifndef PYLOOPS_RANGEJOBPOOL_H
#define PYLOOPS_RANGEJOBPOOL_H
#include <mutex>
#include <thread>
namespace PyLoops{

    class IRangeJob{
    protected:
        
        public:

    };

    class RangeJobPool{
        int m_poolSize;
    public:
        RangeJobPool(int poolSize) : m_poolSize(poolSize){}
            
    }
}
#endif