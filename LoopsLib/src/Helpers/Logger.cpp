#include <LoopsLib/Helpers/Logger.h>

void LoopsLib::Helpers::LockedStream::registerStream(std::ostream* stream)
{
    auto& inst = instance();
    inst.mutexMapUpdateLock.lock();
    if (inst.mutexMap.find(stream) != inst.mutexMap.end())
    {
        inst.mutexMapUpdateLock.unlock();
        return;
    }
    inst.mutexMap[stream] = std::make_unique<std::mutex>();
    inst.mutexMapUpdateLock.unlock();
}

void LoopsLib::Helpers::LockedStream::unregisterStream(std::ostream* stream)
{
    auto& inst = instance();
    inst.mutexMapUpdateLock.lock();
    if (inst.mutexMap.find(stream) != inst.mutexMap.end())
    {
        inst.mutexMap.erase(stream);
    }
    inst.mutexMapUpdateLock.unlock();
}

void LoopsLib::Helpers::LockedStream::assignDeferredLock(std::ostream* stream, std::unique_lock<std::mutex>& target)
{
    auto& inst = instance();
    inst.mutexMapUpdateLock.lock_shared();
    auto* mut = inst.mutexMap.at(stream).get();
    target = std::move(std::unique_lock<std::mutex>(*mut, std::defer_lock));
    inst.mutexMapUpdateLock.unlock_shared();
}
