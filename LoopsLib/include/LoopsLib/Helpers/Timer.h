#ifndef HELPERS_TIMER_H
#define HELPERS_TIMER_H
#include <chrono>
namespace LoopsLib::Helpers
{
    class Timer
    {
        using Clock = std::chrono::high_resolution_clock;
        std::chrono::time_point<Clock> m_start;
        std::chrono::time_point<Clock> m_end;
        bool m_running = false;
        bool m_wasStarted = false;
    public:
        void start()
        {
            m_start = Clock::now();
            m_running = true;
            m_wasStarted = true;
        }
        void stop()
        {
            m_end = Clock::now();
            m_running = false;
        }
        void reset()
        {
            m_running = false;
            m_wasStarted = false;
        }
        double elapsedMs() const
        {
            if (!m_wasStarted) return 0;

            std::chrono::duration<double, std::milli> diff = m_running ? Clock::now() - m_start : m_end-m_start;
            return diff.count();
        }
    };
    class ScopeTimer
    {
        double* m_target;
        Timer m_timer;
    public:
        ScopeTimer(double* timeAddingTarget): m_target(timeAddingTarget)
        {
            m_timer.start();
        }
        ~ScopeTimer()
        {
            m_timer.stop();
            *m_target += m_timer.elapsedMs();
        }
    };
}
#endif
