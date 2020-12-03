#ifndef LOOPSALGS_MULTITHREADING_HELPERS_H
#define LOOPSALGS_MULTITHREADING_HELPERS_H
#include <thread>
#include <future>
#include <LoopsLib/Helpers/Logger.h>
#include <LoopsAlgs/Memory/GetStats.h>
namespace LoopsAlgs::Multithreading
{
    /**
     * \brief Archetype of the tasks as expected by the Runner class.
     * \tparam ReturnType 
     * \tparam InputRangeContainer 
     */
    template<typename ReturnType,typename InputRangeContainer, typename LoggerType>
    class Task
    {
    protected:
        LoopsLib::Helpers::LogFactory<LoggerType> m_logger;
        int m_threadNum = -1;
    public:
        using ConstIt = decltype(std::declval<const InputRangeContainer&>().begin());
        virtual ~Task(){}

        virtual void compute(ConstIt it, std::vector<ReturnType>& output) = 0;

        virtual void init() {}
        virtual void wrapUp() {}
        virtual void onInterrupt(std::vector<ReturnType>& currentOutput) {}

        virtual std::vector<ReturnType> operator()(std::size_t threadNum, ConstIt begin, ConstIt end, std::atomic<int>& progress, std::atomic<bool>& interrupt) 
        {
            m_threadNum = threadNum;
            init();

            auto dist = std::distance(begin, end);
            std::vector<ReturnType> output;
            std::size_t processCount = 0;
            for (auto it = begin; it != end; ++it)
            {
                try
                {
                    compute(it, output);
                }
                catch (std::exception &e)
                {
                    m_logger.error("Caught exception ", e.what(), " while processing path ", *it, " in thread ", threadNum);
                }
                progress = (processCount * 100) / dist;
                ++processCount;
            }
            progress = 100;
            wrapUp();
            return output;
        }
    };

    template<typename ReturnType>
    struct Runner
    {
        // Number of threads to use.
        int m_parallellism = 8;

        std::function<void(int, int)> m_progressCb;

        void setProgressCallback(std::function<void(int, int)> progress)
        {
            m_progressCb = progress;
        }

        /**
         * \brief Applies the tasks in parallel and applies the handler to the resulting objects
         * \tparam TaskTypeCreator Creator for a task. Should return some callable that follows the operator() arguments of Task<ReturnType, InputRangeContainer>
         * \tparam InputContainer The container for a range of inputs to be processed in parallel
         * \tparam ResultHandler Handler for the generated results
         * \param taskCreator The creator for the task.
         * \param input The input container
         * \param progressCb Progress callback. Is provided with thread number and progress in the range [0,100]
         * \param handler Handler for the returned ReturnType elements
         */
        template<typename TaskTypeCreator, typename InputContainer, typename ResultHandler>
        void run(TaskTypeCreator taskCreator, const InputContainer& input, ResultHandler& handler)
        {
            using TaskType = std::decay_t<decltype(taskCreator())>;
            using ConstIt = decltype(input.begin());

            static_assert(std::is_same_v <
                ReturnType, 
                decltype(
                    std::declval<TaskType&>().operator()(
                        std::declval<std::size_t>(), 
                        std::declval<ConstIt>(), 
                        std::declval<ConstIt>(), 
                        std::declval<std::atomic<int>&>(), 
                        std::declval<std::atomic_bool&>()
                    )
                )>
                ,"Invalid operator() on task, should be callable with std::size_t, ConstIt, ConstIt, std::atomic<int>&, std::atomic_bool&");


            std::list<std::atomic<int>> progress;


            std::size_t elsPerThread = input.size() / m_parallellism;
            std::size_t remainder = input.size() - elsPerThread * m_parallellism;

            // Split range

            //Create tasks
            std::vector<std::future<ReturnType>> futures;
            std::vector<std::thread> threads;

            ConstIt curr = input.begin();
            ConstIt end = input.begin();

            // Interrupt flag
            std::atomic_bool interrupt = false;

            std::cout << "Current memory usage " << LoopsAlgs::Memory::getCurrentRSS() / 1024 / 1024 << " MB " << std::endl;

            for (std::size_t i = 0; i < m_parallellism; ++i)
            {
                // Construct task
                std::packaged_task<ReturnType(std::size_t, ConstIt, ConstIt, std::atomic<int>&, std::atomic_bool&)> task(std::move(taskCreator()));
                // Create the future for the result
                futures.push_back(task.get_future());
                progress.emplace_back(0);
                end = std::next(curr, elsPerThread + (i < remainder ? 1 : 0));
                std::atomic<int>& prog = progress.back();
                threads.emplace_back(std::move(task), i, curr, end, std::ref(prog), std::ref(interrupt));

                // Update current iterator
                curr = end;
            }
            std::cout << "Memory usage after thread start " << LoopsAlgs::Memory::getCurrentRSS() / 1024 / 1024 << " MB " << std::endl;
            while (true)
            {
                std::size_t i = 0;
                std::size_t doneCount = 0;
                for (auto& el : progress)
                {
                    const auto prog = el.load();
                    // Call progress callback if provided
                    if(m_progressCb) m_progressCb(i, prog);

                    ++i;
                    if (prog >= 100) ++doneCount;
                }
                std::cout << "Current memory usage " << LoopsAlgs::Memory::getCurrentRSS() / 1024 / 1024 << " MB " << std::endl;
                if (doneCount == progress.size())
                {
                    for (auto& t : threads) t.join();
                    break;
                }
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(2s);
            }
            // Return combined outputs
            for (auto& fut : futures)
            {
                handler(fut.get());
            }
        }
    };

}
#endif