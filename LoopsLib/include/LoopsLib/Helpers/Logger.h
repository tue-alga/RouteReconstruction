#ifndef HELPERS_LOGGER_H
#define HELPERS_LOGGER_H
#include <string>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace LoopsLib::Helpers
{

    enum LogLevel
    {
        Level4 = 96,
        Level3 = 97,
        Level2 = 98,
        Level1 = 99,
        Level0 = 100,
        Trace = 250,
        Debug = 300,
        Deep = 400,
        // Normal levels down below
        Info = 600,
        Warn = 800,
        Error = 1000
    };


    /**
     * \brief Specify default log level. Specialize to overwrite
     * \tparam T The type for which to log
     * \return The loglevel
     */
    template<typename T>
    constexpr int logLevel()
    {
        return LogLevel::Info;
    }


    template<typename It>
    struct RangeLogValue
    {
        It begin, end;
        std::string sep;
        RangeLogValue(It begin, It end, const std::string& sep):begin(begin), end(end), sep(sep)
        {

        }
    };

    struct LockedStream
    {
    private:
        LockedStream(){}
        static LockedStream& instance()
        {
            static LockedStream inst;
            return inst;
        }

        std::unordered_map<std::ostream*, std::unique_ptr<std::mutex>> mutexMap;
        std::shared_mutex mutexMapUpdateLock;
    public:



        static void registerStream(std::ostream* stream);

        static void unregisterStream(std::ostream* stream);

        static void assignDeferredLock(std::ostream* stream, std::unique_lock<std::mutex>& target);

        struct LockedStreamWriter
        {
            std::ostream* m_stream;
            // Goes out of scope after object deletion
            std::unique_lock<std::mutex> m_guard;
            LockedStreamWriter(std::ostream* stream): m_stream(stream)
            {
                LockedStream::assignDeferredLock(stream, m_guard);
                m_guard.lock();
            }
            
            template<typename T>
            LockedStreamWriter& operator<<(const T& t)
            {
                std::ostream& s = *m_stream;
                s << t;
                return *this;
            }
            template<typename It>
            LockedStreamWriter& operator<<(const RangeLogValue<It>& t)
            {
                std::ostream& s = *m_stream;
                for(auto it = t.begin; it != t.end; ++it)
                {
                    if (it != t.begin) s << t.sep;
                    s << *it;
                }
                return *this;
            }
            LockedStreamWriter& operator<<(std::ostream&(*Func)(std::ostream&))
            {
                std::ostream& s = *m_stream;
                s << Func;
                return *this;
            }
            template<typename...Ts>
            void logMultiple(Ts...args)
            {
                using expander = int[];
                (void)expander{ (*this << args, 0)... };
            }
        };
    };

    struct OutStreamMultiplexer
    {
        std::vector<std::ostream*> streams;


        template<typename T>
        OutStreamMultiplexer& operator<<(const T& t)
        {
            for (auto* str : streams) {
                LockedStream::LockedStreamWriter writer(str);
                writer << t;
            }
            return *this;
        }
        OutStreamMultiplexer& operator<<(std::ostream&(*Func)(std::ostream&))
        {
            for (auto* str : streams) {
                LockedStream::LockedStreamWriter writer(str);
                writer << Func;
            }
            return *this;
        }
        template<typename...Ts>
        void logMultiple(Ts...args)
        {
            using expander = int[];
            for (auto* str : streams) {
                LockedStream::LockedStreamWriter writer(str);
                (void)expander {
                    (writer << args, 0)...
                };
            }
            
        }
    };

    template<typename T>
    struct ClassLogLevel
    {
        static constexpr int LogLevel = logLevel<T>();
        static std::string Prefix() {
            static std::string pref = typeid(T).name();
            return pref;
        }
    };

    template<typename LoggerReference, std::size_t Level>
    struct Logger
    {
        std::string indent = "  ";
        std::string prefix= "";

        OutStreamMultiplexer localStream;

        OutStreamMultiplexer& getStream()
        {
            return localStream;
        }
                        
        template<typename...Args>
        void logIndented(int indents, Args...args)
        {
            if constexpr (Level >= LoggerReference::LogLevel) {
                std::string indentStr;
                for (int i = 0; i < indents; ++i) indentStr += indent;
                logln(indentStr, args...);
            }
        }

        template<typename...Args>
        void log(Args...args)
        {
            if constexpr (Level >= LoggerReference::LogLevel) {
                auto& str = getStream();
                str.logMultiple("[", LoggerReference::Prefix(), "] ", prefix, args...);
            }
        }
        template<typename...Args, std::size_t...Is>
        void log(std::tuple<Args...> args, std::index_sequence<Is...>)
        {
            log(std::get<Is>(args)...);
        }
        template<typename It, typename Lambda>
        void logRange(It begin, It end, Lambda dataProvider, char separator = ',')
        {
            if constexpr (Level >= LoggerReference::LogLevel) {
                for (auto it = begin; it != end; ++it)
                {
                    getStream() << dataProvider(*it);
                    if (it + 1 != end) getStream() << separator;
                }
                getStream() << std::endl;
            }
        }
        template<typename It>
        void logRange(It begin, It end, char separator = ',')
        {
            if constexpr (Level >= LoggerReference::LogLevel) {
                for (auto it = begin; it != end; ++it)
                {
                    getStream() << *it;
                    if (it + 1 != end) getStream() << separator;
                }
                getStream() << std::endl;
            }
        }
        template<typename...Args>
        void logln(Args...args)
        {
            if constexpr (Level >= LoggerReference::LogLevel) {
                log(args..., std::endl<std::ostream::char_type, std::ostream::traits_type>);
            }
        }
    };

    template<typename ClassLogLevelT, std::size_t Level>
    inline Logger<ClassLogLevelT, Level>& logger()
    {
        static thread_local Logger<ClassLogLevelT, Level> log = {};
        return log;
    }
    template<std::size_t Level>
    std::string logPrefix()
    {
        return "";
    }
    template<std::size_t Level>
    std::string logPostfix()
    {
        return "";
    }
    template<>
    inline std::string logPrefix<LogLevel::Level0>()
    {
        return "";
    }
    template<>
    inline std::string logPrefix<LogLevel::Level1>()
    {
        return "\t";
    }
    template<>
    inline std::string logPrefix<LogLevel::Level2>()
    {
        return "\t\t";
    }
    template<>
    inline std::string logPrefix<LogLevel::Level3>()
    {
        return "\t\t\t";
    }
    template<>
    inline std::string logPrefix<LogLevel::Level4>()
    {
        return "\t\t\t\t";
    }
    template<>
    inline std::string logPrefix<LogLevel::Warn>()
    {
        return "\u001b[43;1mWARNING:";
    }
    template<>
    inline std::string logPrefix<LogLevel::Error>()
    {
        return "\u001b[41;1m!!ERROR!!:";
    }
    template<>
    inline std::string logPostfix<LogLevel::Warn>()
    {
        return "\u001b[0m";
    }
    template<>
    inline std::string logPostfix<LogLevel::Error>()
    {
        return "\u001b[0m";
    }
    template<typename T>
    struct LogFactory
    {
        static inline std::mutex m_flushMutex;
        using LoggerData = ClassLogLevel<T>;
        static thread_local inline std::vector<std::string> GlobalPrefix = {};

        struct PrefixGuard
        {
            bool m_shouldPop = true;
            PrefixGuard(const std::string& prefix)
            {
                GlobalPrefix.push_back(prefix);
            }
            PrefixGuard& operator=(PrefixGuard&& other)
            {
                other.m_shouldPop = false;
                return *this;
            }
            PrefixGuard& operator=(const PrefixGuard& other) = delete;

            ~PrefixGuard()
            {
                if(m_shouldPop)GlobalPrefix.pop_back();
            }
            void restore()
            {
                if (!m_shouldPop) return;
                GlobalPrefix.pop_back();
                m_shouldPop = false;
            }
        };

        PrefixGuard addGlobalPrefix(const std::string& prefix)
        {
            return PrefixGuard(prefix);
        }

        LogFactory()
        {
        }
        void registerStream(std::ostream* stream)
        {
            LockedStream::registerStream(stream);
        }
        void unregisterStream(std::ostream* stream)
        {
            LockedStream::unregisterStream(stream);
        }
        template<int Level>
        Logger<LoggerData,Level>& getRawLogger()
        {
            return logger<LoggerData, Level>();
        }

        template<int Level>
        void initStreams()
        {
            auto& rawLogger = getRawLogger<Level>();
            // Already something set, so done
            if (!rawLogger.localStream.streams.empty()) return;

            LockedStream::registerStream(&std::cout);
            rawLogger.localStream.streams.push_back(&std::cout);
        }
        template<int Level,typename...Streams>
        void addLevelOutput(std::initializer_list<std::ostream*> streams)
        {
            initStreams<Level>();

            auto& rawLogger = getRawLogger<Level>();
            for(auto str : streams)
            {
                LockedStream::registerStream(str);
                rawLogger.localStream.streams.push_back(str);
            }
        }
        template<int Level>
        void setLevelOutput(std::ostream* stream)
        {
            auto& rawLogger = getRawLogger<Level>(); 
            rawLogger.localStream.streams.clear();
            LockedStream::registerStream(stream);
            rawLogger.localStream.streams.push_back(stream);
        }
        template<int Level>
        void setLevelOutputs(std::initializer_list<std::ostream*> streams)
        {
            auto& rawLogger = getRawLogger<Level>();
            rawLogger.localStream.streams.clear();
            for (auto str : streams)
            {
                LockedStream::registerStream(str);
                rawLogger.localStream.streams.push_back(str);
            }
        }
        template<int Level>
        void resetLevelOutput()
        {
            auto& rawLogger = getRawLogger<Level>();
            rawLogger.localStream.streams.clear();
            initStreams<Level>();
        }
        template<int...Levels>
        void resetLevelsOutput()
        {
            (resetLevelOutput<Levels>(), ...);
        }

        template<std::size_t Level, typename...Args>
        void logToLevel(std::integral_constant<std::size_t, Level>, Args... args)
        {
            if constexpr (Level >= LoggerData::LogLevel) {
                initStreams<Level>();
                auto& rawLogger = getRawLogger<Level>();
                rawLogger.logln(RangeLogValue(GlobalPrefix.begin(), GlobalPrefix.end(),""), logPrefix<Level>(), args..., logPostfix<Level>());
            }
        }
#define LOG_TO_LEVEL(name, constName)template<typename...Args>\
        void name(Args...args)\
        {\
            logToLevel(std::integral_constant<std::size_t, LogLevel::constName>{}, args...);\
        }
        LOG_TO_LEVEL(log0, Level0)
        LOG_TO_LEVEL(log1, Level1)
        LOG_TO_LEVEL(log2, Level2)
        LOG_TO_LEVEL(log3, Level3)
        LOG_TO_LEVEL(log4, Level4)
        LOG_TO_LEVEL(deep, Deep)
        LOG_TO_LEVEL(info, Info)
        LOG_TO_LEVEL(debug, Debug)
        LOG_TO_LEVEL(error, Error)
        LOG_TO_LEVEL(trace, Trace)
        LOG_TO_LEVEL(warn, Warn)
#undef LOG_TO_LEVEL
        template<typename It, typename Lambda>
        void infoRange(It begin, It end, Lambda dataProvider, char separator=',')
        {
            initStreams<LogLevel::Info>();
            auto& rawLogger = getRawLogger<LogLevel::Info>();
            rawLogger.logRange(begin,end,dataProvider,separator);
        }
        template<typename It>
        void infoRange(It begin, It end, char separator = ',')
        {
            initStreams<LogLevel::Info>();
            auto& rawLogger = getRawLogger<LogLevel::Info>();
            rawLogger.logRange(begin, end, separator);
        }
        void flush()
        {
            std::lock_guard<std::mutex> lockG(m_flushMutex);
            std::cout.flush();
        }
    };

    template<typename T>
    using remove_all_t = std::remove_const_t<std::remove_reference_t<std::remove_pointer_t<std::decay_t<T>>>>;

    template<typename T>
    LogFactory<remove_all_t<T>> logFactory(T)
    {
        return LogFactory<remove_all_t<T>>();
    }
}
#endif