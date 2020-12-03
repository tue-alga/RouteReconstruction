#ifndef LOOPSALGS_ALGSINTERFACE_H
#define LOOPSALGS_ALGSINTERFACE_H
#include <type_traits>
#include <string>
#include <tuple>
#include <LoopsAlgs/IFlowDecomposition.h>
#include "FlowDecomposition/ILPFlowDecomposition.h"
#include "FlowDecomposition/FrechetHittingDecomposition.h"
#include "FlowDecomposition/WeightedFrechetPathsDecomposition.h"
#include "FlowDecomposition/GlobalMinCostDecomposition.h"
#include "FlowDecomposition/ShortestPathsDecomposition.h"
#include "FlowDecomposition/MultiCommodityDecomposition.h"
#include "FlowDecomposition/FrechetDecomposition.h"
#include "FlowDecomposition/FrechetBandDecomposition.h"

namespace LoopsAlgs::FlowDecomposition
{
    template<typename Cls, typename GetterDataType, typename SetterDataType>
    struct Property
    {
        using DataType = GetterDataType;
        DataType(Cls::*Getter)() const;
        void(Cls::*Setter)(SetterDataType);
        std::string name;
        std::string label;

        static_assert(std::is_convertible_v<SetterDataType, GetterDataType>, "Incompatible getter/setter in Property");

        Property(): Getter(nullptr),Setter(nullptr) {}
        Property(const std::string& name, GetterDataType(Cls::*Getter)() const, void(Cls::*Setter)(SetterDataType))
            :name(name), Getter(Getter), Setter(Setter) {}
        Property& withLabel(const std::string& label)
        {
            this->label = label;
            return *this;
        }

        template<typename Visitor>
        void visit(Visitor&& visitor) const
        {
            visitor(name, label, Getter, Setter);
        }
    };
    template<typename Cls, typename GetterDataType, typename SetterDataType>
    auto property(const std::string& name, GetterDataType(Cls::*Getter)() const, void(Cls::*Setter)(SetterDataType))
    {
        return Property<Cls, GetterDataType,SetterDataType>(name, Getter, Setter);
    }
    template<typename Cls, typename GetterDataType, typename SetterDataType>
    auto property(const std::string& name, const std::string& label, GetterDataType(Cls::*Getter)() const, void(Cls::*Setter)(SetterDataType))
    {
        return Property<Cls, GetterDataType,SetterDataType>(name, Getter, Setter).withLabel(label);
    }

    using AllAlgs = std::tuple<
        ILPFlowDecomposition,
        FrechetHittingDecomposition,
        WeightedFrechetPathsDecomposition,
        GlobalMinCostDecomposition,
        ShortestPathsDecomposition
    >;

    template<typename Visitor, typename PropertyTuple, std::size_t...Is>
    void visitPropertyTupleElements(Visitor&& visitor, PropertyTuple&& tup, std::index_sequence<Is...>)
    {
        (std::get<Is>(tup).visit(visitor), ...);
    }

    template<typename Visitor, typename PropertyTuple>
    void visitProperties(Visitor&& visitor, PropertyTuple&& tup)
    {
        if constexpr(std::tuple_size_v<std::decay_t<PropertyTuple>> == 0){
            return;
        }
        else
        {
            visitPropertyTupleElements(visitor, tup, std::make_index_sequence<std::tuple_size_v<std::decay_t<PropertyTuple>>>{});
        }
    }

    template<typename AlgsType>
    struct AlgsInterface
    {
        // define a static auto tuple of properties in the specialization
        inline static auto Properties()
        {
            return std::tuple<>();
        }
    };
    template<> struct AlgsInterface<FlowDecomposition::IFlowDecomposition>
    {
        inline static auto Properties() {
            return std::make_tuple(
                property("numberOfIterations", "Number of iterations", &IFlowDecomposition::numberOfIterations, &IFlowDecomposition::setNumberOfIterations),
                property("pruneValue", "Prune value", &IFlowDecomposition::pruneValue, &IFlowDecomposition::setPruneValue),
                property("logDir", "Log dir", &IFlowDecomposition::logDir, &IFlowDecomposition::setLogDir),
                property("logPrefix", "Log prefix", &IFlowDecomposition::logPrefix, &IFlowDecomposition::setLogPrefix),
                property("logFilePrefix", "Log file prefix", &IFlowDecomposition::logFilePrefix, &IFlowDecomposition::setLogFilePrefix)
            );
        }
    };
    template<> struct AlgsInterface<ILPFlowDecomposition>
    {
        inline static auto Properties() {
            return std::make_tuple(
                property("timeLimit", "Time limit", &ILPFlowDecomposition::timeLimit, &ILPFlowDecomposition::setTimeLimit)
            );
        }
    };
    template<> struct AlgsInterface<FrechetHittingDecomposition>
    {
        inline static auto Properties() {
            return std::make_tuple(
                property("pathsPerTrajectory", "Paths per trajectory", &FrechetHittingDecomposition::pathsPerTrajectory, &FrechetHittingDecomposition::setPathsPerTrajectory),
                property("greedyAttachEnds", "Greedy attach ends", &FrechetHittingDecomposition::greedyAttachEnds, &FrechetHittingDecomposition::setGreedyAttachEnds)
            );
        }
    };
    template<> struct AlgsInterface<WeightedFrechetPathsDecomposition>
    {
        inline static auto Properties() {
            return std::make_tuple(
                property("noDataCaching", "No data caching", &WeightedFrechetPathsDecomposition::noDataCaching, &WeightedFrechetPathsDecomposition::setNoDataCaching),
                property("greedyAttachEnds", "Greedy attach ends", &WeightedFrechetPathsDecomposition::greedyAttachEnds, &WeightedFrechetPathsDecomposition::setGreedyAttachEnds),
                property("numberOfThreads", "Number of threads", &WeightedFrechetPathsDecomposition::numberOfThreads, &WeightedFrechetPathsDecomposition::setNumberOfThreads),
                property("maximumPerPath", "Maximum per path", &WeightedFrechetPathsDecomposition::maximumPerPath, &WeightedFrechetPathsDecomposition::setMaximumPerPath),
                property("maximumSearchTime", "Maximum search time in seconds", &WeightedFrechetPathsDecomposition::maxSearchTime, &WeightedFrechetPathsDecomposition::setMaxSearchTime)
            );
        }
    };
    template<> struct AlgsInterface<FrechetDecomposition>
    {
        inline static auto Properties() {
            return std::make_tuple(
                property("lowerBound", "Search lower bound", &FrechetDecomposition::lowerBound, &FrechetDecomposition::setLowerBound),
                property("precision", "Frechet precision", &FrechetDecomposition::precision, &FrechetDecomposition::setPrecision)
            );
        }
    };
    template<> struct AlgsInterface<FrechetBandDecomposition>
    {
        inline static auto Properties() {
            return std::make_tuple(
            );
        }
    };
    //WeightedFrechetPathsDecomposition
}
#endif