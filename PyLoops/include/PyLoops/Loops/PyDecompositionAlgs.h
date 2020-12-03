#ifndef PYLOOPS_BINDINGS_DECOMPOSITIONALGS_
#define PYLOOPS_BINDINGS_DECOMPOSITIONALGS_
#include <PyLoops/PyLoops.inc.h>
#include <pybind11/stl.h>
#include <LoopsLib/Helpers/DecompositionObject.h>
#include <LoopsAlgs/FlowDecomposition.h>
#include <LoopsLib/Algs/Processing/TrajectorySetFrechet.h>
#include <LoopsAlgs/AlgsInterface.h>


namespace PyLoops::Bindings
{
    /**
     * \brief Custom exception for describing decomposition errors
     */
    class DecompositionException : public std::exception
    {
        std::string m_msg;
    public:
        DecompositionException(const std::string& msg);

        char const* what() const noexcept override;
    };

    namespace detail
    {
        template<typename Cls>
        struct TypeHolder
        {
            using type = Cls;
        };
        template<typename Cls, typename SetterDataType>
        struct Property
        {
            using DataType = std::decay_t<SetterDataType>;
            DataType(Cls::*Getter)() const;
            void(Cls::*Setter)(SetterDataType);
            std::string name;

            Property(){}
            Property(const std::string& name, DataType(Cls::*Getter)() const, void(Cls::*Setter)(SetterDataType))
                :name(name),Getter(Getter),Setter(Setter){}

            template<typename ClsType>
            void applyTo(ClsType& cls) const
            {
                cls.def_property(name.c_str(), Getter, Setter);
            }
        };
    }

    template<typename AlgType>
    struct PropVisitor
    {
        pybind11::class_<AlgType>& classObj;

        PropVisitor(pybind11::class_<AlgType>& classObj) :classObj(classObj) {}

        template<typename Getter, typename Setter>
        void operator()(const std::string& name, const std::string& label, Getter getter, Setter setter)
        {
            classObj.def_property(name.c_str(), getter, setter);
        }
    };
    template<typename AlgType>
    struct PropDictCreateVisitor
    {
        AlgType* m_targetAlg;

        pybind11::dict* m_propDict;

        PropDictCreateVisitor(AlgType* targetAlg, pybind11::dict* propDict) :m_targetAlg(targetAlg), m_propDict(propDict) {}

        template<typename Getter, typename Setter>
        void operator()(const std::string& name, const std::string& label, Getter getter, Setter setter)
        {
            auto& dict = *m_propDict;
            dict[name.c_str()] = std::invoke(getter, m_targetAlg);
        }
    };

    template<typename AlgType,typename...PropTypes>
    void bindDecompositionAlgorithm(detail::TypeHolder<AlgType>, pybind11::module& targetModule, const std::string& algName)
    {
        namespace py = pybind11;
        py::class_<AlgType> clsObj(targetModule, algName.c_str());
        clsObj.def("name", &AlgType::name)
            .def(py::init<LoopsLib::Models::DecompositionResult*>())
            .def("setDecompositionResultObject", &AlgType::setDecompositionObject)
            /*.def_property("iterations", &AlgType::numberOfIterations,&AlgType::setNumberOfIterations)
            .def_property("logDir", &AlgType::logDir, &AlgType::setLogDir)
            .def_property("logPrefix", &AlgType::logPrefix, &AlgType::setLogPrefix)*/
            .def("currentDecompositionResult",&AlgType::decompositionObject,py::return_value_policy::reference)
            .def("isDone", &AlgType::isDone)
            .def("reset", &AlgType::isDone)
            .def("nextStep", &AlgType::nextStep)
            .def("fullyDecompose", &AlgType::decompose)
            .def("paramDescription", &AlgType::paramDescription);
        
        PropVisitor<AlgType> visitor(clsObj);
        // Add base flow decomposition props
        auto baseProps = LoopsAlgs::FlowDecomposition::AlgsInterface<LoopsAlgs::FlowDecomposition::IFlowDecomposition>::Properties();
        LoopsAlgs::FlowDecomposition::visitProperties(visitor, baseProps);
        // Add alg specific props
        auto algProps = LoopsAlgs::FlowDecomposition::AlgsInterface<AlgType>::Properties();
        LoopsAlgs::FlowDecomposition::visitProperties(visitor, algProps);
        clsObj.def("propertiesDict",[baseProps, algProps](AlgType& alg)
        {
            py::dict dict;
            PropDictCreateVisitor<AlgType> dictCreator(&alg, &dict);
            LoopsAlgs::FlowDecomposition::visitProperties(dictCreator, baseProps);
            LoopsAlgs::FlowDecomposition::visitProperties(dictCreator, algProps);
            return dict;
        });
    }

    class PyDecompositionAlgs
    {
        struct Tester
        {
            LoopsLib::DS::EmbeddedGraph* m_graph;
            Tester(LoopsLib::DS::EmbeddedGraph* graph):m_graph(graph){}

            void testReachability(const std::vector<LoopsLib::DS::BaseGraph::Id_t>& path, LoopsLib::NT epsilon);
            LoopsAlgs::Frechet::WenkSweeplineResult beforeEdgeProcess(LoopsLib::DS::BaseGraph::Edge* edge, 
                const LoopsAlgs::Frechet::PrioQueueNode& node, 
                const LoopsAlgs::Frechet::Interval& lrInterval, const LoopsAlgs::Frechet::StrongFrechetGraphData& data)
            {
                return LoopsAlgs::Frechet::WenkSweeplineResult::ContinueIt;
            }
        };

        LoopsLib::Models::ProblemInstance* m_obj = nullptr;
        // Algs
        LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition m_frechetHitting;
        LoopsAlgs::FlowDecomposition::HittingPathsDecomposition m_hitting;
        LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition m_weightedFrechet;
        LoopsAlgs::FlowDecomposition::ShortestPathsDecomposition m_shortestPathsDecomp;

        LoopsLib::Models::DecompositionResult runAlg(LoopsAlgs::FlowDecomposition::IFlowDecomposition* alg);

        template<typename AlgType>
        static void bindDecomposer(pybind11::module& mod, const std::string& name, std::vector<std::string>& algorithms)
        {
            static_assert(std::is_base_of_v<LoopsAlgs::FlowDecomposition::IFlowDecomposition, AlgType>, "Can only bind decomposer that inherits IFlowDecomposition");
            bindDecompositionAlgorithm(detail::TypeHolder<AlgType>{}, mod, name);
            algorithms.push_back(name);
        }
    public:
        PyDecompositionAlgs(LoopsLib::Models::ProblemInstance* obj);

        void verifyDecomposable();
        
        static void registerPy(pybind11::module& mod);
    };
}
#endif