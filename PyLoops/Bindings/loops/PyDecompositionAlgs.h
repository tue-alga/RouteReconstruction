#ifndef PYLOOPS_BINDINGS_DECOMPOSITIONALGS_
#define PYLOOPS_BINDINGS_DECOMPOSITIONALGS_
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Helpers/DecompositionObject.h"
#include <LoopsAlgs/FlowDecomposition.h>
#include "Algs/Processing/TrajectorySetFrechet.h"

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
    

    template<typename AlgType,typename...PropTypes>
    void bindDecompositionAlgorithm(detail::TypeHolder<AlgType>, pybind11::module& targetModule, const std::string& algName, const detail::Property<AlgType,PropTypes>&...props)
    {
        namespace py = pybind11;
        py::class_<AlgType> clsObj(targetModule, algName.c_str());
        clsObj.def("name", &AlgType::name)
            .def(py::init<LoopsLib::Models::DecompositionResult*>())
            .def("setDecompositionResultObject", &AlgType::setDecompositionObject)
            .def_property("iterations", &AlgType::numberOfIterations,&AlgType::setNumberOfIterations)
            .def_property("logDir", &AlgType::logDir, &AlgType::setLogDir)
            .def_property("logPrefix", &AlgType::logPrefix, &AlgType::setLogPrefix)
            .def("currentDecompositionResult",&AlgType::decompositionObject,py::return_value_policy::reference)
            .def("isDone", &AlgType::isDone)
            .def("reset", &AlgType::isDone)
            .def("nextStep", &AlgType::nextStep)
            .def("fullyDecompose", &AlgType::decompose)
            .def("paramDescription", &AlgType::paramDescription);
        (props.applyTo(clsObj), ...);
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
                const LoopsAlgs::Frechet::Interval& lrInterval, const LoopsAlgs::Frechet::StrongFrechetGraphData& data){}
        };

        LoopsLib::Models::ProblemInstance* m_obj = nullptr;
        // Algs
        LoopsAlgs::FlowDecomposition::FrechetHittingDecomposition m_frechetHitting;
        LoopsAlgs::FlowDecomposition::HittingPathsDecomposition m_hitting;
        LoopsAlgs::FlowDecomposition::WeightedFrechetPathsDecomposition m_weightedFrechet;

        LoopsLib::Models::DecompositionResult runAlg(LoopsAlgs::FlowDecomposition::IFlowDecomposition* alg);
    public:
        PyDecompositionAlgs(LoopsLib::Models::ProblemInstance* obj);

        void verifyDecomposable();
        
        static void registerPy(pybind11::module& mod);
    };
}
#endif