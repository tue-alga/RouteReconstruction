#ifndef ALGS_LONGESTPATH_ILONGESTPATH_H
#define ALGS_LONGESTPATH_ILONGESTPATH_H
#include <Eigen/Eigen>
#include <string>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Math/Vector.h>

namespace LoopsLib::Algs::LongestPath
{
    class ILongestPath
    {
    protected:
        // Name of the alg
        std::string m_name;
        // Target graph to use
        DS::BaseGraph* m_graph;
        // Source and sinks
        DS::BaseGraph::Id_t m_source, m_sink;

        virtual Algs::BasisElement computeLongestPath(const FieldType& field, NT maxWeight) = 0;

        /**
         * \brief Clear any internal state, possibly needed for retries.
         */
        virtual void clear() {}

        /**
         * \brief A provided default implementation for retrying the algorithm. Adds noise with uniform values between
         * [0,percentageofMaxFieldVal * maxFieldVal]. 
         * \param field The field
         * \param percentageOfMaxFieldVal The percentage of the maximum coefficient to use
         * \return The retried longest path.
         */
        virtual BasisElement retryWithNoise(const FieldType& field, double percentageOfMaxFieldVal)
        {
            const auto offset = Math::VectorWrapper<NT>::Random(field.size(), -1.0, 1.0);
            auto offsetVec = Math::wrapVec(offset);
            auto fieldVec = Math::wrapVec(field);
            auto newVec = fieldVec + percentageOfMaxFieldVal * fieldVec.maxCoeff() * offsetVec;
            return computeLongestPath(newVec.toStd(), std::numeric_limits<double>::max());
        }
    public:
        ILongestPath(const std::string& name, DS::BaseGraph* graph):
        m_name(name),
        m_graph(graph),
        m_source(std::numeric_limits<decltype(m_source)>::max()),
        m_sink(std::numeric_limits<decltype(m_source)>::max())
        {}

        virtual ~ILongestPath(){}

        void setGraph(DS::BaseGraph* graph)
        {
            m_graph = graph;
        }
        void setName(const std::string& name)
        {
            m_name = name;
        }
        std::string name() const
        {
            return m_name;
        }
        /**
         * \brief Computes the longest path approximately.
         * \param graph Directed graph to use
         * \param field Weights for the graph edges
         * \param source Index of the source vertex
         * \param sink Index of the sink vertex
         * \return Vector representation of approximate longest path. (for each edge, 1  if it is in the path, 0 otherwise).
         */
        virtual BasisElement compute(const FieldType& weights, DS::BaseGraph::Id_t src, DS::BaseGraph::Id_t sink)
        {
            m_source = src;
            m_sink = sink;
            clear();

            return this->computeLongestPath(weights, std::numeric_limits<double>::max());
        }
        virtual std::vector<BasisElement> compute(const FieldType& weights, DS::BaseGraph::Id_t src, DS::BaseGraph::Id_t sink, std::size_t kLargest)
        {
            std::vector<BasisElement> ret;
            std::vector<NT> weightsCopy = weights;
            ret.push_back(compute(weights,src,sink));
            for(std::size_t i = 1; i < kLargest; ++i)
            {
                ret.push_back(retry(weights));
            }
            return ret;
        }

        virtual BasisElement retry(const FieldType& field) = 0;

    };
}
#endif