#ifndef LOOPSLIB_MODELS_FLOWFIELD_H
#define LOOPSLIB_MODELS_FLOWFIELD_H
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/DS/EmbeddedGraph.h>
namespace LoopsLib::Models{

    class FlowField{
    public:
        using BasisElement_t = std::vector<DS::BaseGraph::Id_t>;

        DS::EmbeddedGraph* m_sourceGraph = nullptr;

        std::string m_graphFile;

        // Flow value per edge in the graph: E -> \Reals
        std::vector<NT> m_data;
        // Paths as edge indices
        std::vector<BasisElement_t> m_paths;
        // Value per path
        std::vector<NT> m_pathValues;

        /**
         * \brief Return the number of paths that the field is based on
         * \return The number of paths
         */
        std::size_t pathCount() const;

        const BasisElement_t& getPath(std::size_t i) const
        {
            return m_paths[i];
        }
        const NT& pathValue(std::size_t i) const
        {
            return m_pathValues[i];
        }

        /**
         * \brief Sets the field from the current paths
         * \param edgeCount Total number of edges in graph.
         */
        void setFieldFromPaths(std::size_t edgeCount);
    };
}
#endif