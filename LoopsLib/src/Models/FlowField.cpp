#include <LoopsLib/Models/FlowField.h>

std::size_t LoopsLib::Models::FlowField::pathCount() const
{
    return m_paths.size();
}

void LoopsLib::Models::FlowField::setFieldFromPaths(std::size_t edgeCount)
{
    m_data.clear();
    m_data.resize(edgeCount, 0);
    for (std::size_t i = 0; i < m_paths.size(); ++i)
    {
        for (const auto& e : m_paths[i])
        {
            m_data[e] += m_pathValues[i];
        }
    }
}
