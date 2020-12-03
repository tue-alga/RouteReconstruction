#include <LoopsIO/DecompositionIO.h>

void LoopsIO::DecompositionIO::updateFilter()
{
    std::stringstream filter;
    if (m_providers.size() == 0)return;
    filter << m_providers[0]->fileFilter();
    for (int i = 1; i < m_providers.size(); ++i)
    {
        filter << ";;" << m_providers[i]->fileFilter();
    }
    m_filter = filter.str();
}

LoopsIO::DecompositionIO& LoopsIO::DecompositionIO::inst()
{
    static DecompositionIO el;
    return el;
}

LoopsIO::DecompositionIO::DecompositionIO()
{
}

LoopsIO::DecompositionIO::~DecompositionIO()
{
    for (auto provider : m_providers)
    {
        delete provider;
    }
    m_providers.clear();
    m_fileFilterMap.clear();
}

void LoopsIO::DecompositionIO::registerProvider(IDecompositionProvider* provider)
{
    inst().m_providers.push_back(provider);
    inst().m_fileFilterMap[provider->fileFilter()] = provider;
    inst().updateFilter();
}

std::string LoopsIO::DecompositionIO::fileFilter()
{
    return inst().m_filter;
}

void LoopsIO::DecompositionIO::read(const std::string& filter, const std::string& path, LoopsLib::Helpers::DecompositionObject& target,
                               std::function<bool(const std::string&)> dependentMapCallback,
                               std::function<bool(const std::string&)> dependentFieldCallback)
{
    if (inst().m_fileFilterMap.find(filter) == inst().m_fileFilterMap.end()) throw std::runtime_error(
        "Could not find filter");
    inst().m_fileFilterMap[filter]->read(path, target, dependentMapCallback, dependentFieldCallback);
}

void LoopsIO::DecompositionIO::read(const std::string& path, LoopsLib::Helpers::DecompositionObject& target,
                               std::function<bool(const std::string&)> dependentMapCallback,
                               std::function<bool(const std::string&)> dependentFieldCallback)
{
    auto pos = path.find_last_of('.');
    auto ext = path.substr(pos + 1);
    for (auto* prov : inst().m_providers)
    {
        if (prov->ext() == ext)
        {
            prov->read(path, target, dependentMapCallback, dependentFieldCallback);
            return;
        }
    }
    throw std::runtime_error(std::string("No provider for given extension ") + ext);
}

void LoopsIO::DecompositionIO::write(const std::string& filter, const std::string& path,
                                const LoopsLib::Helpers::DecompositionObject& target)
{
    if (inst().m_fileFilterMap.find(filter) == inst().m_fileFilterMap.end()) throw std::runtime_error(
        "Could not find filter");
    inst().m_fileFilterMap[filter]->write(path, target);
}

void LoopsIO::DecompositionIO::write(const std::string& path, const LoopsLib::Helpers::DecompositionObject& target)
{
    auto pos = path.find_last_of('.');
    auto ext = path.substr(pos + 1);
    for (auto* prov : inst().m_providers)
    {
        //std::cout << "Prov ext: " << prov->ext() << ";" << std::endl;
        if (prov->ext() == ext)
        {
            prov->write(path, target);
            return;
        }
    }
    throw std::runtime_error("Could not find appropriate writer for extension " + ext);
}
