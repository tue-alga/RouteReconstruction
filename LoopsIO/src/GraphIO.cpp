#include <LoopsIO/GraphIO.h>
#include <fstream>
#include "LoopsIO/IOHelpers.h"

using namespace LoopsLib;

void LoopsIO::GraphIO::updateFileFilter()
{
    std::stringstream ss;
    if (m_providers.size() == 0) return;
    ss << m_providers[0]->fileFilter();
    for (int i = 1; i < m_providers.size(); ++i)
    {
        ss << ";;" << m_providers[i]->fileFilter();
    }
    m_fileFilter = ss.str();
}

LoopsIO::GraphIO& LoopsIO::GraphIO::inst()
{
    static GraphIO el;
    return el;
}

LoopsIO::GraphIO::GraphIO()
{
}

LoopsIO::GraphIO::~GraphIO()
{
    for (auto prov : m_providers)
    {
        delete prov;
    }
    m_providers.clear();
    m_filterMap.clear();
}

std::string LoopsIO::GraphIO::fileFilter()
{
    return inst().m_fileFilter;
}

void LoopsIO::GraphIO::registerProvider(IMapProvider* provider)
{
    inst().m_providers.push_back(provider);
    inst().m_filterMap[provider->fileFilter()] = provider;
    inst().m_extensionMap[provider->extension()] = provider;
    inst().updateFileFilter();
}

bool LoopsIO::GraphIO::hasMissingArguments(const std::string& filePath, const std::vector<std::string>& requiredArgs,
                                           std::vector<std::string>& missingArgs)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(filePath, path, args);
    for(const auto& arg: requiredArgs)
    {
        if(args.find(arg) == args.end())
        {
            missingArgs.push_back(arg);
        }
    }
    return !missingArgs.empty();
}

bool LoopsIO::GraphIO::hasMissingArguments(const std::string& uri, std::vector<std::string>& missingArgs)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(uri, path, args);

    auto dotIndex = path.rfind('.');
    auto ext = path.substr(dotIndex + 1);
    if(inst().m_extensionMap.find(ext) == inst().m_extensionMap.end())
    {
        std::cout << "Could not find provider for extension " + ext << std::endl;
        throw std::runtime_error("Could not find provider for extension " + ext);
    }
    // TODO Wrong index: throw something
    auto* prov = inst().m_extensionMap.at(ext);
    return hasMissingArguments(uri, prov->requiredParameters(), missingArgs);
}

void LoopsIO::GraphIO::read(const std::string& filter, const std::string& filePath, DS::EmbeddedGraph& decompObj)
{
    if(filter.empty())
    {
        read(filePath, decompObj);
    }
    else
    {
        inst().m_filterMap[filter]->read(filePath, decompObj);
    }
}

void LoopsIO::GraphIO::read(const std::string& filePath, DS::EmbeddedGraph& decompObj)
{
    auto& io = inst();
    std::string path;
    IO::IOHelpers::UrlEncoder::extractPath(filePath, path);
    // TODO check extension exists
    std::string ext = path.substr(path.rfind('.')+1);
    if(io.m_extensionMap.find(ext) == io.m_extensionMap.end())
    {
        throw std::runtime_error("No map provider was applicable for the given filepath");
    }
    io.m_extensionMap.at(ext)->read(filePath, decompObj);
}

void LoopsIO::GraphIO::write(const std::string& filter, const std::string& filePath,
                        const DS::EmbeddedGraph& decompObj)
{
    inst().m_filterMap[filter]->write(filePath, decompObj);
}

void LoopsIO::GraphIO::write(const std::string& filePath, const DS::EmbeddedGraph& decompObj)
{
    auto ext = ::IO::IOHelpers::extension(filePath);
    auto& io = inst();
    for (auto* prov : io.m_providers)
    {
        if (ext == prov->extension())
        {
            prov->write(filePath, decompObj);
            return;
        }
    }
    throw std::runtime_error(std::string("No map provider for writing was applicable for the given filepath:") + filePath);
}
