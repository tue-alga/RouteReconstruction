#include <LoopsIO/FieldIO.h>
#include "LoopsIO/IOHelpers.h"


void LoopsIO::FieldIO::writeRow(std::ostream& str, const Eigen::VectorXd& v, char sep)
{
	for (int i = 0; i < v.size(); i++)
	{
		if (i != 0) str << sep;
		str << v(i);
	}
}

bool LoopsIO::FieldIO::startsWith(const std::string& target, const std::string& search)
{
	if (search.size() > target.size()) return false;
	for (int i = 0; i < search.size(); i++)
	{
		if (search[i] != target[i]) return false;
	}
	return true;
}

void LoopsIO::FieldIO::updateFilter()
{
    std::stringstream ss;
    // Construct all filter
    {
        ss << "All supported files (";
        bool isFirst = true;
        for (const auto& pair : m_extensionMap)
        {
            if (isFirst)
            {
                isFirst = false;
            }
            else
            {
                ss << " ";
            }
            ss << "*." << pair.first;
        }
        ss << ")";
    }

    if (m_providers.size() == 0) return;
    for (int i = 0; i < m_providers.size(); ++i)
    {
        ss << ";;" << m_providers[i]->fileFilter();
    }
    m_filter = ss.str();
}

LoopsIO::FieldIO& LoopsIO::FieldIO::inst()
{
    static FieldIO el;
    return el;
}

LoopsIO::FieldIO::FieldIO()
{
}

LoopsIO::FieldIO::~FieldIO()
{
    for (auto provider : m_providers)
    {
        delete provider;
    }
    m_providers.clear();
    m_fileFilterMap.clear();
}

bool LoopsIO::FieldIO::hasMissingArguments(const std::string& filePath, const std::vector<std::string>& requiredArgs,
    std::vector<std::string>& missingArgs)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(filePath, path, args);
    for (const auto& arg : requiredArgs)
    {
        if (args.find(arg) == args.end())
        {
            missingArgs.push_back(arg);
        }
    }
    return !missingArgs.empty();
}

bool LoopsIO::FieldIO::hasMissingArguments(const std::string& uri, std::vector<std::string>& missingArgs)
{
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(uri, path, args);

    auto dotIndex = path.rfind('.');
    auto ext = path.substr(dotIndex + 1);
    if (inst().m_extensionMap.find(ext) == inst().m_extensionMap.end())
    {
        std::cout << "Could not find provider for extension " + ext << std::endl;
        throw std::runtime_error("Could not find provider for extension " + ext);
    }
    // TODO Wrong index: throw something
    auto* prov = inst().m_extensionMap.at(ext);
    return hasMissingArguments(uri, prov->requiredArguments(), missingArgs);
}

void LoopsIO::FieldIO::registerProvider(IFieldProvider* provider)
{
    inst().m_providers.push_back(provider);
    inst().m_fileFilterMap[provider->fileFilter()] = provider;
    inst().m_extensionMap[provider->ext()] = provider;
    inst().updateFilter();
}

std::string LoopsIO::FieldIO::fileFilter()
{
    return inst().m_filter;
}

std::string LoopsIO::FieldIO::associatedGraphPath()
{
    return inst().m_graphForField;
}

std::string LoopsIO::FieldIO::lastFieldPath()
{
    return inst().m_lastFieldPath;
}

void LoopsIO::FieldIO::read(const std::string& filter, const std::string& path, LoopsLib::Models::FlowField& target)
{
    if(filter.empty())
    {
        read(path, target);
        return;
    }
    if (inst().m_fileFilterMap.find(filter) == inst().m_fileFilterMap.end())
    {
        // Try without filter selection
        read(path, target);
        return;
    }
    auto* prov = inst().m_fileFilterMap[filter];
    prov->read(path, target);
    inst().m_graphForField = prov->associatedGraphPath();
    // Success
    if (target.m_paths.size() > 0)
    {
        inst().m_lastFieldPath = path;
    }
}

void LoopsIO::FieldIO::read(const std::string& path, LoopsLib::Models::FlowField& target)
{
    auto parts = IO::IOHelpers::UrlEncoder::decodeUrl(path);
    auto pos = parts.first.find_last_of('.');
    auto ext = parts.first.substr(pos + 1);
    std::cout << " Reading field from " << parts.first << std::endl;
    auto loc = inst().m_extensionMap.find(ext);
    if (loc == inst().m_extensionMap.end()) throw std::runtime_error("Unsupported file with extension " + ext);

    (*loc).second->read(path, target);
    if (target.m_paths.size() > 0)
    {
        inst().m_lastFieldPath = path;
    }
    inst().m_graphForField = (*loc).second->associatedGraphPath();
}

void LoopsIO::FieldIO::write(const std::string& filter, const std::string& path,
                             const LoopsLib::Models::FlowField& target)
{
    if (inst().m_fileFilterMap.find(filter) == inst().m_fileFilterMap.end()) throw std::runtime_error(
        "Could not find filter");
    inst().m_fileFilterMap[filter]->write(path, target);
}

void LoopsIO::FieldIO::write(const std::string& path, const LoopsLib::Models::FlowField& target)
{
    auto ext = ::IO::IOHelpers::extension(path);
    std::cout << " Writing field to " << path << std::endl;
    for (auto* prov : inst().m_providers)
    {
        std::cout << "Checking " << prov->ext() << std::endl;
        if (prov->ext() == ext)
        {
            prov->write(path, target);
            if (target.m_paths.size() > 0)
            {
                inst().m_lastFieldPath = path;
            }
            inst().m_graphForField = prov->associatedGraphPath();
            return;
        }
    }
    throw std::runtime_error(std::string("No provider for extension:") + ext);
}
