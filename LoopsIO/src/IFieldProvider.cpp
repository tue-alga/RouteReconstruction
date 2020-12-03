#include <LoopsIO/IFieldProvider.h>

LoopsIO::IFieldProvider::IFieldProvider(const std::string& fileFilter, const std::string& extension):
    m_fileFilter(fileFilter), m_extension(extension)
{
}

std::string LoopsIO::IFieldProvider::ext() const
{
    return m_extension;
}

std::string LoopsIO::IFieldProvider::fileFilter() const
{
    return m_fileFilter;
}

LoopsIO::IFieldProvider::~IFieldProvider()
{
}
