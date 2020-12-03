#include <LoopsIO/Helpers/OutStreamWrapper.h>

LoopsIO::Helpers::OutStreamWrapper::OutStreamWrapper(std::ostream& str): str(str)
{
}

LoopsIO::Helpers::OutStreamWrapper& LoopsIO::Helpers::OutStreamWrapper::nl()
{
    str << '\n';
    return *this;
}

LoopsIO::Helpers::OutStreamWrapper& LoopsIO::Helpers::OutStreamWrapper::writeOpenXml(const std::string& tag)
{
    return write('<').write(tag);
}

LoopsIO::Helpers::OutStreamWrapper& LoopsIO::Helpers::OutStreamWrapper::writeCloseXml(const std::string& tag)
{
    return write("</").write(tag).write(">");
    return *this;
}

LoopsIO::Helpers::OutStreamWrapper& LoopsIO::Helpers::OutStreamWrapper::tab()
{
    str << '\t';
    return *this;
}

LoopsIO::Helpers::OutStreamWrapper& LoopsIO::Helpers::OutStreamWrapper::space()
{
    str << ' ';
    return *this;
}
