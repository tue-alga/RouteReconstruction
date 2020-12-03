#include <LoopsIO/Helpers/JsonStream.h>
LoopsIO::Helpers::JsonStream::JsonStream(std::ostream& str) : str(str)
{
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::tab(int count)
{
    for (int i = 0; i < count; ++i)
        str << '\t';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::field(const std::string& name)
{
    str << '"' << name << '"';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::fieldSep()
{
    str << ':';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::nl()
{
    str << '\n';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::beginObject()
{
    str << '{';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::endObject()
{
    str << '}';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::beginArray()
{
    str << '[';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::comma()
{
    str << ',';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::endArray()
{
    str << ']';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::commaNl()
{
    str << ',' << '\n';
    return *this;
}

LoopsIO::Helpers::JsonStream& LoopsIO::Helpers::JsonStream::eigenArray(const Eigen::VectorXd& vec)
{
    str << '[';
    for (int i = 0; i < vec.size(); ++i)
    {
        if (i != 0) str << ',';
        str << vec(i);
    }
    str << ']';
    return *this;
}
