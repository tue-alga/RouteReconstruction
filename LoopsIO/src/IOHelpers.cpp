#include <LoopsIO/IOHelpers.h>

LoopsLib::DS::BiMap<char, std::string> IO::IOHelpers::UrlEncoder::m_encoderMap = {
        {'!', "21"} ,
        {'#',"23"},
        {'$',"24"},
        {'%',"25"},
        {'&',"26"},
        {'\'',"27"},
        {'(',"28"},
        {')',"29"},
        {'*',"2A"},
        {'+',"2B"},
        {',',"2C"},
        {'/',"2F"},
        {':',"3A"},
        {';',"3B"},
        {'=',"3D"},
        {'?',"3F"},
        {'@',"40"},
        {'[',"5B"},
        {']',"5D"},
        {'\n',"0A"},
        {' ',"20"},
        {'"',"22"},
        {'{',"7B"},
        {'}',"7D"}
};

IO::IOHelpers::Uri::Uri()
{
}

IO::IOHelpers::Uri::
Uri(const std::string& path, const std::map<std::string, std::string>& args): m_args(args), m_path(path)
{
}

std::string IO::IOHelpers::Uri::extension() const
{
    return IO::IOHelpers::extension(m_path);
}

std::string IO::IOHelpers::Uri::encoded(char argSeparator, char startOfArgs)
{
    std::stringstream out;
    encodeToStream(out, argSeparator, startOfArgs);
    return out.str();
}

void IO::IOHelpers::Uri::encodeToStream(std::ostream& str, char argSeparator, char startOfArgs)
{
    UrlEncoder::encodeUrlToStream(m_path, m_args, str, argSeparator, startOfArgs);
}

const std::map<std::string, std::string>& IO::IOHelpers::Uri::args() const
{
    return m_args;
}

void IO::IOHelpers::Uri::removeArg(const std::string& name)
{
    m_args.erase(name);
}

bool IO::IOHelpers::Uri::hasArg(const std::string& name) const
{
    return m_args.find(name) != m_args.end();
}

IO::IOHelpers::Uri IO::IOHelpers::Uri::decode(const std::string& uriStr, char argSeparator, char startOfArgs)
{
    Uri uri;
    UrlEncoder::decodeUrl(uriStr, uri.m_path, uri.m_args, argSeparator, startOfArgs);
    return uri;
}

bool IO::IOHelpers::Uri::operator==(const Uri& other) const
{
    if (m_path != other.m_path) return false;
    if (m_args.size() != other.m_args.size()) return false;
    for (const auto& pair : m_args)
    {
        if (!other.hasArg(pair.first)) return false;
        if (pair.second != other.m_args.at(pair.first)) return false;
    }
    return true;
}

bool IO::IOHelpers::Uri::hasArgs(const std::vector<std::string>& args) const
{
    for (const auto& el : args)
    {
        if (!hasArg(el)) return false;
    }
    return true;
}

bool IO::IOHelpers::Uri::hasArgs(const std::vector<std::string>& args, std::vector<std::string>& missing) const
{
    for (const auto& el : args)
    {
        if (!hasArg(el)) missing.push_back(el);
    }
    return missing.empty();
}

void IO::IOHelpers::UrlEncoder::addArgs(std::string& uri, const std::map<std::string, std::string>& args)
{
    auto index = uri.find('?');
    std::stringstream out;
    out << uri;
    if (index == std::string::npos)
    {
        out << "?";
    }
    bool isFirst = true;
    for (const auto& kv : args)
    {
        if (isFirst)
        {
            isFirst = false;
        }
        else
        {
            out << '&';
        }
        encodeStringToStream(kv.first, out);
        out << '=';
        encodeStringToStream(kv.second, out);
    }
    uri = out.str();
}

void IO::IOHelpers::UrlEncoder::extractPath(const std::string& uri, std::string& path)
{
    auto index = uri.find('?');
    if (index == std::string::npos)
    {
        path = uri;
    }
    else
    {
        path = uri.substr(0, index);
    }
}

void IO::IOHelpers::UrlEncoder::extractArgs(const std::string& uri, std::map<std::string, std::string>& args,
    char argSeparator, char startArgs)
{
    auto index = uri.find(startArgs);
    if (index == std::string::npos)
    {
        return;
    }
    auto offset = index + 1;
    auto equalsIndex = uri.find('=', offset);
    // Key-value pair separator
    auto sepIndex = uri.find(argSeparator, offset);
    auto keyLength = equalsIndex - offset;
    auto valLength = sepIndex == std::string::npos ? uri.size() - (equalsIndex + 1) : sepIndex - (equalsIndex + 1);
    while (equalsIndex != std::string::npos)
    {
        // Decode key-value pairs, separated by '=', and pairs separated by '&'
        std::stringstream k;
        decodeStringToStream(std::string_view(uri.data() + offset, keyLength), k);
        std::stringstream v;
        decodeStringToStream(std::string_view(uri.data() + equalsIndex + 1, valLength), v);
        args[k.str()] = v.str();

        // Stop if no next pair is present
        if (sepIndex == std::string::npos) break;

        // Update offsets
        offset = sepIndex + 1;
        equalsIndex = uri.find('=', offset);
        sepIndex = uri.find(argSeparator, offset);
        keyLength = equalsIndex - offset;
        valLength = sepIndex == std::string::npos ? uri.size() - (equalsIndex + 1) : sepIndex - (equalsIndex + 1);
    }

}

bool IO::IOHelpers::UrlEncoder::isSameUri(const std::string& uri1, const std::string& uri2)
{
    std::string path1, path2;
    ArgsMap args1, args2;
    decodeUrl(uri1, path1, args1);
    decodeUrl(uri2, path2, args2);
    if (path1 != path2) return false;
    if (args1.size() != args2.size()) return false;
    for(const auto& kv: args1)
    {
        if (args2.find(kv.first) == args2.end()) return false;
        if (kv.second != args2.at(kv.first)) return false;
    }
    return true;
}

void IO::IOHelpers::UrlEncoder::encodeStringToStream(const std::string& string, std::ostream& out)
{
    for (const char c : string)
    {
        if (m_encoderMap.containsForward(c))
        {
            out << '%' << m_encoderMap.forward(c);
        }
        else
        {
            out << c;
        }
    }
}

void IO::IOHelpers::UrlEncoder::encodeArgsToStream(const std::map<std::string, std::string>& args, std::ostream& out)
{
    bool first = true;
    for(const auto& kv : args)
    {
        if(first)
        {
            first = false;
        }
        else
        {
            out << "&";
        }
        encodeStringToStream(kv.first, out);
        out << "=";
        encodeStringToStream(kv.second, out);
    }
}

void IO::IOHelpers::UrlEncoder::decodeStringToStream(const std::string_view& string, std::ostream& out)
{
    int i = 0;
    for (; i < string.size();)
    {
        if (string[i] == '%')
        {
            auto code = string.substr(i + 1, 2);
            out << m_encoderMap.backward(std::string(code));
            i += 3;
        }
        else
        {
            out << string[i];
            ++i;
        }
    }
}

void IO::IOHelpers::UrlEncoder::decodeStringToStream(const std::string& string, std::ostream& out)
{
    int i = 0;
    for (; i < string.size();)
    {
        if (string[i] == '%')
        {
            auto code = string.substr(i + 1, 2);
            out << m_encoderMap.backward(std::string(code));
            i += 3;
        }
        else
        {
            out << string[i];
            ++i;
        }
    }
}

std::string IO::IOHelpers::UrlEncoder::encodeUrl(const std::string& path,
                                                 const std::map<std::string, std::string>& arguments,
                                                char argSeparator, char startArgs)
{
    std::stringstream total;
    total << path;
    if (arguments.size() == 0) return total.str();
    total << startArgs;
    bool isFirst = true;
    for (const auto& kv : arguments)
    {
        if (isFirst)
        {
            isFirst = false;
        }
        else
        {
            total << argSeparator;
        }
        encodeStringToStream(kv.first, total);
        total << '=';
        encodeStringToStream(kv.second, total);
    }
    return total.str();
}

void IO::IOHelpers::UrlEncoder::encodeUrlToStream(const std::string& path,
    const std::map<std::string, std::string>& arguments, std::ostream& str, char argSeparator, char startArgs)
{
    str << path;
    if (arguments.size() == 0) return;
    str << startArgs;
    bool isFirst = true;
    for (const auto& pair : arguments)
    {
        if (isFirst)
        {
            isFirst = false;
        }
        else
        {
            str << argSeparator;
        }
        encodeStringToStream(pair.first, str);
        str << '=';
        encodeStringToStream(pair.second, str);
    }
}

std::string IO::IOHelpers::UrlEncoder::encodeUrl(const std::string& path,
    const std::map<std::string, std::string>& arguments, const std::vector<std::string>& argSelection, char argSeparator , char startArgs )
{
    std::stringstream total;
    total << path;
    if (arguments.size() == 0) return total.str();
    total << startArgs;
    bool isFirst = true;
    for (const auto& k : argSelection)
    {
        const auto& v = arguments.at(k);
        if (isFirst)
        {
            isFirst = false;
        }
        else
        {
            total << argSeparator;
        }
        encodeStringToStream(k, total);
        total << '=';
        encodeStringToStream(v, total);
    }
    return total.str();
}

void IO::IOHelpers::UrlEncoder::decodeUrl(const std::string& url, std::string& path,
                                          std::map<std::string, std::string>& arguments, char argSeparator, char startArgs)
{
    auto sepIndex = url.find(startArgs);
    if (sepIndex == std::string::npos)
    {
        path = url;
        return;
    }
    path = url.substr(0, sepIndex);

    auto offset = sepIndex + 1;
    auto equalsIndex = url.find('=', offset);
    // Key-value pair separator
    sepIndex = url.find(argSeparator, offset);
    auto keyLength = equalsIndex - offset;
    auto valLength = sepIndex == std::string::npos ? url.size() - (equalsIndex + 1) : sepIndex - (equalsIndex + 1);
    while (equalsIndex != std::string::npos)
    {
        // Decode key-value pairs, separated by '=', and pairs separated by '&'
        std::stringstream k;
        decodeStringToStream(std::string_view(url.data() + offset, keyLength), k);
        std::stringstream v;
        decodeStringToStream(std::string_view(url.data() + equalsIndex + 1, valLength), v);
        arguments[k.str()] = v.str();

        // Stop if no next pair is present
        if (sepIndex == std::string::npos) break;

        // Update offsets
        offset = sepIndex + 1;
        equalsIndex = url.find('=', offset);
        sepIndex = url.find(argSeparator, offset);
        keyLength = equalsIndex - offset;
        valLength = sepIndex == std::string::npos ? url.size() - (equalsIndex + 1) : sepIndex - (equalsIndex + 1);
    }
}

std::pair<std::string, IO::IOHelpers::UrlEncoder::ArgsMap> IO::IOHelpers::UrlEncoder::decodeUrl(const std::string& url,
    char argSeparator, char startArgs)
{
    std::pair<std::string, ArgsMap> output;
    decodeUrl(url, output.first, output.second, argSeparator, startArgs);
    return output;
}

bool IO::IOHelpers::UrlEncoder::hasMissingArguments(const std::string& filePath,
                                                    const std::vector<std::string>& requiredArgs,
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
