#include <LoopsIO/ProblemInstanceProviders/IpePIProvider.h>
#include <QFile>
#include <QXmlStreamReader>
#include "LoopsIO/GraphIO.h"


void LoopsIO::ProblemInstanceProviders::IpePIProvider::read(const std::string& uri,
                                                            LoopsLib::Models::ProblemInstance& out)
{
    using Point = LoopsLib::DS::EmbeddedGraph::Point;
    using namespace IO::IOHelpers;
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(uri, path, args);

    out.m_savePaths.graphPath = UrlEncoder::encodeUrl(path, args, { "verticesLayerName", "edgesLayerName" });
    out.m_savePaths.fieldPath = UrlEncoder::encodeUrl(path, args, { "verticesLayerName", "edgesLayerName", "graphPathPrefix" });
    out.m_savePaths.instancePath = uri;
    //out.m_savePaths.instancePath = 

    LoopsLib::DS::EmbeddedGraph graph;

    // Read map from the same file
    LoopsIO::GraphIO::read(uri, graph);

    auto pathPrefix = QString::fromStdString(args["graphPathPrefix"]);

    // Parse paths
    QFile input(QString::fromStdString(path));
    if (!input.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        throw std::runtime_error("Couldn't open map file " + path);
    }
    QXmlStreamReader reader(&input);

    // Function for creating points from doubles
    auto makePoint = [](double x, double y)
    {
        using DType = decltype(std::declval<Point&>().m_x);
        return Point((DType)x, (DType)y);
    };

    QString reprPrefix = QString::fromStdString(args["representativePrefix"]);

    // Flags that the active layer in the ipe doc should be used for extracting field paths
    std::string activeLayer;
    bool hasApplicableLayer = false;
    int pathId = 0;
    while (!reader.atEnd())
    {
        auto tokenType = reader.readNext();
        if (tokenType != QXmlStreamReader::StartElement)
        {
            continue;
        }
        if (reader.attributes().hasAttribute("layer"))
        {
            activeLayer = reader.attributes().value("layer").toString().toStdString();
            hasApplicableLayer = reader.attributes().value("layer").startsWith(reprPrefix);
        }
        if(activeLayer == "epsilon" && reader.name() == "path")
        {
            // TODO pickup position and transformations at some point.
            auto pathDescription = reader.readElementText();
            // Split on whitespace
            auto parts = pathDescription.split(QRegExp("\\s+"));
            std::vector<Point> locations;
            // Parse path operators.
            for (int i = 0; i < parts.size(); ++i)
            {
                if (parts[i] == "m")
                {
                    locations.push_back(makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble()));
                }
                else if (parts[i] == "l")
                {
                    locations.push_back(makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble()));
                }
                else if (parts[i] == "h")
                {
                    break;
                }
            }
            // Determine length
            out.m_epsilon = (locations[1] - locations[0]).length();
        }
        if (hasApplicableLayer && reader.name() == "path")
        {
            LoopsLib::Math::Matrix3<LoopsLib::KernelDef::NT> transform;

            if(reader.attributes().hasAttribute("matrix"))
            {
                IO::IOHelpers::ipeMatrixFromString(reader.attributes().value("matrix").toString(), transform);
            }

            // TODO pickup position and transformations at some point.
            auto pathDescription = reader.readElementText();
            // Split on whitespace
            auto parts = pathDescription.split(QRegExp("\\s+"));
            std::vector<Point> trajectory;
            // Parse path operators.
            for (int i = 0; i < parts.size(); ++i)
            {
                if (parts[i] == "m")
                {
                    trajectory.push_back(transform*makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble()));
                }
                else if (parts[i] == "l")
                {
                    trajectory.push_back(transform*makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble()));
                }
                else if (parts[i] == "h")
                {
                    break;
                }
            }
            out.addRepresentativeFromTrajectory(trajectory, "IpePath_" + std::to_string(pathId));
            ++pathId;
        }
    }
}

void LoopsIO::ProblemInstanceProviders::IpePIProvider::write(const std::string& fileName,
                                                             const LoopsLib::Models::ProblemInstance& out)
{
    std::ofstream stream(fileName);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open " + fileName);
    }
    boost::archive::text_oarchive arch(stream);
    arch & out;
}

std::vector<std::string> LoopsIO::ProblemInstanceProviders::IpePIProvider::requiredArguments() const
{
    return { "verticesLayerName", "edgesLayerName", "graphPathPrefix","representativePrefix" };
}

std::map<std::string, std::string> LoopsIO::ProblemInstanceProviders::IpePIProvider::argumentDefaults() const
{
    return {
        {"verticesLayerName","Graph_vertices"}, 
        {"edgesLayerName","Graph_edges"}, 
    {"graphPathPrefix","Path"},
    {"representativePrefix" ,"Repr"}
    };
}
