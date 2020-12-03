#include <LoopsIO/FieldProviders/IpeFieldProvider.h>
#include <LoopsIO/IOHelpers.h>
#include <QFile>
#include <QXmlStreamReader>
#include "LoopsIO/GraphIO.h"
#include "LoopsLib/Models/FlowField.h"

LoopsIO::FieldProviders::IpeFieldProvider::IpeFieldProvider(): IFieldProvider("Ipe file (*.ipe)", "ipe")
{
}

void LoopsIO::FieldProviders::IpeFieldProvider::read(const std::string& uri,
    LoopsLib::Models::FlowField& flowField)
{
    using Point = LoopsLib::DS::EmbeddedGraph::Point;

    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(uri, path, args);

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
    flowField.m_graphFile = IO::IOHelpers::UrlEncoder::encodeUrl(path, { {"verticesLayerName",args["verticesLayerName"]},{"edgesLayerName",args["edgesLayerName"]} });
    flowField.m_paths.clear();
    flowField.m_pathValues.clear();
    flowField.m_data.clear();

    // Flags that the active layer in the ipe doc should be used for extracting field paths
    bool hasApplicableLayer = false;
    double value = 1;
    while(!reader.atEnd())
    {
        auto tokenType = reader.readNext();
        if(tokenType != QXmlStreamReader::StartElement)
        {
            continue;
        }
        if(reader.attributes().hasAttribute("layer"))
        {
            if(reader.attributes().value("layer").startsWith(pathPrefix))
            {
                hasApplicableLayer = true;
                std::map<std::string, std::string> layerArgs;
                IO::IOHelpers::UrlEncoder::extractArgs(reader.attributes().value("layer").toString().toStdString(), layerArgs,'|','#');
                if(layerArgs.find("val") != layerArgs.end())
                {
                    value = std::stod(layerArgs.at("val"));
                }
            }
            else
            {
                hasApplicableLayer = false;
            }
        }
        if(hasApplicableLayer && reader.name() == "path")
        {

            LoopsLib::Math::Matrix3<LoopsLib::KernelDef::NT> transform;

            if (reader.attributes().hasAttribute("matrix"))
            {
                IO::IOHelpers::ipeMatrixFromString(reader.attributes().value("matrix").toString(), transform);
            }
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
                    locations.push_back(transform*makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble()));
                }
                else if (parts[i] == "l")
                {
                    locations.push_back(transform*makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble()));
                }
                else if(parts[i] == "h")
                {
                    break;
                }
            }
            // Match to elements in the graph
            std::vector<LoopsLib::DS::EmbeddedGraph::NodeIndex> vertexPath;
            std::vector<LoopsLib::DS::EmbeddedGraph::Idx_t> edgePath;
            std::transform(locations.begin(), locations.end(), std::back_inserter(vertexPath), [&graph](const auto& pnt)
            {
                return graph.getIndex().closest(pnt);
            });
            edgePath.reserve(vertexPath.size() - 1);
            for(int i = 1; i < vertexPath.size(); ++i)
            {
                // Duplicate point...
                if (vertexPath[i - 1] == vertexPath[i]) continue;

                edgePath.push_back(graph.vertex(vertexPath[i - 1])->findOutEdge(vertexPath[i])->id());
            }
            flowField.m_paths.push_back(edgePath);
            flowField.m_pathValues.push_back(value);
            std::cout << "Path with value " << value << std::endl;
        }
    }
}

void LoopsIO::FieldProviders::IpeFieldProvider::write(const std::string& filePath,
    const LoopsLib::Models::FlowField& decompObj)
{
}

std::vector<std::string> LoopsIO::FieldProviders::IpeFieldProvider::requiredArguments() const
{
    return { "verticesLayerName", "edgesLayerName", "graphPathPrefix"};
}
