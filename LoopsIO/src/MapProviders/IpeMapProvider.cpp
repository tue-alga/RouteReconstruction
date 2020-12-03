#include <LoopsIO/MapProviders/IpeMapProvider.h>
#include <LoopsIO/IOHelpers.h>
#include <QXmlStreamReader>
#include <QFile>

using namespace LoopsIO;

LoopsIO::MapProviders::IpeMapProvider::IpeMapProvider(): IMapProvider("Ipe file (*.ipe)", "ipe")
{
}

void LoopsIO::MapProviders::IpeMapProvider::write(const std::string& outputPath,
    const LoopsLib::DS::EmbeddedGraph& decompObj)
{
}

void LoopsIO::MapProviders::IpeMapProvider::read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj)
{
    // Typedefs
    using Point = LoopsLib::DS::EmbeddedGraph::Point;

    // Clear the map
    decompObj.clear();

    // Parse URI args
    std::string path;
    std::map<std::string, std::string> args;
    IO::IOHelpers::UrlEncoder::decodeUrl(inputPath, path, args);

    // Open the file
    QFile input(QString::fromStdString(path));
    if(!input.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        throw std::runtime_error("Couldn't open map file " + path);
    }
    // Setup the XML reader
    QXmlStreamReader reader(&input);

    // Locations of vertices
    std::vector<Point> locations;

    // Layer names to look for
    const std::string verticesLayerName = args["verticesLayerName"];
    const std::string edgesLayerName = args["edgesLayerName"];

    // The known active layer
    std::string activeLayer;

    auto makePoint = [](double x, double y)
    {
        using DType = decltype(std::declval<Point&>().m_x);
        return Point((DType)x, (DType)y);
    };

    // Parsing struct
    struct Edge
    {
        Point p0, p1;
        bool isDirected = false;
    };

    // Edge data
    std::vector<Edge> edges;
    std::size_t totalEdges = 0;

    // Walk through the XML doc
    while(!reader.atEnd())
    {
        auto token = reader.readNext();

        // Only interested in start elements
        if (token != QXmlStreamReader::StartElement) continue;

        //std::cout << "Read node: " << reader.name().toString().toStdString() << std::endl;
        if(reader.attributes().hasAttribute("layer"))
        {
            activeLayer = reader.attributes().value("layer").toString().toStdString();
            //std::cout << "Changing to layer " << activeLayer << std::endl;
        }
        // Vertices as markers
        if(activeLayer == verticesLayerName)
        {
            if(reader.name().toString() == "use")
            {
                LoopsLib::Math::Matrix3<LoopsLib::KernelDef::NT> transform;

                if (reader.attributes().hasAttribute("matrix"))
                {
                    IO::IOHelpers::ipeMatrixFromString(reader.attributes().value("matrix").toString(), transform);
                }

                auto parts = reader.attributes().value("pos").split(" ");
                locations.push_back(transform * makePoint(parts[0].toDouble(), parts[1].toDouble()));
            }
        }
        else if (activeLayer == edgesLayerName)
            {
                if (reader.name().toString() == "path")
                {
                    edges.push_back({});
                    auto& curr = edges.back();

                    LoopsLib::Math::Matrix3<LoopsLib::KernelDef::NT> transform;

                    if (reader.attributes().hasAttribute("matrix"))
                    {
                        IO::IOHelpers::ipeMatrixFromString(reader.attributes().value("matrix").toString(), transform);
                    }

                    bool swapPoints = false;
                    // Arrow signal directionality
                    if (
                        (reader.attributes().hasAttribute("arrow") && reader.attributes().hasAttribute("farrow")) ||
                        (!reader.attributes().hasAttribute("arrow") && !reader.attributes().hasAttribute("farrow")))
                    {
                        curr.isDirected = false;
                        totalEdges += 2;
                    }
                    else
                    {
                        curr.isDirected = true;
                        if (reader.attributes().hasAttribute("rarrow"))
                        {
                            swapPoints = true;
                        }
                        totalEdges += 1;
                    }

                    auto value = reader.readElementText();
                    auto parts = value.split(QRegExp("\\s+"));
                    for (int i = 0; i < parts.size(); ++i)
                    {
                        if (parts[i] == "m")
                        {
                            curr.p0 = transform * makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble());
                        }
                        else if (parts[i] == "l")
                        {
                            curr.p1 = transform * makePoint(parts[i - 2].toDouble(), parts[i - 1].toDouble());
                            break;
                        }
                    }
                    if (swapPoints)
                    {
                        std::swap(curr.p0, curr.p1);
                    }
                }
            }
    }
    if(reader.hasError())
    {
        std::cout << "Read error:" << reader.errorString().toStdString() << std::endl;
    }

    std::cout << "Number of vertices read: " << locations.size() << std::endl;
    decompObj.allocateVertices(locations.size());
    // Set locations and construct index
    decompObj.setLocations(locations);

    decompObj.allocateEdges(totalEdges);
    // Convert edges to actual edges in the graph
    std::size_t eId = 0;
    for(const auto& e : edges)
    {
        // Just resolve via closest point. 
        auto v0Id = decompObj.getIndex().closest(e.p0);
        auto v1Id = decompObj.getIndex().closest(e.p1);

        decompObj.setEdgeConnection(eId, v0Id, v1Id);
        ++eId;
        if(!e.isDirected)
        {
            // Reverse direction
            decompObj.setEdgeConnection(eId, v1Id,v0Id);
            ++eId;
        }
    }
}

std::vector<std::string> LoopsIO::MapProviders::IpeMapProvider::requiredParameters() const
{
    return {"verticesLayerName", "edgesLayerName"};
}
