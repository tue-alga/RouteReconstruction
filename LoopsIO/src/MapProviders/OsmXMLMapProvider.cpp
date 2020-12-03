#include <LoopsIO/MapProviders/OsmXMLMapProvider.h>
#include <GeographicLib/UTMUPS.hpp>
#include <movetk/geom/GeometryInterface.h>
using namespace LoopsLib;

movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel> makePnt = movetk_core::MakePoint<LoopsLib::MovetkGeometryKernel>();


void LoopsIO::MapProviders::OsmXMLMapProvider::RoadParser::Road::reset()
{
    isOneWay = false;
    osmIds.resize(0);
}

void LoopsIO::MapProviders::OsmXMLMapProvider::RoadParser::reset()
{
    parsingWay = false;
    highwayTagFound = false;
    exclude = false;
    activeRoad.reset();
}
//void atElementStart(const std::string& tagName, IOHelpers::InStreamWrapper& reader);
//
//void atElementEnd(const std::string& tagName, IOHelpers::InStreamWrapper& reader);

void LoopsIO::MapProviders::OsmXMLMapProvider::RoadParser::atElementStart(Helpers::XmlReader& reader)
{
    const auto& tagName = reader.tagName();
    if (tagName == "way")
    {
        reset();
        auto attrs = reader.attributes();
        activeRoad.id = attrs["id"].value<long long>();
        parsingWay = true;
        return;
    }
    if (!parsingWay || exclude) return;
    if (tagName == "tag")
    {
        std::string attrName;
        std::string attrValue;
        //std::string k, v;
        
        auto attrs = reader.attributes();
        auto k = attrs["k"];
        auto v = attrs["v"];

        /*reader.readXmlAttribute(attrName, attrValue);
        if (attrName == "k") k = attrValue;
        else if (attrName == "v") v = attrValue;
        reader.readXmlAttribute(attrName, attrValue);
        if (attrName == "k") k = attrValue;
        else if (attrName == "v") v = attrValue;*/

        if (k == "highway") 
        {
            highwayTagFound = true;
            if(layerMapping.find(std::string(v.m_value)) != layerMapping.end())
            {
                activeRoad.layer = layerMapping[std::string(v.m_value)];
            }
            else
            { 
                exclude = true;
                //activeRoad.layer = 5;
            }
        }
        if (k == "building" && v == "yes")
        {
            exclude = true;
        }
        else if (k == "area" && v == "yes")exclude = true;
        else if (k == "oneway" && v == "yes")
        {
            activeRoad.isOneWay = true;
        }
    }
    else if (tagName == "nd")
    {
        auto attrs = reader.attributes();
        //std::map < std::string, std::string> attrs;
        //reader.readXmlAttributes(attrs, std::set<std::string>{"ref"});
        activeRoad.osmIds.push_back(attrs["ref"].value<long long>());
    }
}

void LoopsIO::MapProviders::OsmXMLMapProvider::RoadParser::atElementEnd(Helpers::XmlReader& reader)
{
    if (reader.tagName() == "way")
    {
        parsingWay = false;
        if (!exclude && highwayTagFound)
        {
            roads.push_back(activeRoad);
            // Add nodes
            LoopsLib::DS::BaseGraph::Id_t startId = graph->vertexIdMap().size();
            for (auto node : activeRoad.osmIds)
            {
                if (!graph->vertexIdMap().containsForward(node))
                {
                    graph->vertexIdMap().insert(node,startId);
                    ++startId;
                }
            }
            activeRoad.reset();
        }
        reset();
    }
}

LoopsIO::MapProviders::OsmXMLMapProvider::NodeParser::NodeParser(LoopsLib::DS::EmbeddedGraph* graph):
    graph(graph)
{
    for (auto pair : LoopsLib::Helpers::Iterators::Iterable(graph->vertexIdMap().forwardBegin(), graph->vertexIdMap().forwardEnd()))
    {
        nodes.insert(pair.first);
    }
}

void LoopsIO::MapProviders::OsmXMLMapProvider::NodeParser::atElementStart(Helpers::XmlReader& reader)
{
    const auto& tagName = reader.tagName();
    if (tagName == "node")
    {
        auto attrs = reader.attributes();
        //reader.attributes(attrs, std::set<std::string>{"lat", "lon", "id"});

        const auto osmId = attrs["id"].value<long long>();
        if (graph->vertexIdMap().containsForward(osmId))
        {
            const auto lat = attrs["lat"].value<LoopsLib::NT>();
            const auto lon = attrs["lon"].value<LoopsLib::NT>();
            std::pair<LoopsLib::NT, LoopsLib::NT> xy = m_convert(lat,lon);


            graph->locations()[graph->vertexIdMap().forward(osmId)] = makePnt({xy.first, xy.second});
            nodes.erase(osmId);
        }
    }
}

void LoopsIO::MapProviders::OsmXMLMapProvider::NodeParser::atElementEnd(Helpers::XmlReader& reader)
{
}

LoopsIO::MapProviders::OsmXMLMapProvider::OsmXMLMapProvider():
    IMapProvider("OSM XML(*.osm)","osm")
{
}

void LoopsIO::MapProviders::OsmXMLMapProvider::setNegateY(bool value)
{
    m_negateY = value;
}

void LoopsIO::MapProviders::OsmXMLMapProvider::write(const std::string& outputPath,
                                                const LoopsLib::DS::EmbeddedGraph& decompObj)
{
}

void LoopsIO::MapProviders::OsmXMLMapProvider::read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& graph)
{
    struct SpatRefConverter
    {
        OGRSpatialReference srcRef;
        LoopsLib::DS::EmbeddedGraph* graph = nullptr;
        OGRCoordinateTransformation* transform = nullptr;
        void setup(LoopsLib::NT lat, LoopsLib::NT lon)
        {
            if (transform) return;
            int zone = (int)((lon + 180.) / 6.);
            graph->spatialRef().SetWellKnownGeogCS("WGS84");
            graph->spatialRef().SetUTM(zone);
            srcRef.SetWellKnownGeogCS("WGS84");
            transform = OGRCreateCoordinateTransformation(&srcRef, &graph->spatialRef());
        }
        ~SpatRefConverter() { OGRCoordinateTransformation::DestroyCT(transform); }
    };
    SpatRefConverter conv;
    conv.graph = &graph;
    std::function<std::pair<LoopsLib::NT, LoopsLib::NT>(LoopsLib::NT, LoopsLib::NT)> latLonConvert =
        [this,&conv](LoopsLib::NT lat, LoopsLib::NT lon)
    {
        double x = lon, y = lat;
        conv.setup(lat, lon);
        conv.transform->Transform(1, &x, &y);
        return std::make_pair(x, m_negateY ? -y : y);
    };
    //auto* graph = decompObj.m_graph;

    // Assume OSM map for now
    std::ifstream stream(inputPath, std::ios::binary);
    if (!stream.is_open())
    {
        std::cout << "Could not open file " << inputPath << std::endl;
        return;
    }
    // Setup the stream reader
    RoadParser roadParser(&graph);
    runOverFile(stream, roadParser);

    graph.clear();
    graph.allocateVertices(graph.vertexIdMap().size());
    graph.reserveLayers(6);

    auto& nodeIdRemap = graph.vertexIdMap();

    // Build the roads
    for (auto road : roadParser.roads)
    {
        for (int i = 0; i < road.osmIds.size() - 1; ++i)
        {
            auto* e= graph.addEdge(nodeIdRemap.forward(road.osmIds[i]), nodeIdRemap.forward(road.osmIds[i + 1]));
            graph.edgeIdMap().insert(road.id, e->id());
            graph.addToLayer(e->id(), road.layer);
            if (!road.isOneWay)
            {
                // Add reverse direction
                auto* e2= graph.addEdge(nodeIdRemap.forward(road.osmIds[i + 1]), nodeIdRemap.forward(road.osmIds[i]));
                graph.addToLayer(e2->id(), road.layer);
            }
        }
    }

    // Allocate location vector space
    graph.locations().clear();
    graph.locations().resize(nodeIdRemap.size());

    NodeParser nodeParser(&graph);
    nodeParser.m_convert = latLonConvert;
    runOverFile(stream, nodeParser);

    // Iterate in reverse, since the last element of the vertices may change ID.
    std::size_t vertNum = graph.number_of_vertices();
    if (!nodeParser.nodes.empty()) std::cout << "Missing nodes: " << nodeParser.nodes.size() << std::endl;
    for (auto it = nodeParser.nodes.rbegin(); it != nodeParser.nodes.rend(); ++it)
    {
        auto id = *it;
        graph.swapVertices(nodeIdRemap.forward(*it), vertNum - 1);
        --vertNum;
    }
    // Delete unused vertices
    graph.deleteVertexRange(vertNum);
    graph.locations().resize(graph.number_of_vertices());
}
