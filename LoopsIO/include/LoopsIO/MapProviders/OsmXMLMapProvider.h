#ifndef LOOPSIO_MAPPROVIDERS_OSMXMLMAPPROVIDER_H
#define LOOPSIO_MAPPROVIDERS_OSMXMLMAPPROVIDER_H
#include <LoopsIO/Helpers/XmlReader.h>
#include <LoopsIO/IMapProvider.h>

namespace LoopsIO::MapProviders
{
    class OsmXMLMapProvider : public IMapProvider
    {

        template<typename Hooks>
        void runOverFile(std::ifstream& file, Hooks& hooks)
        {

            // Reset
            file.clear();
            file.seekg(0);
            Helpers::XmlReader wrapper(file);
            std::string tagName;
            while(wrapper)
            {
                wrapper.readNextElement();
                if(!wrapper.isClose())
                {
                    hooks.atElementStart(wrapper);
                }
                else
                {
                    hooks.atElementEnd(wrapper);
                }
            }
        }

        template<typename U, typename V>
        struct BidirectionalUnorderedMap
        {
            std::unordered_map<U, V> m_forward;
            std::unordered_map<V, U> m_backward;

            void insert(U u, V v)
            {
                m_forward[u] = v;
                m_backward[v] = u;
            }
            U backward(V v) const
            {
                return m_backward[v];
            }
            V forward(U u) const
            {
                return m_forward[u];
            }
            void erase(U u, V v)
            {
                m_forward.erase(u);
                m_backward.erase(v);
            }
        };

        struct RoadParser
        {
            bool parsingWay = false;
            bool highwayTagFound = false;
            bool exclude = false;

            std::map < std::string, std::size_t> layerMapping = {
                {"motorway",0},
                {"motorway_link",0},
                {"trunk",0},
                {"trunk_link",0},
                {"primary",1},
                {"primary_link",1},
                {"secondary",2},
                {"secondary_link",2},
                {"tertiary",3},
                {"tertiary_link",3},
                {"unclassified",4},
                {"residential",4}
            };

            struct Road
            {
                long long id = 0;
                bool isOneWay = false;
                std::vector<long long> osmIds;
                std::size_t layer = 0;
                void reset();
            };
            // Parsed data
            std::vector<Road> roads;
            LoopsLib::DS::EmbeddedGraph* graph;

            RoadParser(LoopsLib::DS::EmbeddedGraph* graph):
                graph(graph){}
            
            Road activeRoad;

            void reset();

            void atElementStart(Helpers::XmlReader& reader);

            void atElementEnd(Helpers::XmlReader& reader);
        };

        struct NodeParser
        {
            
            std::set<long long> nodes;
            LoopsLib::DS::EmbeddedGraph* graph;

            std::function<std::pair<LoopsLib::NT, LoopsLib::NT>(LoopsLib::NT, LoopsLib::NT)> m_convert;

            NodeParser(LoopsLib::DS::EmbeddedGraph* graph);

            void atElementStart(Helpers::XmlReader& reader);

            void atElementEnd(Helpers::XmlReader& reader);
        };
        bool m_negateY = false;
    public:
        OsmXMLMapProvider();

        void setNegateY(bool value);

        void write(const std::string& outputPath, const LoopsLib::DS::EmbeddedGraph& decompObj) override;

        void read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj) override;
    };
}
#endif