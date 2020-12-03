#include <LoopsIO/FieldProviders/JsonFieldProvider.h>
#include <LoopsIO/Helpers/JsonInStream.h>
#include <LoopsIO/Helpers/JsonStream.h>
#include <LoopsLib/Models/FlowField.h>

const char* NETWORK_KEY = "network";
const char* PATHS_KEY = "paths";
const char* PATHVALUES_KEY = "pathvalues";
const char* KNOWNPATHS_KEY = "knownpaths";
const char* PATHCOUNT_KEY= "pathCount";
const char* FIELD_KEY = "field";

LoopsIO::FieldProviders::JsonFieldProvider::JsonFieldProvider(): IFieldProvider("JSON field (*.fieldjson)","fieldjson")
{
}

void LoopsIO::FieldProviders::JsonFieldProvider::read(const std::string& filePath, LoopsLib::Models::FlowField& decompObj)
{
    std::ifstream stream(filePath);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open file for writing");
    }
    Helpers::JsonInStream wrapper(stream);
    using Token = Helpers::JsonInStream::Token;
    Token t = wrapper.nextToken();
    assert(t == Token::ObjectBegin);
    t = wrapper.nextToken();
    decltype(wrapper.pos()) networkPos, pathsPos, pathValuesPos, knownPathsPos, pathCountPos;
    while(t != Token::Eof)
    {
        if(t == Token::FieldName)
        {
            std::string fieldName;
            wrapper.parseFieldName(fieldName);
            if (fieldName == NETWORK_KEY) networkPos = wrapper.pos();
            else if (fieldName == PATHS_KEY) pathsPos = wrapper.pos();
            else if (fieldName == PATHVALUES_KEY) pathValuesPos = wrapper.pos();
            else if (fieldName == KNOWNPATHS_KEY) knownPathsPos = wrapper.pos();
            else if (fieldName == PATHCOUNT_KEY) pathCountPos = wrapper.pos();
            t = wrapper.nextToken();
        }
        else
        {
            t = wrapper.skipLastToken();
        }
    }
    // Parse network
    {
        t = wrapper.moveToPos(networkPos);
        assert(t == Token::Value);
        std::string val;
        wrapper.parseValue(val);

        // Set the associated graph file
        m_associatedGraphFile = val;
        //if (!dependentMapCallback(val)) return;
    }
    // Parse field paths
    {
        t = wrapper.moveToPos(pathCountPos);
        assert(t == Token::Value);
        std::size_t pathCount;
        wrapper.parseValue(pathCount);

        decompObj.m_paths.reserve(pathCount);

        t = wrapper.moveToPos(pathsPos);
        assert(t == Token::ArrayBegin);
        for(std::size_t i = 0; i < pathCount; ++i)
        {
            t = wrapper.nextToken();
            if(i != 0)
            {
                assert(t == Token::Comma);
                t = wrapper.skipLastToken();
            }
            assert(t == Token::ArrayBegin);
            t = wrapper.skipLastToken();
            decompObj.m_paths.push_back({});
            auto& path = decompObj.m_paths.back();
            while(t != Token::ArrayEnd)
            {
                if (t == Token::Comma) { t = wrapper.skipLastToken(); }
                LoopsLib::DS::BaseGraph::Id_t id;
                wrapper.parseValue(id);
                path.push_back(id);
                t = wrapper.nextToken();
            }
        }
    }
    // Parse path values
    {
        t = wrapper.moveToPos(pathValuesPos);
        decompObj.m_pathValues.clear();
        assert(t == Token::ArrayBegin);
        t = wrapper.skipLastToken();
        while (t != Token::ArrayEnd)
        {
            if (t == Token::Comma) { t = wrapper.skipLastToken(); }
            LoopsLib::NT val;
            wrapper.parseValue(val);
            decompObj.m_pathValues.push_back(val);
            t = wrapper.nextToken();
        }
    }
    {
        t = wrapper.moveToPos(knownPathsPos);
        assert(t == Token::ArrayBegin);
    }
}

void LoopsIO::FieldProviders::JsonFieldProvider::write(const std::string& filePath,
    const LoopsLib::Models::FlowField& flowField)
{
    std::ofstream stream(filePath);
    if (!stream.is_open())
    {
        throw std::runtime_error("Could not open file for writing");
    }
    Helpers::JsonStream str(stream);
    str.beginObject().nl();
    {
        // The network filepath
        str.tab().field(NETWORK_KEY).fieldSep().value(flowField.m_graphFile).comma().nl();
        // Field 
        str.tab().field(FIELD_KEY).fieldSep().nl();
        {
            str.tab().beginObject().nl();
            str.tab(2).field(PATHCOUNT_KEY).fieldSep().value(flowField.m_paths.size()).commaNl();
            str.tab(2).field(PATHS_KEY).fieldSep().nl();
            // Paths: list of edge indices
            str.tab(2).beginArray().nl();
            for (auto i = 0; i < flowField.m_paths.size(); ++i)
            {
                const auto& p = flowField.m_paths[i];
                if (p.size() == 0)continue;
                str.tab(3).beginArray().commaSeparated(p).endArray();
                if (i != flowField.m_paths.size() - 1) str.comma();
                str.nl();
            }
            str.tab(2).endArray().commaNl();
            // Path values
            str.tab(2).field(PATHVALUES_KEY).fieldSep().beginArray().commaSeparated(flowField.m_pathValues).endArray().comma().nl();
            // Known paths
            //str.tab(2).field(KNOWNPATHS_KEY).fieldSep().array(decompObj.m_availablePaths).nl();
            str.tab().endObject(); //End of field
        }
    }

    // End of file
    str.endObject();
}
