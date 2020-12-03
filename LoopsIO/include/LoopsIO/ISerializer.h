#ifndef LOOPSIO_ISERIALIZER_H
#define LOOPSIO_ISERIALIZER_H
#include <string>
#include <vector>
#include "IOHelpers.h"
#include "LoopsLib/Helpers/TypeTraits.h"

namespace LoopsIO
{
    /**
     * \brief Serializer interface
     */
    class ISerializer
    {
    protected:
        // File filter for this provider type
        std::string m_fileFilter;
        // The name of this provider. Should be unique
        std::string m_providerName;
        // The extension (without dot) that is used by this provider.
        std::string m_extension;
        std::vector<std::string> m_extensions;
        std::unordered_map<std::string, std::string> m_meta;
    public:
        ISerializer(const std::string& name, const std::string& fileFilter, const std::string& extension):
        m_providerName(name),
        m_fileFilter(fileFilter),
        m_extension(extension){}

        const std::vector<std::string>& extensions() const
        {
            return m_extensions;
        }

        /**
         * \brief The extension associated with the field provider
         * \return The extension
         */
        std::string extension() const
        {
            return m_extension;
        }

        /**
         * \brief Stores any non-serialized object data that may be required to reconstruct a correct state
         * \return The map of meta data.
         */
        const std::unordered_map<std::string, std::string>& lastReadMetaData() const
        {
            return m_meta;
        }

        /**
         * \brief The file filter to use
         * \return The file filter
         */
        std::string fileFilter() const
        {
            return m_fileFilter;
        }
        std::string name() const
        {
            return m_providerName;
        }

        /**
         * \brief Specifies the arguments required to properly parse the object to serialize/deserialize.
         * This is apart from the path and should be encoded in an URI whenever loading.
         * \return 
         */
        virtual std::vector<std::string> requiredArguments() const
        {
            return { };
        }
        /**
         * \brief Returns a map of the default values for the required arguments, if applicable.
         */
        virtual std::map<std::string, std::string> argumentDefaults() const
        {
            return { };
        }
        virtual ~ISerializer(){}
    };

    /**
     * \brief Serializer for a specific type
     * \tparam T The type to serialize
     */
    template<typename T>
    class TypedSerializer : public ISerializer
    {
    public:
        TypedSerializer(const std::string& name, const std::string& fileFilter, const std::string& extension)
            : ISerializer(name, fileFilter, extension)
        {
        }

        virtual void read(const std::string& fileName, T& out) = 0;
        virtual void write(const std::string& fileName, const T& out) = 0;
    };

    template<typename T>
    class SerializerFactory
    {
        // Serializers
        std::vector<TypedSerializer<T>*> m_providers;
        // File filters in Qt style
        std::string m_fileFilter = "";
        // Search paths for relative paths
        std::vector<std::string> m_searchPaths;
        // Map filter name to provider
        std::map<std::string, TypedSerializer<T>*> m_filterMap;
        // Map filter name to provider
        std::map<std::string, TypedSerializer<T>*> m_extensionMap;

        // Filters are of shape "<name> (<space separated extensions>)", where an extension should be given as "*.<extension>".
        // Filters are joined with ";;" characters
        // Update the file filter.
        void updateFileFilter()
        {
            std::stringstream ss;
            // Construct all filter
            {
                ss << "All supported files (";
                bool isFirst = true;
                for(const auto& pair: m_extensionMap)
                {
                    if(isFirst)
                    {
                        isFirst = false;
                    }
                    else
                    {
                        ss << " ";
                    }
                    ss << "*." << pair.first;
                }
                ss << ")";
            }

            if (m_providers.size() == 0) return;
            for (int i = 0; i < m_providers.size(); ++i)
            {
                ss << ";;" << m_providers[i]->fileFilter();
            }
            m_fileFilter = ss.str();
        }
        // Should the filter be recomputed
        bool m_filterDirty = false;

        /**
         * \brief Returns the singleton instance of GraphIO
         * \return The instance
         */
        static SerializerFactory<T>& inst()
        {
            static SerializerFactory<T> m_inst;
            return m_inst;
        }

        SerializerFactory(){}

        ~SerializerFactory()
        {
            for (auto* el : m_providers) delete el;
            m_providers.clear();
            m_filterMap.clear();
        }
        template<typename...Ts>
        static void registerTypesOfTuple(const LoopsLib::Helpers::TypeHolder<std::tuple<Ts...>>&)
        {
            (registerProvider(LoopsLib::Helpers::TypeHolder<Ts>{}), ...);
        }
    public:
        /**
         * \brief Retrieves the file filter for parsable maps
         * \return The file filter
         */
        static std::string fileFilter()
        {   
            if (inst().m_filterDirty) {
                inst().updateFileFilter();
                inst().m_filterDirty = false;
            }
            return inst().m_fileFilter;
        }


        /**
         * \brief Registers a new map provider
         * \param provider The provider
         */
        static void registerProvider(TypedSerializer<T>* provider)
        {
            inst().m_providers.push_back(provider);
            inst().m_filterMap[provider->fileFilter()] = provider;
            inst().m_extensionMap[provider->extension()] = provider;
            inst().m_filterDirty = true;
        }

        /**
         * \brief Register a new type by tag dispatching via a TypeHolder object
         * \tparam U The type to register
         */
        template<typename U>
        static void registerProvider(const LoopsLib::Helpers::TypeHolder<U>&)
        {
            static_assert(std::is_base_of_v<TypedSerializer<T>, U>);
            registerProvider(new U());
        }
        /**
         * \brief Register the given typea as provider
         * \tparam U The type to register
         * Type must inherit TypedSerializer<T>
         */
        template<typename U>
        static void registerProvider()
        {
            static_assert(std::is_base_of_v<TypedSerializer<T>, U>);
            registerProvider(new U());
        }

        template<typename Tuple>
        static void registerTypesOfTuple()
        {
            registerTypesOfTuple(LoopsLib::Helpers::TypeHolder<Tuple>{});
        }

        static std::vector<std::string> requiredArgsForUri(const std::string& uri)
        {
            std::string path;
            IO::IOHelpers::UrlEncoder::extractPath(uri, path);
            auto ext = IO::IOHelpers::extension(path);
            return inst().m_extensionMap[ext]->requiredArguments();
        }

        static std::map<std::string, std::string> argDefaultsForUrI(const std::string& uri)
        {
            std::string path;
            IO::IOHelpers::UrlEncoder::extractPath(uri, path);
            auto ext = IO::IOHelpers::extension(path);
            return inst().m_extensionMap[ext]->argumentDefaults();
        }

        static std::vector<std::string> availableExtensions()
        {
            auto& i = inst();
            std::vector<std::string> exts;
            for(auto* prov: i.m_providers)
            {
                exts.push_back(prov->extension());
            }
            return exts;
        }

        static void read(const std::string& filter, const std::string& filePath,
            T& target)
        {
            if(filter.empty())
            {
                read(filePath, target);
                return;
            }
            // Fallback to checking extension if filter fails
            if(inst().m_filterMap.find(filter) == inst().m_filterMap.end())
            {
                read(filePath, target);
                return;
            }
            inst().m_filterMap[filter]->read(filePath, target);
        }

        static void read(const std::string& filePath, T& target)
        {
            auto& io = inst();
            std::string path;
            std::map<std::string, std::string> args;
            IO::IOHelpers::UrlEncoder::decodeUrl(filePath, path, args);
            auto ext = ::IO::IOHelpers::extension(path);

            if(io.m_extensionMap.find(ext) == io.m_extensionMap.end())
            {
                throw std::runtime_error("No provider was applicable for filepath:" + filePath + ", ext: " + ext);
            }
            io.m_extensionMap[ext]->read(filePath, target);
        }

        static void write(const std::string& filter, const std::string& filePath,
            const T& target)
        {
            if(filter.empty())
            {
                write(filePath, target);
                return;
            }
            inst().m_filterMap[filter]->write(filePath, target);
        }

        static void write(const std::string& filePath,
            const T& target)
        {
            auto& io = inst();
            auto ext = ::IO::IOHelpers::extension(filePath);
            for (auto* prov : io.m_providers)
            {
                if (prov->extension() == ext)
                {
                    prov->write(filePath, target);
                    return;
                }
            }
            throw std::runtime_error("No provider was applicable for filepath:" + filePath);
        }
    };
}
#endif