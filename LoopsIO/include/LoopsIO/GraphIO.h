#ifndef IO_GRAPHIO_H
#define IO_GRAPHIO_H
#include <string>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/Helpers/DecompositionObject.h>
#include <LoopsIO/IMapProvider.h>

namespace LoopsLib::DS {
    class BaseGraph;
}

namespace LoopsIO
{
    class GraphIO
    {
        // Map providers
        std::vector<IMapProvider*> m_providers;
        // File filters in Qt style
        std::string m_fileFilter = "";
        // Map filter name to provider
        std::map<std::string, IMapProvider*> m_filterMap;
        // Map extension to provider
        std::map<std::string, IMapProvider*> m_extensionMap;
        // Update the file filter.
        void updateFileFilter();

        /**
         * \brief Returns the singleton instance of GraphIO
         * \return The instance
         */
        static GraphIO& inst();
        GraphIO();
        ~GraphIO();
        template<typename T>
        static void registerTypeProvider()
        {
            registerProvider(new T());
        }
        /**
         * \brief Given the required arguments, checks if the uri has the args, encoded via url-like encoding.
         * If missing, reports it in the output variable and returns true, otherwise returns false.
         * \param uri The uri to check
         * \param requiredArgs Expected arguments
         * \param missingArgs Output missing arguments
         * \return Were there missing arguments.
         */
        static bool hasMissingArguments(const std::string& uri, const std::vector<std::string>& requiredArgs,
            std::vector<std::string>& missingArgs);
    public:
        /**
         * \brief Retrieves the file filter for parsable maps
         * \return The file filter
         */
        static std::string fileFilter();

        /**
         * \brief Registers a new map provider
         * \param provider The provider
         */
        static void registerProvider(IMapProvider* provider);

        template<typename...Types>
        static void registerProviders()
        {
            (registerTypeProvider<Types>(), ...);
        }
        /**
         * \brief Checks if the uri has missing arguments that should be encoded via url-like encoding. 
         * This depends on the provider for the extension in the path.
         * If missing, reports it in the output variable and returns true, otherwise returns false.
         * \param uri The uri to check
         * \param requiredArgs Expected arguments
         * \param missingArgs Output missing arguments
         * \return Were there missing arguments.
         */
        static bool hasMissingArguments(const std::string& uri, std::vector<std::string>& missingArgs);


        static void read(const std::string& filter, const std::string& filePath,
            LoopsLib::DS::EmbeddedGraph& decompObj);

        static void read(const std::string& filePath, LoopsLib::DS::EmbeddedGraph& decompObj);

        static void write(const std::string& filter, const std::string& filePath,
                          const LoopsLib::DS::EmbeddedGraph& decompObj);
        static void write(const std::string& filePath,
            const LoopsLib::DS::EmbeddedGraph& decompObj);
    };
}
#endif