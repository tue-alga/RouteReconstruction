#ifndef LOOPSIO_IFIELDPROVIDER_H
#define LOOPSIO_IFIELDPROVIDER_H
#include <string>
#include <LoopsLib/Helpers/DecompositionObject.h>

namespace LoopsLib {
    namespace Models {
        class FlowField;
    }
}

namespace LoopsIO
{
    class IFieldProvider
    {
    protected:
        // File filter for this provider type
        std::string m_fileFilter;
        // The name of this provider. Should be unique
        std::string m_providerName;
        // The extension (without dot) that is used by this provider.
        std::string m_extension;

        std::string m_associatedGraphFile;
    public:
        IFieldProvider(const std::string& fileFilter, const std::string& extension);

        /**
         * \brief Returns the path of the graph file associated with the field, as read
         * from the last field file
         * \return The graph path.
         */
        std::string associatedGraphPath() const
        {
            return m_associatedGraphFile;
        }
        virtual std::vector<std::string> requiredArguments() const
        {
            return {  };
        }

        /**
         * \brief The extension associated with the field provider
         * \return The extension
         */
        std::string ext() const;

        /**
         * \brief The file filter to use
         * \return The file filter
         */
        std::string fileFilter() const;
        virtual ~IFieldProvider();
        /**
         * \brief Read the field
         * \param filePath 
         * \param decompObj 
         * \param dependentMapCallback 
         */
        virtual void read(const std::string& filePath, LoopsLib::Models::FlowField& decompObj) = 0;
        /**
         * \brief Write the field to file
         * \param filePath The output file path
         * \param decompObj The decomposition object containing the field.
         */
        virtual void write(const std::string& filePath, const LoopsLib::Models::FlowField& decompObj) = 0;
    };
}
#endif