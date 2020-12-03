#ifndef IO_FIELDIO_H
#define IO_FIELDIO_H
#include<string>
#include <Eigen/Eigen>
#include <fstream>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/DS/BaseGraph.h>
#include <LoopsLib/Helpers/DecompositionObject.h>
#include <LoopsIO/IFieldProvider.h>
#include <LoopsLib/Models/FlowField.h>
#include "LoopsLib/Helpers/TypeTraits.h"

namespace LoopsIO{

	class FieldIO
	{
		// The output directory.
		std::string m_outputDir;

		void writeRow(std::ostream& str, const Eigen::VectorXd& v, char sep);

		bool startsWith(const std::string& target, const std::string& search);

        std::vector<IFieldProvider*> m_providers = {};
        std::map < std::string, IFieldProvider*> m_fileFilterMap = {};
        std::map < std::string, IFieldProvider*> m_extensionMap = {};

        // The total filter
        std::string m_filter;

        // Reading/writing data
        std::string m_graphForField;

        std::string m_lastFieldPath;

        void updateFilter();

        static FieldIO& inst();
        FieldIO();

        ~FieldIO();
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
       
        template<typename TupleType, std::size_t...Is>
        static void registerProviders(LoopsLib::Helpers::TypeHolder<TupleType>, std::index_sequence<Is...>)
        {
            (registerProvider<std::tuple_element_t<Is, TupleType>>(), ...);
        }
    public:

        static void registerProvider(IFieldProvider* provider);

        static std::string fileFilter();

        static bool hasMissingArguments(const std::string& uri, std::vector<std::string>& missingArgs);

        template<typename U>
        static void registerProvider()
        {
            static_assert(std::is_base_of_v<IFieldProvider, U>);
            registerProvider(new U());
        }

        template<typename TupleType>
        static void registerProvidersFromTuple()
        {
            registerProviders(LoopsLib::Helpers::TypeHolder<TupleType>{}, std::make_index_sequence<std::tuple_size_v<TupleType>>{});
        }

        /**
         * \brief Path to graph file of last read field
         * \return The path
         */
        static std::string associatedGraphPath();
        /**
         * \brief Returns the last field path that was written/read
         * \return The field path
         */
        static std::string lastFieldPath();

        static void read(const std::string& filter, const std::string& path, LoopsLib::Models::FlowField& target);

        static void read(const std::string& path, LoopsLib::Models::FlowField&  target);

        static void write(const std::string& filter, const std::string& path,
                          const LoopsLib::Models::FlowField&  target);

        static void write(const std::string& path,
            const LoopsLib::Models::FlowField&  target);
	};
}
#endif
