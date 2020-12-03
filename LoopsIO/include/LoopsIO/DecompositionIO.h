#ifndef IO_DECOMPOSITIONIO_H
#define IO_DECOMPOSITIONIO_H
#include<string>
#include <Eigen/Eigen>
#include <fstream>
#include <LoopsLib/Algs/Types.h>
#include <LoopsLib/DS/OwnGraph.h>
#include <LoopsLib/Helpers/DecompositionObject.h>
#include <LoopsIO/IDecompositionProvider.h>

namespace LoopsIO {
	class DecompositionIO
	{
		// The output directory.
		std::string m_outputDir;

        std::vector<IDecompositionProvider*> m_providers = {};
        std::map < std::string, IDecompositionProvider*> m_fileFilterMap = {};

        // Total file filter, in Qt format.
        std::string m_filter;

        void updateFilter();

        static DecompositionIO& inst();
        DecompositionIO();

        ~DecompositionIO();
	public:
        static void registerProvider(IDecompositionProvider* provider);

        static std::string fileFilter();

        static void read(const std::string& filter, const std::string& path, LoopsLib::Helpers::DecompositionObject& target,
                         std::function<bool(const std::string&)> dependentMapCallback,
                         std::function<bool(const std::string&)> dependentFieldCallback);

        static void read(const std::string& path, LoopsLib::Helpers::DecompositionObject& target,
                         std::function<bool(const std::string&)> dependentMapCallback,
                         std::function<bool(const std::string&)> dependentFieldCallback);

        static void write(const std::string& filter, const std::string& path,
                          const LoopsLib::Helpers::DecompositionObject& target);

        static void write(const std::string& path, const LoopsLib::Helpers::DecompositionObject& target);
	};
}
#endif
