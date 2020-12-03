#ifndef HELPERS_FIELDGENERATOR_H
#define HELPERS_FIELDGENERATOR_H
#include <Eigen/Eigen>
#include <LoopsLib/DS/OwnGraph.h>
#include "DecompositionObject.h"

namespace LoopsLib {
    namespace Helpers {
        class IFieldGenerator
        {
        public:
            virtual ~IFieldGenerator() {}
            virtual void generateField() = 0;
        };

        class FieldGenerator
        {
        public:
            enum class Generators
            {
                UniformRandom = 0,
                NormalRandom,
                PathBased,
                VertexOrder
            };
        private:

            enum class RandomType
            {
                Uniform,
                Normal
            };

            Generators m_activeGen = Generators::UniformRandom;
            double m_mean = 10;
            double m_sd = 2;

            // Uniform
            double m_offset = 5;
            double m_maxSpan = 10;

            // Path based
            double m_pathValue = 120.;
            int m_numberOfPaths = 30;
            double m_probSearchEnd = 0.6;

            LoopsLib::DS::BaseGraph* m_graph;

            DecompositionObject* m_decompObj;

            /**
             * \brief Sets the parameters of the normal distribution of RandomType::Normal
             * \param mean The mean
             * \param sd The standard deviation
             */
            void setNormalCoeffs(double mean, double sd);

            /**
             * \brief Generates a random field using the given random type. Internal parameters
             * will be used for the distributions. Set them with set<RandomType>Coeffs().
             * \param m_graph The graph for which to generate a field
             * \param type The type of distribution/approach to apply
             * \param field The output field
             */
            void generateRandomField(RandomType type, Eigen::VectorXd& field);

            void generateRandomPathField(int source, int sink, int maxPaths, double maxVal, Eigen::VectorXd& field);
        public:

            void setNumberOfPaths(int numberOfPaths);

            FieldGenerator(DecompositionObject* decompObj) : m_graph(decompObj->m_graph), m_decompObj(decompObj) {}

            void setGraph(LoopsLib::DS::BaseGraph* graph);

            void generatePathsField();

            void generateRandomPathsField();

            //void generateField(int source, int sink, std::vector<MovetkGeometryKernel::NT>& field);
        };
    }
}
#endif
