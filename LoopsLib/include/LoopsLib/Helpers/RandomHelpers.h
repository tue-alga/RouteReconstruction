#ifndef HELPERS_RANDOMHELPERS_H
#define HELPERS_RANDOMHELPERS_H
#include <random>
#include <chrono>

namespace LoopsLib::Helpers{
	class RandomHelpers
	{
	public:
		static int randomOther(int self, int start, int endExcl)
		{
			int range = (endExcl - start) - 1;
			int el = rand() % range;
			if(el >= self)
			{
				return el + 1;
			}
			return el;
		}

        static inline auto getRandomEngine()
		{
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            return std::default_random_engine(seed);
		}

        template<typename It>
        static void shuffle(It start, It end)
        {
            std::shuffle(start, end, getRandomEngine());
        }

        template<typename It>
        static It randomElement(It start, It end)
		{
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
            It ret = start;
            std::advance(ret, dis(gen));
            return ret;
        }
		static double randomDouble(double min, double max)
		{
			std::default_random_engine eng;
			std::uniform_real_distribution<double> uni(min, max);
			return uni(eng);
		}
        template<typename Flt>
        static Flt randomFloatingPoint(Flt min, Flt max)
        {
            std::default_random_engine eng;
            std::uniform_real_distribution<Flt> uni(min, max);
            return uni(eng);
        }
        /**
         * \brief Generates a random long value
         * \param min Inclusive minimum 
         * \param max Inclusive maximum!
         * \return The random value
         */
        static long long randomLong(long long min, long long max)
        {
            auto eng = getRandomEngine();
            std::uniform_int_distribution<long long> uni(min, max);
            return uni(eng);
        }
	};
}
#endif
