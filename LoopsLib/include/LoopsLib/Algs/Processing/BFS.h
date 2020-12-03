#ifndef BFS_H
#define BFS_H
#include <queue>

namespace LoopsLib::Algs::Processing
{
	template<typename GraphType>
	struct BFSGraphArchetype
	{
		using Identifier = typename GraphType::VertexIdentifier;
		BFSGraphArchetype(const GraphType& g)
		{
			// Identifier should be trivially constructible
			static_assert(std::is_trivially_constructible<Identifier>::value,"Vertex Identifier should be trivially constructible");
			// Identifier should be convertible to int
			static_assert(std::is_convertible<Identifier, int>::value, "Vertex identifier should be castable to int");
			// Has a adjacent(int) function returning an iterable giving Identifier elements
			static_assert(std::is_convertible< decltype( *(std::declval<GraphType>().adjacent(0).begin()) ), Identifier>::value);
			// Has a function vertexCount() returning an integer.
			static_assert(std::is_integral< decltype(std::declval<GraphType>().vertexCount())>::value);
		}
	};

	template<typename GraphType>
	class DefaultGoalPred
	{
	public:
		/**
		 * No particular goal
		 */
		bool operator()(typename GraphType::Identifier v)
		{
			return false;
		}
	};
	template<typename...Args>
	class Noop
	{
	public:
		void operator()(Args...args)
		{
		}
	};

	template<
		typename GraphType,
		typename IsGoalPred = DefaultGoalPred<GraphType>, 
		typename DiscoveryAction = Noop<typename GraphType::Identifier, 
		typename GraphType::Identifier>>
	class BFS
	{
		// Goal predicate. Called with argument the current vertex being processed.
		IsGoalPred m_goalPred;
		// Action to perform when a node is just discovered.
		DiscoveryAction m_discoveryAction;
	public:
		BFS(const IsGoalPred& goalPred, const DiscoveryAction& discoveryAction):
		m_goalPred(goalPred),
		m_discoveryAction(discoveryAction)
		{}

		using Identifier = typename GraphType::VertexIdentifier;

		/**
		 * \brief Applies BFS using the given action and predicate
		 * \param g The graph to apply BFS to
		 * \param startVertex The starting vertex for the search.
		 */
		void apply(const GraphType& g, Identifier startVertex)
		{
			BFSGraphArchetype a(g); // Check correct graph type.

			std::queue<Identifier> processQueue;
			std::vector<bool> discovered(g.vertexCount(), false);
			processQueue.push(startVertex);
			discovered[std::size_t(startVertex)] = true;
			while (!processQueue.empty())
			{
				auto v = processQueue.front();
				processQueue.pop();
				if (m_goalPred(v)) return;
				for(Identifier vAdj : g.adjacent(v))
				{
					if(!discovered[int(vAdj)])
					{
						discovered[int(vAdj)] = true;
						m_discoveryAction(v, vAdj);
						processQueue.push(vAdj);
					}
				}
			}
		}
	};
}

#endif