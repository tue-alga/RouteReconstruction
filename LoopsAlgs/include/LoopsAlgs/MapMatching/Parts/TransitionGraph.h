/**
 * HMM transition graph
 */

#ifndef FMM_TEMPLATED_TRANSITION_GRAPH_HPP
#define FMM_TEMPLATED_TRANSITION_GRAPH_HPP

#include <LoopsLib/Helpers/Logger.h>
#include <LoopsLib/Algs/Types.h>
#include "LoopsLib/GraphAlgs/ShortestPath.h"

namespace LoopsAlgs::MapMatching::Parts
{
    template<typename NT>
    struct Candidate{
        std::size_t edgeId;
        NT distance;
        NT offset;
    };

    /**
     * Transition graph class in HMM.
     *
     * The class stores the underlying transition graph of a HMM, which stores
     * the probabilities of candidates matched to a trajectories.
     * @tparam EP_Provider Emission probability provider function
     * @tparam TP_Provider Transmission probability provider function
     */
    template<typename NT, typename EP_Provider, typename TP_Provider>
    class TransitionGraph
    {
        // Transmission probability provider functor
        // Arguments: const Candidate& src, const Candidate& target
        // Expected output: st
        TP_Provider* m_tpProvider = nullptr;
        // 
        EP_Provider* m_epProvider = nullptr;
        
        // Value for elements that are not set.
        static inline std::size_t m_notSet = std::numeric_limits<std::size_t>::max();

        TP_Provider& tpProvider()
        {
            return *m_tpProvider;
        }
        EP_Provider& epProvider()
        {
            return *m_epProvider;
        }
        bool m_verbose = false;
    public:
        // Node in the transitionGraph
        struct Node {
            std::size_t candidateIdx = m_notSet; // The id of the candidate associated with this node.
            std::size_t previousOptimal = m_notSet; // The previous optimal node
            NT emissionProb = 0.0; // The emission probability for this node
            NT transmissionProb = 0.0; // The best transmission probability 
            NT cumulativeProb = 0.0; // The best cumulative probability

            Node(){}
            Node(std::size_t candidateIdx):candidateIdx(candidateIdx){}
            Node(std::size_t candidateIdx, NT emssionProb) :candidateIdx(candidateIdx),emissionProb(emssionProb){}
        };
        // Single layer is a vector of nodes
        using Layer = std::vector<Node>;
    private:
        std::vector<Layer> m_layers;

        std::string m_failMessage;

        template<typename...Args>
        void verbosePrint(Args&&...args) const
        {
            if (!m_verbose) return;
            using exp = int[];
            (void) exp{
                (std::cout << args,0)...
            };
            std::cout << '\n';
        }
    public:
        /**
         * Transition graph constructor.
         *
         * A transition graph is created to store the probability of
         * emission and transition in HMM model where optimal path will be
         * infered.
         *
         * @param tc        Trajectory candidates
         * @param gps_error GPS error
         */
        TransitionGraph()
        {
            static_assert(std::is_same_v<
                std::pair<bool, NT>, 
                std::invoke_result_t<decltype(&TP_Provider::operator()), TP_Provider&, std::size_t, std::size_t, std::size_t, std::size_t>
            >,
            "Expected TP_Provider type to return std::pair<bool, NT> on operator()(std::size_t currentLayer, std::size_t srcCandidate, std::size_t targetCandidate)");
        }
        void setVerbose(const bool& verbose)
        {
            m_verbose = verbose;
        }
        bool verbose() const
        {
            return m_verbose;
        }


        std::size_t layerCount() const
        {
            return m_layers.size();
        }
        const Layer& layer(std::size_t ind) const
        {
            return m_layers.at(ind);
        }

        /**
         * Find the optimal candidate in a layer with
         * the highest accumulative probability.
         * @param  layer [description]
         * @return  pointer to the optimal node in the transition graph
         */
        bool findOptimalCandidate(const Layer &layer, std::size_t& optimal) const
        {
            auto el  =std::max_element(layer.begin(), layer.end(), 
                [](const auto& n0, const auto& n1){return n0.cumulativeProb < n1.cumulativeProb;}
            );
            if (el == layer.end()) return false;
            optimal = std::distance(layer.begin(), el);
            return true;
        }

        /**
         * \brief If the algorithm failed, return true and set the fail message.
         * This may happen when a path could not be constructed. 
         * \param message The message with which the algorithm failed
         * \return Did the algorithm fail.
         */
        bool didFail(std::string& message) const
        {
            if (m_failMessage.empty()) return false;
            message = m_failMessage;
            return true;
        }

        /**
         * \brief Computes all probabilities and finds the optimal transitions.
         * \tparam TrajectoryCandidatesIt Iterator type of trajectory candidates container. Should return candidates for successive layers
         * \tparam CandidateIteratorsSupplier Type of the function that converts a TrajectoryCandidatesIt into a pair of iterators over all candidates for the underlying layer
         * \param epProvider Emission probability provider
         * \param tpProvider Transmission probability provider
         * \param begin Begin of the trajectory candidates
         * \param end End of the trajectory candidates
         * \param candidateIteratorFunc Function that converts candidate iterator to a pair of iterators, specifying the begin and end of all candidates for the current layer.
         */
        template<typename TrajectoryCandidatesIt, typename CandidateIteratorsSupplier>
        void compute(EP_Provider& epProvider, TP_Provider& tpProvider,TrajectoryCandidatesIt begin, TrajectoryCandidatesIt end, CandidateIteratorsSupplier candidateIteratorFunc)
        {
            m_epProvider = &epProvider;
            m_tpProvider = &tpProvider;

            m_layers.clear();
            m_layers.resize(std::distance(begin, end),{});
            TrajectoryCandidatesIt curr = begin;
            std::size_t layer = 0;
            while(curr!=end)
            {
                std::size_t candidate = 0;
                auto iteratorPair = candidateIteratorFunc(curr);
                for(auto candidateIt = iteratorPair.first; candidateIt != iteratorPair.second; ++candidateIt)
                {
                    NT ep = this->epProvider()(layer,candidateIt);
                    m_layers[layer].push_back(Node(candidate,ep));
                    ++candidate;
                }
                ++layer;
                ++curr;
            }
            // Compute transitions between layers
            bool restartNext = false;
            for(std::size_t j = 0; j < m_layers.size()-1; ++j)
            {
                if(!computeTransition(j,restartNext))
                {
                    verbosePrint("No transition from layer ", j, " to next under current model in HMM");
                    if(m_verbose)
                    {
                        if (m_layers[j].empty()) verbosePrint("Empty layer ", j);
                        if (m_layers[j + 1].empty()) verbosePrint("Empty layer ", j+1);
                        tpProvider.setVerbose(true);
                        // Rerun for output
                        computeTransition(j, restartNext);
                        tpProvider.setVerbose(false);
                    }
                    restartNext = true;
                }
                else
                {
                    restartNext = false;
                }
            }
        }
        /**
         * Reconstruct optimal path in the transition graph
         */
        std::vector<std::size_t> reconstructPath(std::string& failReason, std::size_t& startLayer) const
        {
            if (!m_failMessage.empty()) {
                failReason = m_failMessage;
                return {};
            }

            std::size_t optimalCandidate = 0;

            int currLayer = layerCount() - 1;
            // For some magical reason, could not find max element
            for(; currLayer >= 1; --currLayer)
            {
                if (findOptimalCandidate(m_layers[currLayer], optimalCandidate))
                {
                    break;  
                }
            }
            if (currLayer == 0) 
            {
                verbosePrint("No optimal layer present");
                return {};
            }

            std::vector<std::size_t> optimalPath;
            std::vector<std::size_t> currPath;

            optimalPath.reserve(m_layers.size());
            currPath.push_back(optimalCandidate);
            auto curr = optimalCandidate;
            for(; currLayer >= 1; --currLayer)
            {
                auto prev = curr;
                curr = m_layers[currLayer][curr].previousOptimal;
                if(curr == m_notSet)
                {
                    verbosePrint("\t Found partial path up to layer ", currLayer);
                    if(currPath.size() > optimalPath.size())
                    {
                        optimalPath = currPath;
                        startLayer = currLayer;
                    }
                    currPath = {};
                    currLayer -= 1;
                    // Look for a new candidate the in the new layer
                    for (; currLayer >= 1; --currLayer)
                    {
                        if (findOptimalCandidate(m_layers[currLayer], optimalCandidate))
                        {
                            break;
                        }
                    }
                    // One will be subtracted at the loop end
                    currLayer += 1;
                    curr = optimalCandidate;
                    currPath.push_back(optimalCandidate);
                    // Skip one
                    continue;
                }
                currPath.push_back(curr);
            }
            if (currPath.size() > optimalPath.size()) {
                optimalPath = currPath;
                startLayer = 0;
            }
            // Reverse  the path
            std::reverse(optimalPath.begin(), optimalPath.end());

            std::cout << " Opt path: ";
            std::ostream_iterator<std::size_t> out(std::cout, ",");
            std::copy(optimalPath.begin(), optimalPath.end(), out);
            std::cout << std::endl;

            return optimalPath;
        }
    private:
        bool computeTransition(std::size_t srcLayer, bool isRestart)
        {
            auto& startLayer = m_layers[srcLayer];
            auto& endLayer = m_layers[srcLayer+1];
            auto logger = LoopsLib::Helpers::logFactory(this);
            bool wasAssigned = false;
            // Work from target layer
            for (std::size_t i = 0; i < endLayer.size(); ++i) {
                auto& targetNode = endLayer[i];

                for (std::size_t j = 0; j < startLayer.size(); ++j) {
                    const auto& startNode = startLayer[j];
                    // No path exists from start of trajectory to the node,
                    // so don't bother.
                    if (startNode.previousOptimal == m_notSet && srcLayer != 0 && !isRestart) continue;
                    
                    auto resPair = tpProvider()(srcLayer, startNode.candidateIdx, srcLayer+1, targetNode.candidateIdx);
                    const auto tp = resPair.second;
                    // No transition exists: continue
                    if (!resPair.first) continue;

                    logger.trace("Transition of level ", srcLayer, ": tp=", tp);

                    if (startNode.cumulativeProb + tp * targetNode.emissionProb >= targetNode.cumulativeProb) {
                        targetNode.cumulativeProb = startNode.cumulativeProb + tp * targetNode.emissionProb;
                        targetNode.previousOptimal = j;
                        targetNode.transmissionProb = tp;
                        wasAssigned = true;
                    }
                }
            }
            logger.trace("Update layer done");
            return wasAssigned;
        }
    };
}
#endif 