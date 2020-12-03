#ifndef LOOPSLIB_DS_HEAP_H
#define LOOPSLIB_DS_HEAP_H
#include <vector>
#include <map>
#include <cassert>

namespace LoopsLib::DS
{
    /**
     * \brief Min heap implementation
     * \tparam T 
     * \tparam Compare 
     */
    template<typename T, typename Id_t = int, typename Compare=std::less<T>>
    class Heap
    {
    public:
        struct Node
        {
            Id_t id;
            T value;
            Node():id(Id_t{}),value(T{}){}
            Node(Id_t id, T value):id(id),value(value){}
            Node& operator=(const Node& other)
            {
                id = other.id;
                value = other.value;
                return *this;
            }
        };

        bool verifyHeapProperty()
        {
            if (this->size() <= 1) return true;
            for (std::size_t i = 1; i < size(); ++i)
            {
                if (!compare(parent(i), i)) {
                    if (compare(i, parent(i))) return false;
                }
            }
            return true;
        }
    private:
        // Pair of ID and data
        std::vector<Node> m_data;
        // Maps ID to index in data.
        std::map<Id_t, std::size_t> m_idToData;
        // Counter for generating IDs if not specified.
        Id_t m_currentId = 0;

        Compare m_comp;


        bool hasId(Id_t id) const
        {
            return m_idToData.find(id) != m_idToData.end();
        }
        inline size_t parent(std::size_t self) const
        {
            return (self - 1) / 2;
        }
        inline size_t leftChild(size_t self) const
        {
            return 2 * self + 1;
        }
        inline bool hasLeftChild(size_t self) const
        {
            return leftChild(self) < m_data.size();
        }
        inline size_t rightChild(size_t self) const
        {
            return 2 * self + 2;
        }
        inline bool hasRightChild(size_t self) const
        {
            return rightChild(self) < m_data.size();
        }
        inline bool compare(size_t first, size_t second) const
        {
            return m_comp(m_data[first].value, m_data[second].value);
        }
        inline void swap(size_t firstIndex, size_t secondIndex)
        {
            auto firstId = m_data[firstIndex].id;
            auto secondId = m_data[secondIndex].id;
            std::swap(m_data[firstIndex], m_data[secondIndex]);
            m_idToData[firstId] = secondIndex;
            m_idToData[secondId] = firstIndex;
        }
        void heapifyUp(size_t start)
        {
            if (start == 0) return;
            auto current = start;
            auto parentInd = parent(current);
            while(current > 0 && !compare(parentInd, current))
            {
                swap(parentInd, current);
                current = parentInd;
                parentInd = parent(current);
            }
        }
        void heapifyDown(size_t start)
        {
            int current = start;
            while(true)
            {
                int highest = current;//Element that should be highest up the tree
                if(hasLeftChild(current) && !compare(highest, leftChild(current)))
                {
                    highest = leftChild(current);
                }
                if(hasRightChild(current) && !compare(highest, rightChild(current)))
                {
                    highest = rightChild(current);
                }
                if (highest != current)
                {
                    swap(highest, current);
                    current = highest;
                }
                else
                    break;
            }
        }
        void buildHeap()
        {
            if (m_data.size() == 1) return;

            for(size_t i = (m_data.size() / 2) ; i >= 1; --i)
            {
                heapifyDown(i-1);
            }
            assert(verifyHeapProperty());
        }
    public:
        Heap(){}
        Heap(const std::vector<T>& data)
        {
            for(std::size_t i = 0; i < data.size(); ++i)
            {
                m_data.push_back(Node{ i, data[i] });
                m_idToData[i] = i;
            }
            buildHeap();
        }
        Heap(const std::vector<T>& data, const std::vector<int>& ids)
        {
            assert(data.size() == ids.size());
            for (std::size_t i = 0; i < data.size(); ++i)
            {
                m_data.push_back(Node{ ids[i],data[i] });
                m_idToData[ids[i]] = i;
            }
            buildHeap();
        }

        std::size_t size() const
        {
            return m_data.size();
        }

        void setCompareFunctor(const Compare& compare)
        {
            m_comp = compare;
        }

        bool empty() const
        {
            return m_data.empty();
        }

        const Node& peak()
        {
            assert(!m_data.empty());

            return m_data[0];
        }

        bool containsId(const Id_t& id) const
        {
            return hasId(id);
        }

        T dataForId(Id_t id) const
        {
            return m_data[m_idToData.at(id)].value;
        }

        Node extract()
        {
            assert(!m_data.empty());

            Node returnVal = m_data[0];
            swap(0, m_data.size() - 1);
            m_data.resize(m_data.size() - 1);
            heapifyDown(0);
            m_idToData.erase(returnVal.id);
            assert(verifyHeapProperty());
            return returnVal;
        }

        /**
         * \brief Inserts or updates key in the queue. Returns true when inserted, false otherwise
         * \param val The value
         * \param id The ID to add
         * \return 
         */
        bool upsert(const T& val, Id_t id)
        {
            if(containsId(id)){
                updateKey(id,val);
                return false;
            }
            else
            {
                insert(val, id);
                return true;
            }
        }

        void insert(const T& val, Id_t id)
        {
            assert(!hasId(id));

            m_data.emplace_back(id, val);
            m_idToData[id] = m_data.size() - 1;
            heapifyUp(m_idToData[id]);
            verifyHeapProperty();
        }
        Id_t insert(const T& val)
        {
            while (hasId(m_currentId))
            {
                ++m_currentId;
            }
            insert(val, m_currentId);
            return m_currentId;
        }
        void updateKey(Id_t id, const T& val)
        {
            const int index = m_idToData[id];
            m_data[index] = Node{ id,val };
            heapifyUp(index);
            heapifyDown(index);
            assert(verifyHeapProperty());
        }
    };

}
#endif