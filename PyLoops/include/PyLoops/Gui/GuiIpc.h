#ifndef PYLOOPS_GUIMESSAGES_GUIIPC_H
#define PYLOOPS_GUIMESSAGES_GUIIPC_H
#include <PyLoops/PyLoops.inc.h>
#include <GuiMessages/BaseMessage.h>
#include <tuple>

namespace PyLoops::Gui
{
    class GuiIpc
    {
        GuiMessages::BaseMessage* handleMessage();

        void sendMessage(const GuiMessages::BaseMessage& message);

        GuiMessages::BaseMessage* sendMessageWithReturn(const GuiMessages::BaseMessage& message);
    public:
        using ReceiveFunc = std::function<pybind11::bytes()>;
        using SendFunc = std::function<void(const pybind11::bytes&)>;
    private:
        ReceiveFunc m_receive;
        SendFunc m_send;
    public:
        GuiIpc(ReceiveFunc receive, SendFunc send);

        void test(const std::string& str);

        void showTrajectory(const LoopsLib::MovetkGeometryKernel::Trajectory& trajectory, const std::string& name, bool convertCrs);
        void showTrajectory(const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& trajectory, const std::string& name,bool convertCrs);
        void showGraphTrajectory(const LoopsLib::MovetkGeometryKernel::GraphTrajectory& trajectory, const std::string& name);
        void markVertex(long long vertex, bool resetRest);
        void loadMap(const std::string& mapPath);

        static void registerPy(pybind11::module& mod);
    };
}

#endif