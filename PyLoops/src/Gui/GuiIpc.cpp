#include <PyLoops/Gui/GuiIpc.h>
#include <GuiMessages/Messages.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

GuiMessages::BaseMessage* PyLoops::Gui::GuiIpc::handleMessage()
{
    using namespace GuiMessages;
    BytesBuffer buff(512);
    auto& factory = MessageFactory::inst();
    bool timedOut = false;

    while(!factory.isFullMessage(buff) && !timedOut)
    {
        auto data = m_receive();
        std::string dataStr = data;
        // Should be uninterpreted
        buff.copyRaw(dataStr.data(), dataStr.size());
    }
    if(timedOut)
    {
        throw std::runtime_error("Timeout while receiving message");
    }

    return factory.messageFromBytes(buff);
}

void PyLoops::Gui::GuiIpc::sendMessage(const GuiMessages::BaseMessage& message)
{
    GuiMessages::BytesBuffer buff(512);
    GuiMessages::MessageFactory::inst().encodeMessage(message, buff);
    auto dataDesc = buff.readableRange().toPair();
    m_send(pybind11::bytes(dataDesc.first, dataDesc.second));
}


GuiMessages::BaseMessage* PyLoops::Gui::GuiIpc::sendMessageWithReturn(const GuiMessages::BaseMessage& message)
{
    sendMessage(message);
    return handleMessage();
}


PyLoops::Gui::GuiIpc::GuiIpc(ReceiveFunc receive,
    SendFunc send):
    m_send(send),
m_receive(receive)
{
}

void PyLoops::Gui::GuiIpc::test(const std::string& str)
{
    GuiMessages::TestMessage tm;
    tm.setValue(str);
    sendMessage(tm);
}

void PyLoops::Gui::GuiIpc::showTrajectory(const LoopsLib::MovetkGeometryKernel::Trajectory& trajectory, const std::string& name, bool convertCrs)
{
    GuiMessages::ShowTrajectoryMessage stm;
    stm.setConvertfromcrs(convertCrs);
    stm.setTrajectory(trajectory);
    stm.setName(name);
    sendMessage(stm);
}

void PyLoops::Gui::GuiIpc::showTrajectory(const LoopsLib::MovetkGeometryKernel::TimestampedTrajectory& trajectory, const std::string& name, bool convertCrs)
{
    GuiMessages::ShowTrajectoryMessage stm;
    stm.setConvertfromcrs(convertCrs);
    LoopsLib::MovetkGeometryKernel::Trajectory newTraj;
    std::transform(trajectory.begin(), trajectory.end(), std::back_inserter(newTraj), [](const auto& pnt) {return pnt.first; });
    stm.setTrajectory(newTraj);
    stm.setName(name);
    sendMessage(stm);
}

void PyLoops::Gui::GuiIpc::showGraphTrajectory(const LoopsLib::MovetkGeometryKernel::GraphTrajectory& trajectory, const std::string& name)
{
    GuiMessages::ShowGraphTrajectoryMessage stm;
    stm.setTrajectory(trajectory);
    stm.setName(name);
    sendMessage(stm);
}

void PyLoops::Gui::GuiIpc::markVertex(long long vertex, bool resetRest)
{
    GuiMessages::MarkVertexMessage mvm;
    mvm.setResetrest(resetRest);
    mvm.setVertex(vertex);
    sendMessage(mvm);
}

void PyLoops::Gui::GuiIpc::loadMap(const std::string& mapPath)
{
    GuiMessages::LoadMapMessage lmm;
    lmm.setMap(mapPath);
    sendMessage(lmm);
}

void PyLoops::Gui::GuiIpc::registerPy(pybind11::module& mod)
{
    using Kernel = LoopsLib::MovetkGeometryKernel;
    namespace py = pybind11;
    using IPC = PyLoops::Gui::GuiIpc;
    py::class_<IPC>(mod, "GuiIpc")
        .def(py::init<ReceiveFunc, SendFunc>())
        .def("test", &IPC::test)
        .def("showTrajectory", py::overload_cast<const Kernel::Trajectory&, const std::string&, bool>(&IPC::showTrajectory), 
            py::arg("trajectory"),
            py::arg("name"),
            py::arg("convertCrs") = true)
        .def("showTrajectory", py::overload_cast<const Kernel::TimestampedTrajectory&, const std::string&,bool>(&IPC::showTrajectory), 
            py::arg("trajectory"),
            py::arg("name"),
            py::arg("convertCrs") = true)
        .def("showGraphTrajectory", &IPC::showGraphTrajectory)
        .def("markVertex", &IPC::markVertex, py::arg("vertex"),py::arg("resetRest")=false)
        .def("loadMap", &IPC::loadMap)
    ;
}
