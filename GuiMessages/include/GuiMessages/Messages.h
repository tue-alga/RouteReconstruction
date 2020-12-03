#ifndef GUIMESSAGES_MESSAGES_H
#define GUIMESSAGES_MESSAGES_H
#include "BaseMessage.h"
#include <LoopsLib/Algs/Types.h>

namespace GuiMessages
{
	class StringArgMessage : public BaseMessage
	{
	protected:
		std::string m_value;
	public:
		std::string value() const { return m_value;}
		void setValue(const std::string& value){m_value = value;}

		std::string type() const override { return "stringarg";}

		void encode(BytesBuffer& buff) const override
		{
			buff << m_value;
		}
		
		void decode(BytesBuffer& buff) override
		{
			buff >> m_value;
		}
	};
	class ShowFieldMessage : public BaseMessage
	{
	protected:
		std::vector<LoopsLib::NT> m_field;
	public:
		std::vector<LoopsLib::NT> field() const { return m_field;}
		void setField(const std::vector<LoopsLib::NT>& field){m_field = field;}

		std::string type() const override { return "SetField";}

		void encode(BytesBuffer& buff) const override
		{
			buff << m_field;
		}
		
		void decode(BytesBuffer& buff) override
		{
			buff >> m_field;
		}
	};
	class LoadMapMessage : public BaseMessage
	{
	protected:
		std::string m_map;
	public:
		std::string map() const { return m_map;}
		void setMap(const std::string& map){m_map = map;}

		std::string type() const override { return "LoadMap";}

		void encode(BytesBuffer& buff) const override
		{
			buff << m_map;
		}
		
		void decode(BytesBuffer& buff) override
		{
			buff >> m_map;
		}
	};
	class TestMessage : public BaseMessage
	{
	protected:
		std::string m_value;
	public:
		std::string value() const { return m_value;}
		void setValue(const std::string& value){m_value = value;}

		std::string type() const override { return "Test";}

		void encode(BytesBuffer& buff) const override
		{
			buff << m_value;
		}
		
		void decode(BytesBuffer& buff) override
		{
			buff >> m_value;
		}
	};
	class ShowTrajectoryMessage : public BaseMessage
	{
	protected:
		LoopsLib::MovetkGeometryKernel::Trajectory m_trajectory;
		std::string m_styleProvider;
		bool m_convertFromCRS = true;
		std::string m_name;
	public:
		LoopsLib::MovetkGeometryKernel::Trajectory trajectory() const { return m_trajectory;}
		void setTrajectory(const LoopsLib::MovetkGeometryKernel::Trajectory& trajectory){m_trajectory = trajectory;}
		std::string styleProvider() const { return m_styleProvider;}
		void setStyleprovider(const std::string& styleProvider){m_styleProvider = styleProvider;}
		bool convertFromCRS() const { return m_convertFromCRS;}
		void setConvertfromcrs(const bool& convertFromCRS){m_convertFromCRS = convertFromCRS;}
		std::string name() const { return m_name;}
		void setName(const std::string& name){m_name = name;}

		std::string type() const override { return "ShowTrajectory";}

		void encode(BytesBuffer& buff) const override
		{
			buff << m_trajectory << m_styleProvider << m_convertFromCRS << m_name;
		}
		
		void decode(BytesBuffer& buff) override
		{
			buff >> m_trajectory >> m_styleProvider >> m_convertFromCRS >> m_name;
		}
	};
	class ShowGraphTrajectoryMessage : public BaseMessage
	{
	protected:
		LoopsLib::MovetkGeometryKernel::GraphTrajectory m_trajectory;
		std::string m_styleProvider;
		std::string m_name;
	public:
		LoopsLib::MovetkGeometryKernel::GraphTrajectory trajectory() const { return m_trajectory;}
		void setTrajectory(const LoopsLib::MovetkGeometryKernel::GraphTrajectory& trajectory){m_trajectory = trajectory;}
		std::string styleProvider() const { return m_styleProvider;}
		void setStyleprovider(const std::string& styleProvider){m_styleProvider = styleProvider;}
		std::string name() const { return m_name;}
		void setName(const std::string& name){m_name = name;}

		std::string type() const override { return "ShowGraphTrajectory";}

		void encode(BytesBuffer& buff) const override
		{
			buff << m_trajectory << m_styleProvider << m_name;
		}
		
		void decode(BytesBuffer& buff) override
		{
			buff >> m_trajectory >> m_styleProvider >> m_name;
		}
	};
	class MarkVertexMessage : public BaseMessage
	{
	protected:
		long long m_vertex;
		bool m_resetRest = false;
	public:
		long long vertex() const { return m_vertex;}
		void setVertex(const long long& vertex){m_vertex = vertex;}
		bool resetRest() const { return m_resetRest;}
		void setResetrest(const bool& resetRest){m_resetRest = resetRest;}

		std::string type() const override { return "MarkVertex";}

		void encode(BytesBuffer& buff) const override
		{
			buff << m_vertex << m_resetRest;
		}
		
		void decode(BytesBuffer& buff) override
		{
			buff >> m_vertex >> m_resetRest;
		}
	};
	using AllMessages = std::tuple<StringArgMessage,ShowFieldMessage,LoadMapMessage,TestMessage,ShowTrajectoryMessage,ShowGraphTrajectoryMessage,MarkVertexMessage>;
}
#endif