#pragma once
#include "MathHelper.h"
#include "ModelInterface.h"

namespace EditingTools {
	static const float Editing_Tools_Default_Radius = 0.05f;
	static const float Metabll_Radius_Display_Factor = 2.0f;

	class IEditingTool
	{
	public:
		// Reset all the state and make ready for editing
		virtual void Start() = 0;
		virtual void Finish() = 0;
		virtual void Abort() = 0;
		// the data should be HAND , WRIST , ELBOW , SHOULDER since we only have arm to use as a tool
		virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&) = 0;
		//			virtual void Edit(const std::vector<HolographicPoint>& EditingPoints) = 0;

		virtual bool IsActive() const = 0;
		
		//// The generic description about this tool's affection
		//virtual float Radius() const = 0;
		//virtual bool SetRadius(float Radius) = 0;
		//virtual bool TurnRadius(float deltaRadius) = 0;

		// the Interface for rendering
		virtual DirectX::IRenderObject* Visualization() = 0;
	};

	class IAreaTool
		: public IEditingTool
	{
	public:
		IAreaTool()
			:_radius(Editing_Tools_Default_Radius)
		{}
		IAreaTool(float Radius)
			:_radius(Radius)
		{}

		virtual ~IAreaTool() {}

		// The generic description about this tool's affection
		inline float Radius() const{return _radius;}
		virtual bool  SetRadius(float Radius) { _radius = Radius; return true;};
		virtual bool  TurnRadius(float deltaRadius) { _radius += deltaRadius; return true;}


	protected:
		float _radius;
	};

	class IDoubleHandTool : public IEditingTool
	{
		void Edit(const DirectX::Vector3&p1,const DirectX::Vector3&p2,const DirectX::Vector3&p3,const DirectX::Vector3&p4)
		{
			if (!IsActive()) return;
			static DirectX::Vector3 _buffer[8];
			if (_hand_switch)
			{
				//std::cout<<"+!"<<std::endl;
				_buffer[4] = p1;
				_buffer[5] = p2;
				_buffer[6] = p3;
				_buffer[7] = p4;
				Edit(_buffer);
			}else
			{
				//std::cout<<"-!"<<std::endl;
				_buffer[0] = p1;
				_buffer[1] = p2;
				_buffer[2] = p3;
				_buffer[3] = p4;
			}
			_hand_switch = !_hand_switch;
		}
		virtual void Edit(_In_reads_(8) const DirectX::Vector3* EditingPoints) = 0;

		protected:
		/// <summary>
		/// indicator for which hand is perform this editing , false = L , false = Right , RESET it in Starting;
		/// </summary>
		bool _hand_switch;
	};
}
