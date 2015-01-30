#pragma once
#include "DynamicMetaballModel.h"
#include "EditingToolsInterface.h"
#include "SpaceWarpperInterface.h"

namespace EditingTools
{
	static const float TrajectoryDensity = 0.01f;

	// An base for geometry addtive tool
	class BasicTool
		: public IAreaTool
	{
	public:
		BasicTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel);
		virtual ~BasicTool(void);

		virtual void Start();
		virtual void Finish();
		virtual void Abort();

		virtual bool IsActive() const{
			return _Active;
		}

		DirectX::IRenderObject* Visualization();

		virtual bool SetRadius(float Radius);
		virtual bool TurnRadius(float deltaRadius);

	protected:
		//The safe-functions to manipulate the joints radius
		const bool TurnJointRadius(unsigned index,float deltaR);
		const bool SetJointRadius(unsigned index,float Radius);
		const bool TurnAllJointsRadius(float deltaR);
		const bool SetAllJointsRadius(float Radius);

	protected:

		//static void InterpolateOnLineSegment(Geometrics::MetaBallModel& Target , DirectX::CXMVECTOR St , DirectX::CXMVECTOR Ed , float Rs , float Re , unsigned int BindIndex);

	protected:
		bool _Active;

		std::shared_ptr<Geometrics::DynamicMetaBallModel> m_pTargetModel;
		std::shared_ptr<Geometrics::DynamicMetaBallModel> m_pLocalModel;
		//Geometrics::DynamicMetaBallModel* m_pTargetModel;
		//Geometrics::DynamicMetaBallModel *m_pLocalModel;

		//Kinematics::Joint* m_pBindingBone;
		//std::shared_ptr<const IWarpper> m_pWarpper;
		//IWarpper* m_pWarpper;

	};

} 
