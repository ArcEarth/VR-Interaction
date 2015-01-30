#pragma once
#include "GeometricsModifyTool.h"
#include "DXMathExtend.h"
//#include <array>
#include <deque>


namespace EditingTools
{
	class SweepManager :
		public BasicTool
	{
	public:
		SweepManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel);
		~SweepManager(void);

		virtual void Start();
		virtual void Finish();

		virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);
		void Edit(const DirectX::Vector3& editPoint);

		void SculptByArmTrajectory(const DirectX::Vector3& vHand,const DirectX::Vector3& vWrist,const DirectX::Vector3& vElbow,const DirectX::Vector3& vShoulder);

		bool ReshapeSurface(DirectX::CXMMATRIX Transform);
		//void AppendMetaballsByPoint(DirectX::FXMVECTOR EndPoint);
	protected:
		DirectX::SpaceCurveSampler	m_EditTrajectory; // The trajectory of hands
		DirectX::Vector3			m_Core;
		std::vector<float>			m_TransformWeights;
		//		std::vector<DirectX::Vector3> m_EditTrajectory;
	};

}
