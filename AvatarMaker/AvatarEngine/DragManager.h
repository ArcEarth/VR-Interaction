#pragma once
#include "GeometricsModifyTool.h"
#include "DXMathExtend.h"
namespace EditingTools
{
	class DragManager :
		public BasicTool
	{
	public:
		DragManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel);
		~DragManager(void);

		virtual void Start();
		virtual void Finish();

		virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);
		void Edit(const DirectX::Vector3& editPoint);

		bool ReshapePath(DirectX::CXMMATRIX Transform);
	protected:
		float m_PeakRadius;
		bool m_AppendingFlag;
		//std::vector<DirectX::Vector3> m_EditTrajectory;
		DirectX::SpaceCurveSampler m_EditTrajectory;

		std::vector<float> m_TransformWeights;
		//DirectX::Vector3 m_Displacement;
		//DirectX::SimpleMath::Matrix	 m_InverseTransformMatrix;
	};
} 
