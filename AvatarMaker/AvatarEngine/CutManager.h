#pragma once
#include "editingtoolsinterface.h"
#include "DynamicMetaBallModel.h"
#include "DXMathExtend.h"
#include "SpaceWarpperInterface.h"
#include <memory>

namespace EditingTools{
	class CutManager :
		public IAreaTool
	{
	public:
		CutManager( ID3D11DeviceContext *pContext,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const ICamera* pCamera );
		~CutManager(void);

		virtual void Start();

		virtual void Finish();

		virtual void Abort();

		virtual void Edit( const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3& );

		virtual bool IsActive() const {return _Active;}

		virtual float Radius() const;

		virtual bool SetRadius( float Radius );

		virtual bool TurnRadius( float deltaRadius );

		virtual DirectX::IRenderObject* Visualization();

		inline bool CutedFlag() const { return m_cutedFlag; }

		void ContructCutsurfaceFromTrajectory();

		bool CutSubject();

	protected:
		static const unsigned int CuttingCurvTessellation = 5;

		//IWarpper* m_pWarpper;
		std::shared_ptr<Geometrics::DynamicMetaBallModel> m_pTargetModel;
		//			Geometrics::DynamicMetaBallModel::Package m_pBackup;
		DirectX::SpaceCurveSampler m_Trajectories[2];
		std::vector<DirectX::Vector3> m_CuttingSurface[CuttingCurvTessellation];

		class Visual;
		std::unique_ptr<Visual> m_pVisual;
		bool _Active;
		int m_DirtyFlag;
		bool m_cutedFlag;
	};

}

