#pragma once
#include "geometricsmodifytool.h"
#include "DXMathExtend.h"
#include "TriangleMesh.h"
//#include <deque>

namespace EditingTools{
	class VolumeSculptManager : 
		public BasicTool
	{
	public:
		static const unsigned int TargetSurfaceTesselation = 12U;
		static_assert(TargetSurfaceTesselation > 1,"Surface tessellation must > 1");
		typedef Geometrics::Bezier::BezierPatch<DirectX::Vector3,3U> CubicBezierPatch;
		typedef CubicBezierPatch::ClippingType CubicBezierCurve;

		VolumeSculptManager(ID3D11DeviceContext *pContext ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel ,const ICamera* pCamera);  
		~VolumeSculptManager(void);

		virtual void Start();
		virtual void Finish();
		virtual DirectX::IRenderObject* Visualization();

		virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);
		virtual void Edit(_In_reads_(8) const DirectX::Vector3* EditingPoints);

	protected:
		void ConstructSkeleton( DirectX::FXMVECTOR BoneEndPoint );

		bool ReshapeVolume(DirectX::CXMMATRIX Transform);
		void TesselateBezierPatch(const VolumeSculptManager::CubicBezierPatch &patch, size_t tessellation,DirectX::FXMVECTOR Color);
		void AppendToTargetSurface(DirectX::FXMVECTOR LPoint, DirectX::FXMVECTOR RPoint);

		void ConstructeEndupBoundry();
		void IterativeApproximate();

		void ApproximateVolum(_Out_ Geometrics::MetaBallModel &Volume,_In_ const Geometrics::MetaBallModel &Subject);

		DirectX::Vector3 FindTendencyPointOnSurface( const DirectX::Vector3 &p0,const DirectX::Vector3 &p1,const DirectX::Vector3 &p2,const DirectX::Vector3 &defaultTarget );
	protected:
		bool _hand_switch;

		Geometrics::PolygonSoup<DirectX::VertexPositionColor> m_TargetMesh;
		std::vector<DirectX::Vector3> m_TargetCenterAxis;
		//std::vector<DirectX::Vector3> m_TargetSurface[TargetSurfaceTesselation];
		//		std::vector<DirectX::Vector3>  m_CentrePath,m_LeftPath,m_RightPath,m_SpherePath;

		DirectX::Vector3 m_LTDisp,m_RTDisp;

		DirectX::SpaceCurveSampler EditingTrajectories[2][4];
		Geometrics::Bezier::BezierPatch<DirectX::Vector3,3> Patches[6];
		std::vector<float> m_TransformWeights;

		class Visual;
		std::unique_ptr<Visual> m_pVisual;
	};
} 
