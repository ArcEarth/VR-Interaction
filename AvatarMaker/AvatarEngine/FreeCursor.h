#pragma once
#include "EditingToolsInterface.h"
#include "SpaceWarpperInterface.h"
#include "Carmera.h"
#include "Lights.h"
#include "DynamicMetaBallModel.h"

namespace EditingTools{

	class FreeCursor
		: public IAreaTool
	{
	public:
		FreeCursor(ID3D11DeviceContext *pContext , const std::shared_ptr<Geometrics::DynamicMetaBallModel>& pTarget ,const ICamera *pCamera, SphereLightData* pLight);
		virtual ~FreeCursor();

		virtual void Start();
		virtual void Finish();
		virtual void Abort();
		//virtual void Edit(const std::vector<HolographicPoint>& EditingPoints);
		virtual void Edit(const DirectX::Vector3& pHand,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);

//		void SetReferenceTarget(DynamicMetaBallModel* pTarget);

		DirectX::IRenderObject* Visualization();

		virtual float Radius() const;
		virtual bool SetRadius(float Radius);
		virtual bool TurnRadius(float deltaRadius);
		virtual bool IsActive() const;

	protected:
		bool				Snaped;
		std::shared_ptr<Geometrics::DynamicMetaBallModel> m_pTarget;
		//IWarpper			*m_pWarpper;

		class Preview;
		std::unique_ptr<Preview> m_pPreview;
	};

}
