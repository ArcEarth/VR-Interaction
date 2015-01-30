#pragma once
#include "GeometricsModifyTool.h"

namespace EditingTools
{
	class GrowManager :
		public BasicTool
	{
	public:
		GrowManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel);
		~GrowManager(void);

		//virtual void Start();

		virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);
		void Edit(const DirectX::Vector3& editPoint);
	};
} 
