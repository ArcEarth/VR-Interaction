#include "stdafx.h"
#include "GrowManager.h"
#include <iostream>
using namespace DirectX;
using namespace std;
using namespace Kinematics;
using namespace EditingTools;

GrowManager::GrowManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel)
	: BasicTool(pTargetModel,pLocalModel)
{
}

GrowManager::~GrowManager(void)
{
}

void GrowManager::Edit(const DirectX::Vector3& pHand,const DirectX::Vector3& pWrist,const DirectX::Vector3& pElbow,const DirectX::Vector3& pShoulder)
{
	Vector3 Chain[4] = {pShoulder,pElbow,pWrist,pHand};
	size_t i = 0;
	for (; i < 4; i++)
	{
		if (!m_pTargetModel->Volume().Contains(Chain[i])) break;
	}
	if (i>=4 || i==0) 
	{
		cout << "Whole arm inside / outside , Abort"<<endl;
		return;
	}

	m_pLocalModel->Clear();
	bool first = true;
	for (; i < 4; i++)
	{
		m_pLocalModel->AddCylinder(Chain[i-1],Chain[i],Radius(),Radius(),!first);
		first = false;
	}

	m_pLocalModel->Update();
}
