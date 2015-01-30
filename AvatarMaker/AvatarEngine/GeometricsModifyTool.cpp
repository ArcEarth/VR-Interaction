#include "stdafx.h"
#include "GeometricsModifyTool.h"

static const bool Remesh_When_Finish = false; 
using namespace EditingTools;
using namespace Geometrics;
using namespace DirectX;

#define Metaballs (m_pLocalModel->RestVolume())
#define  SUPERIOR_LIMIT Metaball_Radius_Superior_Limit
#define  INFIEROR_LIMIT Metaball_Radius_Infieror_Limit

BasicTool::BasicTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel)
	: m_pLocalModel(pLocalModel) , m_pTargetModel(pTargetModel) , _Active(false) //, m_pVolume(m_pLocalModel?(&m_pLocalModel->RestVolume):nullptr)
{
}


BasicTool::~BasicTool(void)
{
}

void BasicTool::Start(){
	if (_Active) 
		return;
	m_pLocalModel->Clear();
	_Active = true;
}

void BasicTool::Finish(){
	if (!IsActive()) return;

	m_pTargetModel->AddVolume(Metaballs);

	m_pTargetModel->Update();

	m_pLocalModel->Clear();
	_Active = false;
}

void BasicTool::Abort(){
	if (!IsActive()) return;
	m_pLocalModel->Clear();
	_Active = false;
}

const bool BasicTool::TurnAllJointsRadius(float deltaR){
	if (!_Active) return false;
	for (unsigned int i=0;i<Metaballs.size();i++){
		Metaballs[i].Radius += deltaR;
		if (Metaballs[i].Radius<INFIEROR_LIMIT) Metaballs[i].Radius = INFIEROR_LIMIT;
		if (Metaballs[i].Radius>SUPERIOR_LIMIT) Metaballs[i].Radius = SUPERIOR_LIMIT;
	}
	return true;
}

const bool BasicTool::SetAllJointsRadius(float Radius){
	if (!_Active) return false;
	for (unsigned int i=0;i<Metaballs.size();i++){
		Metaballs[i].Radius = Radius;
	}
	return true;
}

const bool BasicTool::SetJointRadius(unsigned index,float Radius){
	if (!_Active) return false;
	Metaballs[index].Radius = Radius;
	return true;
}

const bool BasicTool::TurnJointRadius(unsigned index, float deltaR){
	if (!_Active || Metaballs[index].Radius<=INFIEROR_LIMIT || Metaballs[index].Radius>=SUPERIOR_LIMIT) return false;
	Metaballs[index].Radius += deltaR;
	if (Metaballs[index].Radius<INFIEROR_LIMIT) Metaballs[index].Radius = INFIEROR_LIMIT;
	if (Metaballs[index].Radius>SUPERIOR_LIMIT) Metaballs[index].Radius = SUPERIOR_LIMIT;
	return true;
}

bool BasicTool::SetRadius(float Radius){
	if (Radius<INFIEROR_LIMIT) return false;
	if (Radius>SUPERIOR_LIMIT) return false;
	_radius = Radius;
	if (SetAllJointsRadius(Radius))
		m_pLocalModel->Update();
	return true;
}

bool BasicTool::TurnRadius(float delta){
	if (_radius + delta < INFIEROR_LIMIT)
	{
		delta = INFIEROR_LIMIT - _radius;
	} else if (_radius + delta > SUPERIOR_LIMIT)
	{
		delta = SUPERIOR_LIMIT - _radius;
	}

	if (abs(delta) < 0.01f) return false;

	_radius += delta;

	if (TurnAllJointsRadius(delta))
		m_pLocalModel->Update();
	return true;
}

DirectX::IRenderObject* BasicTool::Visualization()
{
	if (!_Active) 
	{
		//			cout<<"Can't viusalize when it's not start.";
		return nullptr;
	}
	return &m_pLocalModel->Skin();
}

//void BasicTool::InterpolateOnLineSegment(Geometrics::MetaBallModel& Target , DirectX::CXMVECTOR St , DirectX::CXMVECTOR Ed , float Rs , float Re , unsigned int BindIndex )
//{
//	float dis = Vector3::Distance(St,Ed);
//	//0.1 is some kind of transform factor
//	float disthre = (Rs + Re)*0.5f*Metaball_Spare_Factor;
//	int m = (int)(dis/disthre);
//	XMVECTOR InterpolateUnit = (Ed-St)/(float)(m+1);
//	XMVECTOR InterpolatePoint = St;
//	float RadiusStep = (Re - Rs)/(float)(m+1);
//	float Radius = Rs;
//	for(int i = 1; i <= m+1; i++,Radius+=RadiusStep,InterpolatePoint += InterpolateUnit)
//	{
//		Target.push_back(Metaball((Vector3)InterpolatePoint,Radius,BindIndex));
//	}
//}
