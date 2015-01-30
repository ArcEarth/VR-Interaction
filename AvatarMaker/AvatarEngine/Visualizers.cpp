#include "stdafx.h"
#include "Visualizers.h"
#include "GeometricPrimitive.h"
#include <iostream>
#include <PrimitiveBatch.h>
#include <VertexTypes.h>
#include "DebugVisualizer.h"

static const float Radius_Effect_Factor = 2.0f;

std::function<void()> SkeletonViewer::CustomStateFunc = nullptr;

using namespace DirectX;
using namespace std;
using namespace Kinematics;
using namespace boost;

XMGLOBALCONST XMVECTORI32 SelectW = {XM_SELECT_0,XM_SELECT_0,XM_SELECT_0,XM_SELECT_1};

class SkeletonViewer::Viewer
{
public:
	Viewer(ID3D11DeviceContext* pContext ,const ICamera* pCamera)
	{
		m_pCamera = pCamera;
		m_pCylinder = GeometricPrimitive::CreateCylinder(pContext,1.0f,1.0f,16U);
		m_pSphere = GeometricPrimitive::CreateSphere(pContext,1.0f,8U);
		m_pCube = GeometricPrimitive::CreateCube(pContext);

	}

	void DrawSkeleton(const Kinematics::IndexedSkeleton *pSkeleton , State state , FXMVECTOR Colorindicator = Colors::White , const XMFLOAT4A* ColorSchedule=nullptr , _In_ std::function<void()> setCustomState = nullptr)
	{
		for (const auto& idx : pSkeleton->Index)
		{
			const auto pJoint = idx.second;
			XMVECTOR Color = Colorindicator;
			if (ColorSchedule) Color *= XMLoadFloat4A(&ColorSchedule[pJoint->ID]);
			if (XMVectorGetW(Color) <= 0.01f)
				continue;
			float Length = -pJoint->Entity[state].Scale.y;
			XMVECTOR Position = pJoint->Entity[state].Position;
			XMVECTOR Orientation = pJoint->Entity[state].Orientation;
			float Radius = pJoint->Radius * pJoint->Entity[state].Scale.x;
			// * pJoint->Entity[state].Scale.x;
			if (pJoint->IsRoot)
			{
				Length = 0.0f;
				Radius *= 2;
			}
			DrawBone(Length,Radius,Orientation,Position,Color,setCustomState);

		}
	}

	void DrawSpecific(const Kinematics::IndexedSkeleton *pSkeleton , const std::vector<unsigned int> &JointList , State state , FXMVECTOR Colorindicator = Colors::White , const XMFLOAT4A* ColorSchedule=nullptr , _In_ std::function<void()> setCustomState = nullptr)
	{
		for (auto& ID : JointList)
		{
			const auto pJoint = (*pSkeleton)[ID];
			float Length = -pJoint->Entity[state].Scale.y;
			if (pJoint->IsRoot)
				Length = 0.0f;
			XMVECTOR Position = pJoint->Entity[state].Position;
			XMVECTOR Orientation = pJoint->Entity[state].Orientation;
			XMVECTOR Color = Colorindicator;
			if (ColorSchedule) Color *= XMLoadFloat4A(&ColorSchedule[pJoint->ID]);
			float Radius = pJoint->Radius * pJoint->Entity[state].Scale.x;
			DrawBone(Length,Radius,Orientation,Position,Color,setCustomState);
		}
	}

	inline void DrawBone(float Length , float Radius , FXMVECTOR Orientation , FXMVECTOR Position , FXMVECTOR Color , _In_ std::function<void()> setCustomState = nullptr)
	{
		XMMATRIX View = m_pCamera->GetViewMatrix();
		XMMATRIX Projection = m_pCamera->GetProjectionMatrix();
		if (Length!=0.0f)
			m_pCylinder->Draw(
			XMMatrixTranslation(0.0f,0.5f,0.0f) * XMMatrixScaling(Radius,Length,Radius) * XMMatrixRotationQuaternion(Orientation) * XMMatrixTranslationFromVector(Position)
			,View
			,Projection
			,Color,nullptr,false,setCustomState);

		m_pSphere->Draw(XMMatrixScaling(Radius,Radius,Radius) * XMMatrixTranslationFromVector(Position)
			,View
			,Projection
			,Color,nullptr,false,setCustomState);

#ifdef _DEBUG
		XMVECTOR AxisColor = XMVectorSelect(Colors::Red,Color,SelectW);
		m_pCube->Draw(
			XMMatrixTranslation(0.5f,0.0f,0.0f) * XMMatrixScaling(0.1f,0.015f,0.015f) * XMMatrixRotationQuaternion(Orientation) * XMMatrixTranslationFromVector(Position)
			,View
			,Projection
			,AxisColor,nullptr,false,setCustomState);
		AxisColor = XMVectorSelect(Colors::Blue,Color,SelectW);
		m_pCube->Draw(
			XMMatrixTranslation(0.0f,0.0f,0.5f) * XMMatrixScaling(0.015f,0.015f,0.1f) * XMMatrixRotationQuaternion(Orientation) * XMMatrixTranslationFromVector(Position)
			,View
			,Projection
			,AxisColor,nullptr,false,setCustomState);
#endif
	}

protected:
	const ICamera *m_pCamera;
	unique_ptr<GeometricPrimitive> m_pCylinder;
	unique_ptr<GeometricPrimitive> m_pSphere;
	unique_ptr<GeometricPrimitive> m_pCube;
};

SkeletonViewer::SkeletonViewer(ID3D11DeviceContext* pContext ,const ICamera* pCamera ,const Kinematics::IndexedSkeleton* pSkeleton , const DirectX::XMFLOAT4A* _ColorSchedule )
	: m_pViewer(new Viewer(pContext,pCamera))
	, ColorSchedule(_ColorSchedule)
{
	m_pSkeleton = pSkeleton;
	Color = Colors::White;
	SelectFlag.Specify(Current);
}

void SkeletonViewer::Render( ID3D11DeviceContext *pContext )
{
	if (!m_pSkeleton->Root)
		return;
	if (SelectFlag.Contains(Default))
	{
		XMVECTOR ColorModifier = XMVectorSet(0.8f,1.0f,0.8f,1.0f);
		m_pViewer->DrawSkeleton(m_pSkeleton , Default , XMLoadFloat4(&Color) * ColorModifier, ColorSchedule , CustomStateFunc);
	}
	if (SelectFlag.Contains(Current))
	{
		m_pViewer->DrawSkeleton(m_pSkeleton , Current , XMLoadFloat4(&Color) , ColorSchedule , CustomStateFunc);
	}

}

SkeletonViewer::~SkeletonViewer( void )
{
}


class MetaballViewer::Viewer
{
public:
	Viewer(ID3D11DeviceContext* pContext ,const ICamera* pCamera)
		: m_pPrimitveBatch(new PrimitiveBatch<VertexPositionColor>(pContext))
	{
		m_pCamera = pCamera;
		m_pSphere = GeometricPrimitive::CreateGeoSphere(pContext,2.0f);
	}

	void DrawModel(const Geometrics::DynamicMetaBallModel &Model ,FXMVECTOR Color = Colors::White)
	{
		DrawConnections(Model);
		XMMATRIX View = m_pCamera->GetViewMatrix();
		XMMATRIX Projection = m_pCamera->GetProjectionMatrix();
		XMMATRIX ViewProjection = XMMatrixMultiply(View,Projection);
		const auto &volume = Model.Volume();
		const auto &colors = Model.Colors();
		//float RadiusRatio = 1.0f;
		float RadiusRatio = volume.EffictiveRadiusRatio();
		std::vector<size_t> Spheres(volume.size());
		std::vector<float> SpheresZ(volume.size());
		auto& Metaballs = volume.Primitives;

		// Sort the spheres in Z distance in projected space
		std::transform(Metaballs.cbegin(),Metaballs.cend(),SpheresZ.begin(),[&ViewProjection](const Geometrics::Metaball &ball)->float
		{
			XMVECTOR Pos = ball.Position;
			Pos = XMVector3TransformCoord(Pos,ViewProjection);
			return XMVectorGetZ(Pos);
		});

		unsigned int Count = 0;
		for (size_t i = 0;i<Spheres.size();++i)
		{
			Spheres[i] = i;
		}

		std::sort(Spheres.begin(),Spheres.end(),[&SpheresZ](size_t lhs,size_t rhs)->bool
		{
			return(SpheresZ[lhs] > SpheresZ[rhs]);
		});

		//dxout.Begin();
		for (auto index : Spheres)
		{
			const auto& Ball = Metaballs[index];
			XMVECTOR vColor = colors[index];
			//dxout.DrawSphere(Ball.Position,Ball.Radius*RadiusRatio,Color);
			m_pSphere->Draw(XMMatrixScaling(Ball.Radius * RadiusRatio,Ball.Radius * RadiusRatio,Ball.Radius * RadiusRatio) * XMMatrixTranslationFromVector(Ball.Position)
				,View
				,Projection
				,vColor * Color);
		}
		//dxout.End();
	}

	void DrawConnections(const Geometrics::DynamicMetaBallModel &Model)
	{
		dxout.Begin();
		//for (unsigned int i = 0; i < Model.size(); i++)
		//{
		//	for (unsigned int j = i+1; j < Model.size(); j++)
		//	{
		//		//if (Model.IsTwoMetaballIntersect(Model[i],Model[j]))
		//		if (edge(i,j,Model.Connections).second)
		//		{
		//			dxout.DrawLine(Model[i].Position,Model[j].Position,Colors::Red);
		//		}
		//	}
		//}
		const auto &volume = Model.Volume();
		boost::graph_traits<Geometrics::ConnectionGraph>::edge_iterator eitr,eend;
		auto& con =Model.Connections();
		std::tie(eitr,eend)=boost::edges(con);
		for (;eitr != eend ; ++eitr)
		{
			dxout.DrawLine(volume[source(*eitr,con)].Position,volume[target(*eitr,con)].Position,Colors::Red);
		}
		dxout.End();
	}


protected:
	const ICamera *m_pCamera;
	unique_ptr<GeometricPrimitive> m_pCylinder;
	unique_ptr<GeometricPrimitive> m_pSphere;
	unique_ptr<PrimitiveBatch<VertexPositionColor>>	   m_pPrimitveBatch;

};

MetaballViewer::MetaballViewer( ID3D11DeviceContext* pContext ,const ICamera* pCamera )
	: m_pViewer(new Viewer(pContext,pCamera))
{
	Color = DirectX::Colors::White;
	Color.w = 0.2f;
	Target = nullptr;
}

void MetaballViewer::Render( ID3D11DeviceContext *pContext )
{
	if (Target)
		m_pViewer->DrawModel(*Target,Color);
}

MetaballViewer::~MetaballViewer( void )
{

}
