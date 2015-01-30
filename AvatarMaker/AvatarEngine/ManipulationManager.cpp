#include "stdafx.h"
#include "ManipulationManager.h"
#include "DXMathExtend.h"
#include <GeometricPrimitive.h>

using namespace EditingTools;
using namespace DirectX;
using namespace Kinematics;
using namespace std;
using namespace Filters;

/// <summary>
/// The quaternion which will rotate (1,0,0) to V with no-roll-constraint.
/// </summary>
/// <param name="v0">The target direction vector.</param>
/// <returns></returns>
XMVECTOR RotationQuaternionNoRoll(FXMVECTOR V)
{
	XMVECTOR vDir = DirectX::XMVector3Normalize(V);
	XMFLOAT4A Sp;
	DirectX::XMStoreFloat4A(&Sp,vDir);
	float Roll = std::asinf(Sp.y);
	float Yaw = std::atan2f(-Sp.z,Sp.x);
	XMVECTOR qRot = XMQuaternionRotationRollPitchYaw(0.0f,Yaw,Roll);
	XMVECTOR vDir1 = XMVector3Rotate(vDir,XMQuaternionConjugate(qRot));
	XMVECTOR vDir2 = XMVector3Rotate(g_XMIdentityR0,qRot);
	assert(XMVector4NearEqual(vDir-vDir2,g_XMZero,g_XMEpsilon*100));
	assert(XMVector4NearEqual(g_XMIdentityR0-vDir1,g_XMZero,g_XMEpsilon*100));
	return qRot;
}

ManipulationManager::ManipulationManager(void)
	: pTarget(nullptr)
	, pPlayer(nullptr)
	, m_IsWorking(false)
	, GroundHeight(0.0f)
{
	_hand_switch = false;
	OriginHandleVector = g_XMZero;
	ZeroMemory(&OriginData,sizeof(OriginData));
}

ManipulationManager::ManipulationManager(Geometrics::DynamicMetaBallModel *_pTarget ,const HumanSkeleton *_pPlayer ,IFloatingText* pTextBlock)
	: pTarget(_pTarget)
	, pPlayer(_pPlayer)
	, m_IsWorking(false)
	, GroundHeight(0.0f)
{
	_hand_switch = false;
	OriginHandleVector = g_XMZero;
	ZeroMemory(&OriginData,sizeof(OriginData));
	assert(pTarget);
	pTextIndicator = pTextBlock; 
}


ManipulationManager::~ManipulationManager(void)
{
}

void ManipulationManager::Edit(_In_reads_(8) const DirectX::Vector3* EditingPoints)
{
	Vector3 LHand = EditingPoints[0] , RHand = EditingPoints[4];
	Vector3 HandleVector = RHand - LHand;
	Vector3 HandleCenter = (RHand + LHand) / 2.0f;
	if (OriginHandleVector.LengthSq() < 0.01f) 
	{
		OriginHandleVector = HandleVector;
		OriginHandleCenter = HandleCenter;
		OriginData = pTarget->Skeleton->Root->Entity[Current];
		auto Box = pTarget->Volume().GetBoundingBox();
		OriginCenter = Box.Center;
		OriginHeight = Box.Extents.y * 2.0f;
		ForceGroundFlag = ((Box.Center.y - Box.Extents.y) <= 0.2f);
		//OriginCenter = OriginData.Position;
		//cout<<"Start Rotating! Base Vector and rotation"<<endl;
		//cout<<HandleVector<<endl;
		//cout<<Quaternion(InverseInitialRotation)<<endl;
		//cout<<"Frame states : "<<endl;
	} else
	{
		HandleManipulate(HandleVector,HandleCenter);
	}
}


void ManipulationManager::Abort()
{
	if (!m_IsWorking) return;
	pTarget->Skeleton->Root->Entity[Current] = OriginData;
	pTarget->Skeleton->Update();
	pTextIndicator->Content = L"";
	m_IsWorking = false; 
}


void ManipulationManager::Start()
{
	if (m_IsWorking) return;
	assert(pTarget);
	_hand_switch = false;
	OriginHandleVector = g_XMZero;
	ZeroMemory(&OriginData,sizeof(OriginData));
	OriginData = pTarget->Skeleton->Root->Entity[Current];
	// Assume the model always have no local transform(skinning) 
	// not work for DEBUG MODE 
	//OriginCenter = pTarget->Skeleton->Root->BlendPoint(OriginCenter); 
	m_IsWorking = true; 
	PreviousScale = 1.0f;
	Mode = Intial;
	cout<<"Start!"<<endl;
	pTextIndicator->Content = L"";
}


void ManipulationManager::Finish() 
{
	if (!m_IsWorking) return;
	pTextIndicator->Content = L"";
	//GroundTarget(0.0f);
	//pTarget->Scale(OriginData.Scale.x,g_XMZero);
	m_IsWorking = false; 	
}

void ManipulationManager::SetGroundHeight(float groundHeight)
{
	GroundHeight = groundHeight;
}

void ManipulationManager::HandleManipulate(DirectX::Vector3& HandleVector , DirectX::Vector3& HandleCenter)
{
	static const float Threhold_Axis_Cos45 = std::cosf(XM_PI/4);
	static const float Threhold_Scale = 0.2f;
	static const float Threhold_Rotation = XM_PI / 12;
	static const float Threhold_Translation = 0.2f;
	switch (Mode)
	{
	case EditingTools::ManipulationManager::Intial:
		{
			//XMVECTOR vHandle = HandleVector;
			//Quaternion qRot = Quaternion::RotationQuaternion(OriginHandleVector,HandleVector);
			float angle = std::acosf(Vector3::Dot(OriginHandleVector.Normalization(),HandleVector.Normalization()));
			float scale = HandleVector.Length() - OriginHandleVector.Length();
			float displacement = Vector3::Length(HandleCenter - OriginHandleCenter);

			if (abs(angle) > Threhold_Rotation)
			{
				Mode = Modes::Rotation;
				cout<<"Rotation Determined!"<<endl;
				auto Axis = Vector3::Cross(OriginHandleVector,HandleVector);
				Axis = Vector3::Normalize(Axis);
				if (std::abs(Vector3::Dot(Axis,g_XMIdentityR0)) > Threhold_Axis_Cos45)
				{
					SnapedRotationAxis = g_XMIdentityR0;
					cout<<"Snap To X Axis."<<endl;
				} else if (std::abs(Vector3::Dot(Axis,g_XMIdentityR1)) > Threhold_Axis_Cos45)
				{
					SnapedRotationAxis = g_XMIdentityR1;
					cout<<"Snap To Y Axis."<<endl;
				} else if (std::abs(Vector3::Dot(Axis,g_XMIdentityR2)) > Threhold_Axis_Cos45)
				{
					SnapedRotationAxis = g_XMIdentityR2;
					cout<<"Snap To Z Axis."<<endl;
				}else
				{
					Mode = Modes::Intial;
					cout<<"Snap To Axis Failed."<<endl;
				}
				if (Mode == Modes::Rotation)
				{
					OriginHandleVector = HandleVector;
					OriginHandleCenter = HandleCenter;
					ScaleFilter.ClearKey();
					ScaleFilter.InsertKey(0			,XM_PI/12);
					ScaleFilter.InsertKey(XM_PI/2	,XM_PI/12);
					ScaleFilter.InsertKey(-XM_PI/2	,XM_PI/12);
					ScaleFilter.InsertKey(XM_PI		,XM_PI/12);
					//ScaleFilter.InsertKey(XM_PI/4	,XM_PI/12);
					//ScaleFilter.InsertKey(3*XM_PI/4	,XM_PI/12);
					//ScaleFilter.InsertKey(-XM_PI/4	,XM_PI/12);
					//ScaleFilter.InsertKey(-3*XM_PI/4,XM_PI/12);
					ScaleFilter.ResetDynamic();
				}

				break;
			} else if (abs(scale) > Threhold_Scale)
			{
				Mode = Modes::Scale;
				OriginHandleVector = HandleVector;
				OriginHandleCenter = HandleCenter;

				SnapedToPlayerScale = (pPlayer->Height()) / OriginHeight;
				cout<<setprecision(3) <<"Human Height:" << pPlayer->Height() << " Model Height :" << OriginHeight <<" Human Scale : "<<SnapedToPlayerScale<<endl;
				ScaleFilter.ClearKey();
				ScaleFilter.InsertKey(SnapedToPlayerScale,0.3f);
				//ScaleFilter.InsertKey(1.0f,0.2f);
				ScaleFilter.ResetDynamic();

				cout<<"Scale Determined!"<<endl;
			} else if (abs(displacement) > Threhold_Translation)
			{
				Mode = Modes::Translation;
				XMVECTOR vDisplacement = HandleCenter - OriginHandleCenter;
				vDisplacement = XMVector3Normalize(vDisplacement);

				if (std::abs(Vector3::Dot(vDisplacement,g_XMIdentityR0)) > Threhold_Axis_Cos45)
				{
					SnapedTranslationDirection = g_XMIdentityR0;
					cout<<"Snap To X Axis."<<endl;
				} else if (std::abs(Vector3::Dot(vDisplacement,g_XMIdentityR1)) > Threhold_Axis_Cos45)
				{
					SnapedTranslationDirection = g_XMIdentityR1;
					cout<<"Snap To Y Axis."<<endl;
				} else if (std::abs(Vector3::Dot(vDisplacement,g_XMIdentityR2)) > Threhold_Axis_Cos45)
				{
					SnapedTranslationDirection = g_XMIdentityR2;
					cout<<"Snap To Z Axis."<<endl;
				}else
				{
					Mode = Modes::Intial;
					cout<<"Snap To Axis Failed."<<endl;
				}
				
				if (Mode != Modes::Intial)
				{
					OriginHandleVector = HandleVector;
					OriginHandleCenter = HandleCenter;
					cout<<"Translation Determined!"<<endl;
					PreviousCenter = HandleCenter;

					auto pBox = pPlayer->GetBoundingBox();
					float HH = pBox->Center.y + pBox->Extents.y;
					float AH = OriginCenter.y + OriginHeight * 0.5f;
					float ARH = pTarget->Position.y;

					ScaleFilter.ClearKey();
					ScaleFilter.InsertKey(ARH,0.05f);
					ScaleFilter.InsertKey(HH-AH+ARH,0.05f);
					ScaleFilter.ResetDynamic();
				}
			}
		}
		break;
	case EditingTools::ManipulationManager::Rotation:
		{
			XMVECTOR vSnapAxis = SnapedRotationAxis;
			XMVECTOR vHandle = HandleVector;
			XMVECTOR vOriginHandle = OriginHandleVector;
			vHandle = XMVector3Normalize(vHandle);
			vOriginHandle = XMVector3Normalize(vOriginHandle);

			/// Mirror the rotation!
			vHandle = XMVector3Reflect(vHandle,g_XMIdentityR2);
			vOriginHandle = XMVector3Reflect(vOriginHandle,g_XMIdentityR2);
			vSnapAxis = XMVector3Reflect(vSnapAxis,g_XMIdentityR2);

			float angle = acosf(Vector3::Dot(vOriginHandle,vHandle));
			cout<<"angle : "<<setprecision(3)<<angle<<endl;
			if (abs(angle - XM_PI) < 0.003f || abs(angle) < 0.03f)
			{
			} else
			{
				/// Projection the handle vector into rotation plane
				vHandle -= vSnapAxis*XMVector3Dot(vHandle,vSnapAxis);
				vOriginHandle -= vSnapAxis*XMVector3Dot(vHandle,vOriginHandle);
				vHandle = XMVector3Normalize(vHandle);
				vOriginHandle = XMVector3Normalize(vOriginHandle);

				auto vAxis = Vector3::Cross(vOriginHandle,vHandle);
				vAxis = XMVector3Normalize(vAxis);
				angle *= sgn(Vector3::Dot(vAxis,vSnapAxis));
			}
			//angle = ScaleFilter.Apply(angle);
			cout<<"Angle : "<<setprecision(3)<<angle<<endl;
			auto qRot = XMQuaternionRotationAxis(vSnapAxis,angle);
			//Quaternion qRot = Quaternion::RotationQuaternion(OriginHandleVector,vHandle);
			RotateTarget(qRot,OriginCenter);
			UpGroundTarget(GroundHeight);
		}
		break;
	case EditingTools::ManipulationManager::Scale:
		{
			float scale = HandleVector.Length() / OriginHandleVector.Length();
			//cout<<"scale : "<<setprecision(3)<<scale<<" => ";
			scale = ScaleFilter.Apply(scale);

			if (scale == SnapedToPlayerScale)
			{
				pTextIndicator->Content = L"Snap to your height.";
			} else
			{
				pTextIndicator->Content = L"";
			}

			// Limit the scale factor
			scale = min(scale,(pPlayer->Height() + 0.2f) / OriginHeight * 1.5f);
			scale = max(scale,(pPlayer->Height() + 0.2f) / OriginHeight * 0.5f);

			//cout<<setprecision(3)<<scale<<endl;
			ScaleTarget(scale,OriginCenter);
			if (ForceGroundFlag)
				GroundTarget(GroundHeight);
			else
				UpGroundTarget(GroundHeight);
		}
		break;
	case EditingTools::ManipulationManager::Translation:
		{
			const float magnifyFactor = 1.0f;
			XMVECTOR vDisplacement = HandleCenter - PreviousCenter;
			XMVECTOR vDirection = SnapedTranslationDirection;
			vDisplacement = XMVector3Dot(vDirection,vDisplacement) * vDirection;
			PreviousCenter = HandleCenter;
			
			/// Mirror the displacement!
			vDisplacement = XMVector3Reflect(vDisplacement,g_XMIdentityR2);

			TranslateTarget(vDisplacement * magnifyFactor);
			UpGroundTarget(GroundHeight);
		}
		break;
	default:
		break;
	}
}

void ManipulationManager::RotateTarget(DirectX::FXMVECTOR qRotationQuaternion , DirectX::FXMVECTOR vRotationCenter)
{
	// Rotate respect to root position
	pTarget->Orientation = XMQuaternionMultiply(OriginData.Orientation,qRotationQuaternion);

	// Calculate the displacement caused by rotate respect to a specific model center but not the root 
	auto UnRotated = OriginData.Position - vRotationCenter;
	auto Rotated = XMVector3Rotate(UnRotated,qRotationQuaternion);
	Vector3 newPos(OriginData.Position + Rotated - UnRotated);

	pTarget->Position = newPos;
	// FK Rebuild the skeleton and transform matrix
	pTarget->Skeleton->Update();
}

void ManipulationManager::ScaleTarget(float scale , DirectX::FXMVECTOR vScaleCenter)
{
	float scaleFactor = scale / PreviousScale;
	if (std::abs(scaleFactor-1.0f) < 0.01f) return;
	PreviousScale = scale;
	pTarget->Scale(scaleFactor,vScaleCenter);
}

void ManipulationManager::TranslateTarget(FXMVECTOR vDelta)
{
	auto Pos = pTarget->Position;
	Pos = Pos + vDelta;
	//Pos.y = ScaleFilter.Apply(Pos.y);
	pTarget->Position = Pos;
	pTarget->Skeleton->Update();
}



void ManipulationManager::GroundTarget(float groundHeight)
{
	pTarget->AnimationUpdate();
	auto& Model = pTarget->GetAnimatedMetaballModel();
	auto lowestVertex = std::min_element( Model.cbegin() ,  Model.cend(), 
		[](const Geometrics::Metaball& lhs , const Geometrics::Metaball& rhs ) -> bool 
	{
		return lhs.Position.y - lhs.Radius < rhs.Position.y - rhs.Radius;
	});

	auto displacement = groundHeight - (lowestVertex->Position.y - lowestVertex->Radius);
	auto Origin = pTarget->Position;
	Origin.y += displacement;
	pTarget->Position = Origin;
	pTarget->Skeleton->Update();
}

void ManipulationManager::UpGroundTarget(float groundHeight)
{
	pTarget->AnimationUpdate();
	auto& Model = pTarget->GetAnimatedMetaballModel();
	auto lowestVertex = std::min_element( Model.cbegin() ,  Model.cend(), 
		[](const Geometrics::Metaball& lhs , const Geometrics::Metaball& rhs ) -> bool 
	{
		return lhs.Position.y - lhs.Radius < rhs.Position.y - rhs.Radius;
	});

	auto displacement = groundHeight - (lowestVertex->Position.y - lowestVertex->Radius);
	if (displacement <= 0.0f) return;
	auto Origin = pTarget->Position;
	Origin.y += displacement;
	pTarget->Position = Origin;
	pTarget->Skeleton->Update();
}

