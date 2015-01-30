#include "stdafx.h"
#include "PlayerSkeletonUpdater.h"
#include "KinectFilter.h"
#include <deque>
#include <iomanip>
typedef Vector4 NUIVECTOR;

using namespace Kinematics;
using namespace DirectX;

class PlayerSkeletonUpdater::Implement
{
public:
	Implement()
		: m_pFilter(new Filters::SkeletonFilter)
	{
	}

	~Implement()
	{
		delete m_pFilter;
	}

	void ApplySmooth(NUI_SKELETON_FRAME* pFrame)
	{
		m_pFilter->Smooth(pFrame);
	}

protected:
	Filters::SkeletonFilter *m_pFilter;
};


PlayerSkeletonUpdater::PlayerSkeletonUpdater(void)
	: m_pImplement(new Implement)
{
	StickyTime = 2000;
	ZeroMemory(&m_GroundingMatrix,sizeof(m_GroundingMatrix));
	m_GroundingMatrix(0,0) = m_GroundingMatrix(1,1) = m_GroundingMatrix(2,2) = m_GroundingMatrix(3,3) = 1.0f;
}

PlayerSkeletonUpdater::~PlayerSkeletonUpdater(void)
{
}

bool PlayerSkeletonUpdater::UpdatePlayerSkeleton(NUI_SKELETON_FRAME* pFrame)
{
	static std::map<DWORD,std::deque<float>> GroundSamples;

	if (!pFrame) return false;

	for (auto player : m_pPlayers)
	{
		if (!player) continue;
		player->m_Tracked = false;
	}

	NUI_SKELETON_FRAME& Frame = *pFrame;
	m_pImplement->ApplySmooth(&Frame);

	XMFLOAT4 Equation = *(XMFLOAT4*)(&pFrame->vFloorClipPlane);
	XMVECTOR vEquation = XMLoadFloat4(&Equation);
	XMMATRIX GroundingMatrix;
	if (!XMVector4Equal(vEquation,g_XMZero))
	{
		XMVECTOR qRot = DirectX::XMQuaternionRotationVectorToVector(vEquation,g_XMIdentityR1);
		float H = (Equation.w / Equation.y);
		XMVECTOR vDis = H * g_XMIdentityR1;
		GroundingMatrix = XMMatrixRotationQuaternion(qRot);
		GroundingMatrix *= XMMatrixTranslationFromVector(vDis);
		XMStoreFloat4x4(&m_GroundingMatrix,GroundingMatrix);
	}else{	
		GroundingMatrix = XMLoadFloat4x4(&m_GroundingMatrix);
	}

	DWORD TrackedIDs[NUI_SKELETON_COUNT];
	unsigned int TrackedIDCount = 0;
	// Grounding the skeletons
	for (auto& skeleton : Frame.SkeletonData)
	{
		if (skeleton.eTrackingState == NUI_SKELETON_NOT_TRACKED) continue;

		TrackedIDs[TrackedIDCount++] = skeleton.dwTrackingID;

		XMVECTOR vPos = XMLoadFloat4((XMFLOAT4*)&skeleton.Position);
		vPos = XMVector3Transform(vPos,GroundingMatrix);
		XMStoreFloat4((XMFLOAT4*)&skeleton.Position,vPos);

		if (skeleton.eTrackingState == NUI_SKELETON_POSITION_ONLY) continue;

		for (auto& position : skeleton.SkeletonPositions)
		{
			XMVECTOR vPos = XMLoadFloat4((XMFLOAT4*)&position);
			vPos = XMVector3Transform(vPos,GroundingMatrix);
			XMStoreFloat4((XMFLOAT4*)&position,vPos);
		}

		auto foot = std::min_element(skeleton.SkeletonPositions,skeleton.SkeletonPositions + 20 ,
			[](const NUIVECTOR& lhs , const NUIVECTOR& rhs) -> bool{
				return lhs.y < rhs.y;
		});

		if (GroundSamples[skeleton.dwTrackingID].size() < 10)
			GroundSamples[skeleton.dwTrackingID].push_back(foot->y);
		else
		{
			GroundSamples[skeleton.dwTrackingID].pop_front();
			GroundSamples[skeleton.dwTrackingID].push_back(foot->y);
		}

		size_t center = GroundSamples[skeleton.dwTrackingID].size() / 2;
		auto Sorted = GroundSamples[skeleton.dwTrackingID];
		std::nth_element(Sorted.begin(),Sorted.begin() + center,Sorted.end());
		float H = Sorted[center];

		skeleton.Position.y -= H;
		for (auto& position : skeleton.SkeletonPositions)
		{
			position.y -= H;
		}
	}

	if (TrackedIDCount == 0) return false;

	for (auto& player : m_pPlayers)
	{
		unsigned int j;
		for (j = 0; j < TrackedIDCount; j++)
		{
			if (TrackedIDs[j] == player->m_TrackingID) {
				TrackedIDs[j] = 0;
				break;
			}
		}
		if (j>=TrackedIDCount)
		{
			player->m_TrackingID = 0;
		}
	}

	if ((m_pPlayers[0]->m_TrackingID == 0)&&(m_pPlayers[1]->m_TrackingID != 0))
	{
		if (m_StickyTimer.GetTime() > StickyTime)
		{
			std::swap(m_pPlayers[0]->m_TrackingID,m_pPlayers[1]->m_TrackingID);
			m_StickyTimer.Reset();
		}
	} else
	{
		m_StickyTimer.Reset();
	}



	for (auto& player : m_pPlayers)
	{
		if (player->m_TrackingID == 0)
		{
			for (unsigned int j = 0; j < TrackedIDCount; j++)
			{
				if (TrackedIDs[j] != 0) {
					player->m_TrackingID = TrackedIDs[j];
					break;
				}
			}
		}
	}

	bool Hr = false;

	for (auto& player : m_pPlayers)
	{
		int SID = 0;
		for (SID=0; SID<NUI_SKELETON_COUNT ;SID++)
		{
			if (Frame.SkeletonData[SID].dwTrackingID == player->m_TrackingID)
				break;
		}

		if (SID >= NUI_SKELETON_COUNT)
		{
			player->m_Tracked = false;
			player->m_TrackingID = 0;
			player->TimeStamps.clear();
			player->BodyCenterTrajectory.clear();
			player->Hands[0].Trajectory.clear();
			player->Hands[1].Trajectory.clear();
			player->m_TrackStartTime = 0;
		} else
		{
			UpdatePlayerSkeleton(player,Frame.SkeletonData[SID],Frame.liTimeStamp.QuadPart);
			++SID;
		}
		Hr |= player->m_Tracked;
	}
	return Hr;
}

void PlayerSkeletonUpdater::UpdatePlayerSkeleton(Player* player , NUI_SKELETON_DATA& skeleton , long long timeStamp)
{
	if (skeleton.eTrackingState == NUI_SKELETON_NOT_TRACKED) return;

	player->m_TrackingID = skeleton.dwTrackingID;
	if (skeleton.eTrackingState == NUI_SKELETON_TRACKED)
	{
		if (player->m_TrackStartTime == 0) 
			player->m_TrackStartTime  = timeStamp;
		if (player->TimeStamps.size() > 0 && timeStamp - player->TimeStamps.front() > 500)
		{
			player->TimeStamps.pop_front();
			player->BodyCenterTrajectory.pop_front();
			player->Hands[0].Trajectory.pop_front();
			player->Hands[1].Trajectory.pop_front();
		}

		player->TimeStamps.emplace_back(timeStamp);
		player->BodyCenterTrajectory.emplace_back(reinterpret_cast<float*>(&skeleton.SkeletonPositions[0]));
		player->Hands[0].Trajectory.emplace_back(reinterpret_cast<float*>(&skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT]));
		player->Hands[1].Trajectory.emplace_back(reinterpret_cast<float*>(&skeleton.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT]));

		UpdatePlayerSkeletonTracked(player,skeleton);

	}else if (skeleton.eTrackingState == NUI_SKELETON_POSITION_ONLY)
	{
		UpdatePlayerSkeletonPositionOnly(player,skeleton);
	} else
	{
		player->m_Tracked = false;
	}
}

void PlayerSkeletonUpdater::UpdatePlayerSkeletonTracked(Player* player , NUI_SKELETON_DATA& skeleton)
{
	//Get the skeleton data buffer
	NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];
	DirectX::Quaternion Orientations[NUI_SKELETON_POSITION_COUNT];
	DirectX::Vector3 JointPositions[NUI_SKELETON_POSITION_COUNT];

	player->m_TrackingID = skeleton.dwTrackingID;
	// Turn the x axis since it's a mirror transform
	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
	{
		skeleton.SkeletonPositions[i].x = -skeleton.SkeletonPositions[i].x;
	}

	HRESULT hr = NuiSkeletonCalculateBoneOrientations(&skeleton, boneOrientations);
	if (FAILED(hr)) return;
	//DirectX::XMQuaternionRotationMatrix

	//for (auto& obj : boneOrientations)
	//{
	//	std::cout << "Relative Rotation Matrix of joint ID : " << std::dec <<(unsigned int)obj.endJoint<<std::endl;
	//	std::cout<<std::setiosflags(std::ios::fixed);
	//	std::cout << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M11  << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M12  << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M13  << std::endl;
	//	std::cout << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M21  << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M22  << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M23  << std::endl;
	//	std::cout << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M31  << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M32  << std::setw(7) << std::setprecision(3) << obj.hierarchicalRotation.rotationMatrix.M33  << std::endl;
	//}

	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
	{
		JointPositions[i] = DirectX::Vector3(*reinterpret_cast<DirectX::XMFLOAT4*>(&skeleton.SkeletonPositions[i]));
		Orientations[i] = *reinterpret_cast<DirectX::Quaternion*>(&boneOrientations[i].hierarchicalRotation.rotationQuaternion);
	}

	player->getSkeleton()->Update(JointPositions,Orientations);
	player->m_Tracked = true;
}

void PlayerSkeletonUpdater::UpdatePlayerSkeletonPositionOnly(Player* player , NUI_SKELETON_DATA& skeleton)
{
	player->getSkeleton()->at(HumanSkeleton::HIP_RIGHT)->Entity[Current].Position = *((XMFLOAT3*)&skeleton.Position);
	static_cast<IndexedSkeleton*>(player->getSkeleton())->Update();
	player->m_TrackingID = skeleton.dwTrackingID;
	player->m_Tracked = false;
	player->TimeStamps.clear();
	player->BodyCenterTrajectory.clear();
	player->Hands[0].Trajectory.clear();
	player->Hands[1].Trajectory.clear();
	player->m_TrackStartTime = 0;
	//if (player->BodyCenterTrajectory.size() >= 30)
	//	player->BodyCenterTrajectory.pop_back();
	//player->BodyCenterTrajectory.emplace_front(reinterpret_cast<float*>(&skeleton.SkeletonPositions[0]));

}


