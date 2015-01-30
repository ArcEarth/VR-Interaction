#pragma once

#include "HumanSkeleton.h"
#include "ModelInterface.h"
#include "Visualizers.h"
#include "MultiMouseControler.h"
#include <array>
#include <string>
#include <deque>

class PlayerSkeletonUpdater;

static const unsigned int MAX_PLAYER_COUNT = 2;

enum GestureEnum
{
	UNUPDATED	=-1,
	FREE		=0,
	PINCH		=1,
	SLAP		=2,
	FINGERGUN	=3,
	FIST		=4,
};

class Player
{
public:
	struct GestureHolder
	{
		GestureEnum Gesture;
		//GestureEnum PriorGesture;
		std::string DeviceName;
		unsigned int DeviceIndex;
		std::array<GestureEnum,8> Decoder;
		bool Trigger;
		bool Update(const MultiMouseControler& Mice);

		GestureHolder()
		{
			clear();
		}

		void clear()
		{
			memset(this,0,sizeof(GestureHolder));
		}
	};

	// Storage per hand data
	struct PlayerHand
	{
		bool UpdateGestures(const MultiMouseControler& Mice);

		bool IsGestureTrigger(unsigned int Hand)
		{
			return GestureHolders[Hand].Trigger;
		}

		/// <summary>
		/// Read the Gestures of the specified hand. side effect : set the gesture to UNUPDATED.
		/// </summary>
		/// <returns></returns>
		GestureEnum Gesture()
		{
			for (auto& holder : GestureHolders)
			{
				if (holder.Trigger)
				{
					holder.Trigger = false;
					return holder.Gesture;
				} else if (holder.Gesture != GestureEnum::FREE)
				{
					break;
				}
			}
			return UNUPDATED;
		}

		//GestureEnum getGesture(unsigned int Hand) const 
		//{
		//	return GestureHolders[Hand].Gesture;
		//};

	public:
		std::vector<GestureHolder>					GestureHolders;
		std::deque<DirectX::Vector3>				Trajectory;
	};

public:
	Player(ID3D11DeviceContext* pContext ,const ICamera* pCamera  , DirectX::FXMVECTOR IndicatorColor = DirectX::g_XMOne);
	Player();
	~Player(void);

	void InitializeVisualizer(ID3D11DeviceContext* pContext ,const ICamera* pCamera , DirectX::FXMVECTOR IndicatorColor = DirectX::g_XMOne);

	bool IsActive() const { return m_Tracked && m_Enable; }
	bool Tracked() const {return m_Tracked; }

	bool getVisiable() const { return m_Visiable && m_Tracked; }
	void setVisiable(bool visiable)  { m_Visiable = visiable;}

	bool getEnabled() const { return m_Enable; }
	void setEnabled(bool enable)  { m_Enable = enable;}

	bool getArmScaling() const {return m_pSkeleton->ScaleArmReach; }
	void setArmScaling(bool isScaling) { m_pSkeleton->ScaleArmReach = isScaling; }

	float getArmScalingRadius() const {return m_pSkeleton->ScaleArmReachScaleFactor; }
	void setArmScalingRadius(float radius) 
	{
		if (radius == 0.0f) 
			setArmScaling(false);
		else
		{
			m_pSkeleton->ScaleArmReachScaleFactor = radius;
		}
	}

	void swap(Player& rhs)
	{
		std::swap(m_TrackingID,rhs.m_TrackingID);
		std::swap(m_Tracked,rhs.m_Tracked);
		std::swap(BodyCenterTrajectory,rhs.BodyCenterTrajectory);
		std::swap(TimeStamps,rhs.TimeStamps);
		std::swap(Hands[0].Trajectory,rhs.Hands[0].Trajectory);
		std::swap(Hands[1].Trajectory,rhs.Hands[1].Trajectory);
		std::swap(*m_pSkeleton,*rhs.m_pSkeleton);
		//.swap(rhs.m_pSkeleton);
	}


	__declspec(property(get = getArmScaling , put = setArmScaling))
		bool ArmScaling;

	__declspec(property(get = getArmScalingRadius , put = setArmScalingRadius))
		float ArmScalingRadius;

	__declspec(property(get = getVisiable , put = setVisiable))
		bool Visiable;

	__declspec(property(get = getEnabled , put = setEnabled))
		bool Enabled;

	__declspec(property(get = getSkeleton))
		Kinematics::HumanSkeleton* Skeleton;

	Kinematics::HumanSkeleton* getSkeleton() {return m_pSkeleton.get();}
	const Kinematics::HumanSkeleton* getSkeleton() const {return m_pSkeleton.get();}

	unsigned int UpdateGestures(const MultiMouseControler& Mice);

	GestureEnum ReadGesture(unsigned int Hand)
	{
		return Hands[Hand].Gesture();
		//if (!GestureHolders[Hand].Trigger) return UNUPDATED;
		//GestureHolders[Hand].Trigger = false;
		//return GestureHolders[Hand].Gesture;
	}

	unsigned int getPlayerIdentity() const {return m_TrackingID;}

	DirectX::IRenderObject* getVisualizer()
	{ 
		if (m_Visiable && m_Tracked) 
			return m_pVisualizer.get();
		else 
			return nullptr;	
	}

	float Velocity() const;
	float HandVelovity(unsigned int wichHand) const;

private:
	bool m_Enable;
	bool m_Tracked;
	bool m_Visiable;
	unsigned long m_TrackingID;
	long long m_TrackStartTime;
	std::unique_ptr<Kinematics::HumanSkeleton> m_pSkeleton;
	std::unique_ptr<SkeletonViewer> m_pVisualizer;

public:
	std::array<PlayerHand,2>					Hands;
	//std::array<GestureHolder,2>					GestureHolders;
	std::deque<DirectX::Vector3>				BodyCenterTrajectory;
	//std::array<std::deque<DirectX::Vector3>,2>	HandsTrajectory;
	std::deque<long long>						TimeStamps;

	friend PlayerSkeletonUpdater;
};

inline std::istream& operator>>(std::istream& lhs , Player::GestureHolder& rhs)
{
	lhs>>rhs.DeviceName;
	for (unsigned int i = 0; i < rhs.Decoder.size(); i++)
	{
		unsigned int gesture;
		lhs>>gesture;
		rhs.Decoder[i] = (GestureEnum)gesture;
	}
	return lhs;
}
