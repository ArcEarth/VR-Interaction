#pragma once

#include "KinematicsSkeleton.h"
#include "AttachMap.h"
#include <DirectXCollision.h>
#include <memory>

namespace Kinematics
{
	class HumanSkeleton :
		public IndexedSkeleton
	{
	public:
		enum HumanJointEnum
		{
			HIP_CENTER = 0,
			SPINE_CENTER = 1,
			SHOULDER_CENTER = 2,
			HEAD = 3,
			SHOULDER_LEFT = 4,
			ELBOW_LEFT = 5,
			WRIST_LEFT = 6,
			HAND_LEFT = 7,
			SHOULDER_RIGHT = 8,
			ELBOW_RIGHT = 9,
			WRIST_RIGHT = 10,
			HAND_RIGHT = 11,
			HIP_LEFT = 12,
			KNEE_LEFT = 13,
			ANKLE_LEFT = 14,
			FOOT_LEFT = 15,
			HIP_RIGHT = 16,
			KNEE_RIGHT = 17,
			ANKLE_RIGHT = 18,
			FOOT_RIGHT = 19,
		};
		enum HumanBoneEnum
		{
			Illegal = 0,
			Origin = 0,
			LumbarVertebra,
			ThoracicVertebra,
			CervicalVertebra,
			Clavicle_Left,
			Humerus_Left,
			Radius_Left,
			Metacarpals_Left,
			Clavicle_Right,
			Humerus_Right,
			Radius_Right,
			Metacarpals_Right,
			Pelvic_Left,
			Femur_Left,
			Tibia_Left,
			Metatarsals_Left,
			Pelvic_Right,
			Femur_Right,
			Tibia_Right,
			Metatarsals_Right,
		};


		const static int JointCount = 20;
		const static int BoneCount = 20;

		const static DirectX::XMFLOAT4A HumanBoneColorSchedule[JointCount];

	public:
		HumanSkeleton();
		~HumanSkeleton(void);

		//DirectX::Vector3 GetDummyJoint(unsigned int jointIndex);
		DirectX::Vector3 GetActualJoint(unsigned int jointIndex) const;

		std::auto_ptr<DirectX::BoundingBox> GetBoundingBox() const;
		
		/// <summary>
		/// Return the establish of the human heights of this instance.
		/// </summary>
		/// <returns></returns>
		float Height() const;
		// Update the human skeleton with out any joint constraints
		void Update(const DirectX::Vector3 JointPosistions[]);
		void Update(const DirectX::Vector3 JointPosistions[JointCount],const DirectX::Quaternion BoneOrientations[BoneCount]);

	protected:
		void ScaleArm();

	public:
		bool ScaleArmReach;
		float ScaleArmReachScaleFactor;
		float FleshRadius;
		//AttachMap *m_pAttacher;
		std::pair<unsigned int,float> BoneLengthes[20];
		//	DirectX::Vector3 m_JointPositions[JointCount];
	};


}