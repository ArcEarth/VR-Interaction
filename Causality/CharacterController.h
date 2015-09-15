#pragma once
#include "Animations.h"
#include <atomic>
#include <map>

namespace Causality
{
	class ClipInfo;
	class CharacterObject;

	class CharacterController
	{
	public:
		~CharacterController();
		void Initialize(const IArmature& player, CharacterObject& character);

		const ArmatureTransform& Binding() const;
		ArmatureTransform& Binding();
		void SetBinding(ArmatureTransform* pBinding);

		const CharacterObject& Character() const;
		CharacterObject& Character();

		void UpdateTargetCharacter(const AffineFrame& sourceFrame, const AffineFrame& lastSourceFrame, double deltaTime_seconds) const;

		std::atomic_bool		IsReady;
		int						ID;
		AffineFrame				PotientialFrame;
		float					CharacterScore;
		DirectX::Vector3					MapRefPos;
		DirectX::Vector3					CMapRefPos;
		mutable DirectX::Vector3			LastPos;
		DirectX::Quaternion				MapRefRot;
		DirectX::Quaternion				CMapRefRot;
		int						CurrentActionIndex;

		// Addtional velocity
		DirectX::Vector3					Vaff;

		// Principle displacement driver
		ClipInfo& GetAnimationInfo(const std::string& name);

		std::map<std::string, ClipInfo*>& GetClipInfos() { return m_Clipinfos; }

	public:
		CharacterObject*										m_pCharacter;
		std::map<std::string, ClipInfo*>						m_Clipinfos;
		std::unique_ptr<ArmatureTransform>						m_pBinding;
		std::unique_ptr<ArmatureTransform>						m_pSelfBinding;
		//std::unique_ptr<BlockFeatureExtractor>					m_pFeatureExtrator;


		void SetSourceArmature(const IArmature& armature);

		void SetTargetCharacter(CharacterObject& object);
	};

}