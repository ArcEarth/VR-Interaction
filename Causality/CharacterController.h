#pragma once
#include "Animations.h"
#include <atomic>
#include "ClipMetric.h"
#include "StylizedIK.h"

namespace tinyxml2
{
	class XMLElement;
}

namespace Causality
{
	class ClipInfo;
	class CharacterObject;
	class InputClipInfo;
	class CharacterClipinfo;
	class ClipFacade;

	class CharacterController
	{
	public:
		~CharacterController();
		void Initialize(const IArmature& player, CharacterObject& character);

		const ArmatureTransform& Binding() const;
		ArmatureTransform& Binding();
		void SetBinding(std::unique_ptr<ArmatureTransform> &&upBinding);

		const ArmatureTransform& SelfBinding() const { return *m_pSelfBinding; }
		ArmatureTransform& SelfBinding() { return *m_pSelfBinding; }

		const CharacterObject& Character() const;
		CharacterObject& Character();

		void UpdateTargetCharacter(const BoneHiracheryFrame& sourceFrame, const BoneHiracheryFrame& lastSourceFrame, double deltaTime_seconds) const;

		float CreateControlBinding(const ClipFacade& inputClip);

		const std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>>& PvHandles() const;

		std::atomic_bool			IsReady;
		int							ID;
		BoneHiracheryFrame			PotientialFrame;

		float						CharacterScore;
		DirectX::Vector3			MapRefPos;
		DirectX::Vector3			CMapRefPos;
		mutable DirectX::Vector3	LastPos;
		DirectX::Quaternion			MapRefRot;
		DirectX::Quaternion			CMapRefRot;
		int							CurrentActionIndex;

		Eigen::MatrixXf				XabpvT; // Pca matrix of Xabpv
		Eigen::RowVectorXf			uXabpv; // Pca mean of Xabpv


		// Addtional velocity
		DirectX::Vector3					Vaff;
		std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>> m_PvHandles;

		// Principle displacement driver
		CharacterClipinfo& GetClipInfo(const std::string& name);

		std::vector<CharacterClipinfo>& GetClipInfos() { return m_Clipinfos; }

		StylizedChainIK& GetStylizedIK(int pid) { return m_SIKs[pid]; }
		const StylizedChainIK& GetStylizedIK(int pid) const { return m_SIKs[pid]; }

		CharacterClipinfo& GetUnitedClipinfo() { return m_cpxClipinfo; }
		const CharacterClipinfo& GetUnitedClipinfo() const { return m_cpxClipinfo; }
	protected:
		CharacterObject*										m_pCharacter;
		std::vector<CharacterClipinfo>							m_Clipinfos;

		// A Clipinfo which encapture all frames from clips
		CharacterClipinfo										m_cpxClipinfo;
		std::vector<StylizedChainIK>							m_SIKs;

		std::unique_ptr<ArmatureTransform>						m_pBinding;
		std::unique_ptr<ArmatureTransform>						m_pSelfBinding;

	protected:
		void SetSourceArmature(const IArmature& armature);
		void SetTargetCharacter(CharacterObject& object);

		Eigen::MatrixXf GenerateXapv(const std::vector<int> &activeParts);

		void InitializeAcvtivePart(ArmaturePart & part, tinyxml2::XMLElement * settings);
		void InitializeSubacvtivePart(ArmaturePart & part, const Eigen::MatrixXf& Xabpv, tinyxml2::XMLElement * settings);
	};

}