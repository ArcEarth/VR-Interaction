#pragma once
#include "Animations.h"
#include <atomic>
#include <map>

namespace Causality
{
	class ClipInfo;
	class CharacterObject;
	class InputClipInfo;

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

		void UpdateTargetCharacter(const BoneHiracheryFrame& sourceFrame, const BoneHiracheryFrame& lastSourceFrame, double deltaTime_seconds) const;

		void UpdateBindingByInputClip(const InputClipInfo& inputClip);

		const std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>>& PvHandles() const;

		std::atomic_bool		IsReady;
		int						ID;
		BoneHiracheryFrame				PotientialFrame;
		float					CharacterScore;
		DirectX::Vector3		MapRefPos;
		DirectX::Vector3		CMapRefPos;
		mutable DirectX::Vector3 LastPos;
		DirectX::Quaternion		MapRefRot;
		DirectX::Quaternion		CMapRefRot;
		int						CurrentActionIndex;
		Eigen::MatrixXf			XabpvT; // Pca matrix of Xabpv
		Eigen::RowVectorXf		uXabpv; // Pca mean of Xabpv

		Eigen::Array<Eigen::RowVector3f, Eigen::Dynamic, Eigen::Dynamic>	PvDifMean;
		Eigen::Array<Eigen::Matrix3f, Eigen::Dynamic, Eigen::Dynamic>		PvDifCov;


		// Addtional velocity
		DirectX::Vector3					Vaff;
		std::vector<std::pair<DirectX::Vector3, DirectX::Vector3>> m_PvHandles;

		// Principle displacement driver
		ClipInfo& GetAnimationInfo(const std::string& name);

		std::map<std::string, ClipInfo*>& GetClipInfos() { return m_Clipinfos; }

	public:
		CharacterObject*										m_pCharacter;
		std::map<std::string, ClipInfo*>						m_Clipinfos;
		std::unique_ptr<ArmatureTransform>						m_pBinding;
		std::unique_ptr<ArmatureTransform>						m_pSelfBinding;
		//std::unique_ptr<IArmaturePartFeature>					m_pFeatureExtrator;


		void SetSourceArmature(const IArmature& armature);

		void SetTargetCharacter(CharacterObject& object);
	};

}