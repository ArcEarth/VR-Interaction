#pragma once
#include "Armature.h"

namespace Causality
{
	namespace BoneFeatures
	{
		struct LclRotLnQuatFeature //: concept BoneFeature 
		{
			static const size_t Dimension = 3;
			static const bool EnableBlcokwiseLocalization = false;

			template <class Derived>
			inline static void Get(_Out_ Eigen::DenseBase<Derived>& fv, _In_ const Bone& bone)
			{
				XM_ALIGN16 Vector3 qs;
				DirectX::XMVECTOR q = bone.LclRotation.LoadA();
				q = DirectX::XMQuaternionLn(q);
				//q *= bone.GblLength;
				qs.StoreA(q);
				fv = Eigen::Vector3f::MapAligned(&qs.x);
			}

			template <class Derived>
			inline static void Set(_Out_ Bone& bone, _In_ const Eigen::DenseBase<Derived>& fv)
			{
				// ensure continious storage
				Eigen::Vector3f cfv = fv;
				DirectX::XMVECTOR q = DirectX::XMLoadFloat3(reinterpret_cast<const DirectX::XMFLOAT3 *>(cfv.data()));
				//q /= bone.GblLength;
				q = DirectX::XMQuaternionExp(q);
				bone.LclRotation.StoreA(q);
			}
		};

		struct LclTRFeature
		{
			static const size_t Dimension = 6;
			static const bool EnableBlcokwiseLocalization = false;
			typedef Eigen::Matrix<float, Dimension, 1> VectorType;

			template <class Derived>
			inline static void Get(_Out_ Eigen::DenseBase<Derived>& fv, _In_ const Bone& bone)
			{
				assert(fv.size() == Dimension);
				using DirectX::operator*=;
				DirectX::XMVECTOR q = bone.LclRotation.LoadA();
				q = DirectX::XMQuaternionLn(q);

				//! IMPORTANT
				q *= bone.GblLength;

				fv.segment<3>(0) = Eigen::Vector3f::MapAligned(q.m128_f32);
				fv.segment<3>(3) = Eigen::Vector3f::MapAligned(&bone.LclTranslation.x);
			}

			inline static VectorType Get(_In_ const Bone& bone)
			{
				VectorType fv;
				Get(fv, bone);
				return fv;
			}

			template <class Derived>
			inline static void Set(_Out_ Bone& bone, _In_ const Eigen::DenseBase<Derived>& fv)
			{
				// ensure continious storage
				VectorType cfv = fv;
				DirectX::XMVECTOR q = DirectX::XMLoadFloat3(reinterpret_cast<const DirectX::XMFLOAT3 *>(cfv.data()));

				//! IMPORTANT
				q /= bone.GblLength;

				q = DirectX::XMQuaternionExp(q);
				bone.LclRotation.StoreA(q);
				bone.LclTranslation = reinterpret_cast<const DirectX::Vector3&>(cfv.data()[3]);
			}
		};

		struct GblPosLclRotFeature
		{
			static const size_t Dimension = 6;
			static const bool EnableBlcokwiseLocalization = false;
			typedef Eigen::Matrix<float, Dimension, 1> VectorType;

			template <class Derived>
			inline static void Get(_Out_ Eigen::DenseBase<Derived>& fv, _In_ const Bone& bone)
			{
				assert(fv.size() == Dimension);
				using DirectX::operator*=;
				DirectX::XMVECTOR q = bone.LclRotation.LoadA();
				q = DirectX::XMQuaternionLn(q);

				//! IMPORTANT
				q *= bone.GblLength;

				fv.segment<3>(0) = Eigen::Vector3f::MapAligned(q.m128_f32);
				fv.segment<3>(3) = Eigen::Vector3f::MapAligned(&bone.EndPostion.x);
			}

			inline static VectorType Get(_In_ const Bone& bone)
			{
				VectorType fv;
				Get(fv, bone);
				return fv;
			}

			template <class Derived>
			inline static void Set(_Out_ Bone& bone, _In_ const Eigen::DenseBase<Derived>& fv)
			{
				// ensure continious storage
				VectorType cfv = fv;
				DirectX::XMVECTOR q = DirectX::XMLoadFloat3(reinterpret_cast<const DirectX::XMFLOAT3 *>(cfv.data()));

				//! IMPORTANT
				q /= bone.GblLength;

				q = DirectX::XMQuaternionExp(q);
				bone.LclRotation.StoreA(q);
				bone.EndPostion = reinterpret_cast<const DirectX::Vector3&>(cfv.data()[3]);
			}
		};

		struct GblPosFeature
		{
			static const size_t Dimension = 3;
			static const bool EnableBlcokwiseLocalization = true;

			template <class Derived>
			inline static void Get(_Out_ Eigen::DenseBase<Derived>& fv, _In_ const Bone& bone)
			{
				fv = Eigen::Vector3f::MapAligned(&bone.EndPostion.x);
			}

			template <class Derived>
			inline static void Set(_Out_ Bone& bone, _In_ const Eigen::DenseBase<Derived>& fv)
			{
				// ensure continious storage
				Eigen::Vector3f::MapAligned(&bone.EndPostion.x) = fv;
			}
		};
	}

	// Orientation data is wayyyyyyyy toooooo noisy
	typedef BoneFeatures::GblPosFeature		  InputFeature;
	typedef BoneFeatures::LclRotLnQuatFeature CharacterFeature;
}