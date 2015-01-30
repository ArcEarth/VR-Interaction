#pragma once

#include <DirectXMath.h>
#include <vector>
#include <Eigen\Sparse>
#include "MathHelper.h"

namespace Kinematics
{
class IMotionAdaptor
	{
	public:
		//virtual bool UpdateTranformMatrices(_Outref_ std::vector<DirectX::XMFLOAT3X4>& TransformMatricesBuffer) const = 0;
		virtual size_t UpdateTranformMatrices(_Out_ DirectX::XMFLOAT3X4* TransformMatricesBuffer) const = 0;

		virtual size_t TranformMatricesCount() const = 0;

		virtual DirectX::XMMATRIX TransformMatrix(const std::vector<float>& weights) const = 0;

		virtual std::vector<DirectX::SimpleMath::DualQuaternion> TransformBones() const = 0;
		/// <summary>
		/// Alters this adaptor when receptor 's geometry or connection changed.
		/// </summary>
		// Maybe vector<vector<pair<index,float>>> is a better choice?
		virtual void Alter(_Outref_ Eigen::MatrixXf& WeightMatrix) = 0;

		virtual ~IMotionAdaptor() {};
	};

	class IHumanAdaptor
	{
	public:
		virtual void Detach() = 0;

		virtual void Detach(unsigned int SourceJointIndex) = 0;

		virtual bool Attach() = 0;

		virtual bool Attach(unsigned int SourceJointIndex) = 0;

		virtual bool IsAttached(unsigned int SourceJointIndex) const = 0;
	};
}