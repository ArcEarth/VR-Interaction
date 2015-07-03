#include "pch_directX.h"

#include <ShadowMapGenerationEffect.h>
#include <..\Src\EffectCommon.h>

using namespace DirectX;

struct ShadowMapGenerationEffectConstants
{
	XMMATRIX	WorldLightProj;
	XMFLOAT3X4	Bones[ShadowMapGenerationEffect::MaxBones];
};

struct ShadowMapGenerationEffectTraits
{
	typedef ShadowMapGenerationEffectConstants ConstantBufferType;

	static const int VertexShaderCount = 4;
	static const int PixelShaderCount = 1;
	static const int ShaderPermutationCount = 4;
};

typedef ShadowMapGenerationEffectTraits			EffectTraitsType;
typedef EffectBase<EffectTraitsType>			EffectBaseType;

SharedResourcePool<ID3D11Device*, EffectBaseType::DeviceResources> EffectBaseType::deviceResourcesPool;

class ShadowMapGenerationEffect::Impl : public EffectBaseType
{
public:
	int weightsPerVertex;
	Matrix4x4 World, LightView, LightProj;
	ID3D11DepthStencilView* pDepthMap;

	typedef EffectTraitsType	Traits;
	typedef EffectBaseType		Base;

	Impl(ID3D11Device* device)
			: EffectBase(device),
			weightsPerVertex(0)
		{
			static_assert(_countof(Base::VertexShaderIndices) == Traits::ShaderPermutationCount, "array/max mismatch");
			static_assert(_countof(Base::VertexShaderBytecode) == Traits::VertexShaderCount, "array/max mismatch");
			static_assert(_countof(Base::PixelShaderBytecode) == Traits::PixelShaderCount, "array/max mismatch");
			static_assert(_countof(Base::PixelShaderIndices) == Traits::ShaderPermutationCount, "array/max mismatch");

			XMMATRIX id = XMMatrixIdentity();

			for (size_t i = 0; i < MaxBones; ++i)
			{
				XMStoreFloat3x4(&constants.Bones[i], id);
			}
		}

	int GetCurrentShaderPermutation() const
	{
		// 0 1 2 4
		if (weightsPerVertex <= 2) return weightsPerVertex;
		if (weightsPerVertex == 4) return 3;
		return -1;
	}

	void Apply(ID3D11DeviceContext* deviceContext)
	{
		// Compute derived parameter values.
		matrices.SetConstants(dirtyFlags, constants.WorldLightProj);

		deviceContext->OMSetRenderTargets(1, NULL, pDepthMap);

		ApplyShaders(deviceContext, GetCurrentShaderPermutation());
	}
};


namespace
{
#if defined(_XBOX_ONE) && defined(_TITLE)
#include "Shaders/Xbox/ShadowMapGen_VS_NoBone.inc"
#include "Shaders/Xbox/ShadowMapGen_VS_OneBone.inc"
#include "Shaders/Xbox/ShadowMapGen_VS_TwoBone.inc"
#include "Shaders/Xbox/ShadowMapGen_VS_FourBone.inc"

#include "Shaders/Windows/ShadowMapGen_PS.inc"
#else
#include "Shaders/Windows/ShadowMapGen_VS_NoBone.inc"
#include "Shaders/Windows/ShadowMapGen_VS_OneBone.inc"
#include "Shaders/Windows/ShadowMapGen_VS_TwoBone.inc"
#include "Shaders/Windows/ShadowMapGen_VS_FourBone.inc"

#include "Shaders/Windows/ShadowMapGen_PS.inc"
#endif
}

template <size_t Size>
inline ShaderBytecode MakeShaderByteCode(const BYTE(&bytecode)[Size])
{
	return ShaderBytecode{ bytecode ,sizeof(bytecode) };
}

const ShaderBytecode EffectBase<ShadowMapGenerationEffectTraits>::VertexShaderBytecode[] =
{
	MakeShaderByteCode(ShadowMapGen_VS_NoBone),
	MakeShaderByteCode(ShadowMapGen_VS_OneBone),
	MakeShaderByteCode(ShadowMapGen_VS_TwoBone),
	MakeShaderByteCode(ShadowMapGen_VS_FourBone),
};


const int EffectBase<ShadowMapGenerationEffectTraits>::VertexShaderIndices[] =
{
	0,      // vertex lighting, one bone
	1,
	2,
	3,
};


const ShaderBytecode EffectBase<ShadowMapGenerationEffectTraits>::PixelShaderBytecode[] =
{
	MakeShaderByteCode(ShadowMapGen_PS),
};


const int EffectBase<ShadowMapGenerationEffectTraits>::PixelShaderIndices[] =
{
	0,      // vertex lighting, one bone
	0,      // vertex lighting, one bone
	0,      // vertex lighting, one bone
	0,      // vertex lighting, one bone
};

ShadowMapGenerationEffect::ShadowMapGenerationEffect(ID3D11Device * device)
	: pImpl(new Impl(device))
{
}

ShadowMapGenerationEffect::~ShadowMapGenerationEffect()
{
}

void ShadowMapGenerationEffect::SetShadowMap(ID3D11DepthStencilView * pShaodwMap)
{
	pImpl->pDepthMap = pShaodwMap;
}

void XM_CALLCONV ShadowMapGenerationEffect::SetWorld(FXMMATRIX value)
{
	pImpl->matrices.world = value;

	pImpl->dirtyFlags |= EffectDirtyFlags::WorldViewProj | EffectDirtyFlags::WorldInverseTranspose | EffectDirtyFlags::FogVector;
}


void XM_CALLCONV ShadowMapGenerationEffect::SetView(FXMMATRIX value)
{
	pImpl->matrices.view = value;

	pImpl->dirtyFlags |= EffectDirtyFlags::WorldViewProj | EffectDirtyFlags::EyePosition | EffectDirtyFlags::FogVector;
}


void XM_CALLCONV ShadowMapGenerationEffect::SetProjection(FXMMATRIX value)
{
	pImpl->matrices.projection = value;

	pImpl->dirtyFlags |= EffectDirtyFlags::WorldViewProj;
}

void ShadowMapGenerationEffect::SetWeightsPerVertex(int value)
{
	if ((value != 0) &&
		(value != 1) &&
		(value != 2) &&
		(value != 4))
	{
		throw std::out_of_range("WeightsPerVertex must be 0, 1, 2, or 4");
	}

	pImpl->weightsPerVertex = value;
}


void ShadowMapGenerationEffect::SetBoneTransforms(_In_reads_(count) XMMATRIX const* value, size_t count)
{
	if (count > MaxBones)
		throw std::out_of_range("count parameter out of range");

	auto& boneConstant = pImpl->constants.Bones;

	for (size_t i = 0; i < count; i++)
	{
		XMMATRIX boneMatrix = XMMatrixTranspose(value[i]);
		XMStoreFloat3x4(&boneConstant[i], boneMatrix);
	}

	pImpl->dirtyFlags |= EffectDirtyFlags::ConstantBuffer;
}


void ShadowMapGenerationEffect::ResetBoneTransforms()
{
	auto boneConstant = pImpl->constants.Bones;

	XMMATRIX id = XMMatrixIdentity();

	for (size_t i = 0; i < MaxBones; ++i)
	{
		XMStoreFloat3x4(&boneConstant[i], id);
	}

	pImpl->dirtyFlags |= EffectDirtyFlags::ConstantBuffer;
}



void ShadowMapGenerationEffect::Apply(ID3D11DeviceContext * deviceContext)
{
	pImpl->Apply(deviceContext);
	//ApplyShaders(deviceContext, m_pImpl->GetCurrentShaderPermutation());
}

void ShadowMapGenerationEffect::GetVertexShaderBytecode(void const ** pShaderByteCode, size_t * pByteCodeLength)
{
	auto perm = pImpl->GetCurrentShaderPermutation();
	pImpl->GetVertexShaderBytecode(perm, pShaderByteCode, pByteCodeLength);
}
