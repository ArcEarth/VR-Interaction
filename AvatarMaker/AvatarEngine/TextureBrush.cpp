#include "stdafx.h"
#include "TextureBrush.h"
#include "ConstantBuffer.h"
#include "BaseEffect.h"
#include <map>
#include <array>
#include <CommonStates.h>
#include <wrl\client.h>

using namespace Microsoft::WRL;
using namespace EditingTools::PaintTools;
using namespace DirectX;

struct BrushBuffer
{
	static const unsigned int BrushCount = 4;
	Vector4 BrushCenter[BrushCount];
	Vector4 BrushColor[BrushCount];
	XMUINT4 ActiveBrushCount;
};

#ifdef _DEBUG
const static char BrushVertexShaderFile[]				=	"..\\Debug\\VS_RenderToUV.cso";
const static char BrushPixelShaderFile[]				=	"..\\Debug\\PS_VertexColor.cso";
#else
const static char BrushVertexShaderFile[]				=	"Data\\Shaders\\VS_RenderToUV.cso";
const static char BrushPixelShaderFile[]				=	"Data\\Shaders\\PS_VertexColor.cso";
#endif

class GaussBrush::Impl
{
public:
	Impl(ID3D11Device* pDevice , DirectX::SkinMesh* pTarget)
	: m_GPUBuffer(pDevice)
	, m_pEffect(new CustomEffect<SkinVertex>(pDevice,BrushVertexShaderFile,nullptr,BrushPixelShaderFile))
	, m_pTarget(pTarget)
	//, m_States(pDevice)
	{
		D3D11_BLEND_DESC desc;
		ZeroMemory(&desc, sizeof(desc));

		desc.RenderTarget[0].BlendEnable = TRUE;

		desc.RenderTarget[0].SrcBlend  = D3D11_BLEND_SRC_ALPHA;
		desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ZERO;
		desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
		desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
		desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
		desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;

		desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

		HRESULT hr = pDevice->CreateBlendState(&desc, &m_pBlendState);

		if (SUCCEEDED(hr))
			SetDebugObjectName(m_pBlendState.Get(), "DirectXTK:CommonStates");

		for (unsigned int i = 0; i < BrushBuffer::BrushCount ; ++i)
		{
			DisableBrush(i);
		}
	}

	~Impl();

	void SetBrushData(unsigned int Index,XMVECTOR Position,XMVECTOR Color,float Radius)
	{

		auto N = m_Slots.size();
		unsigned int slot = -1;
		for (size_t i = 0; i < N; i++)
		{
			if (m_Slots[i] == Index) {
				slot = i;
				break;
			}
		}

		if (slot == -1) return;
		m_DirtyFlags |= 1<<slot;

		m_BrushBuffer.BrushCenter[slot] = Position;
		m_BrushBuffer.BrushCenter[slot].w = Radius;
		m_BrushBuffer.BrushColor[slot] = Color;
	}

	void EnableBrush(unsigned int Index)
	{
		//m_DirtyFlags |= 1<<Index;
		auto itr = m_Slots.begin();
		for (; itr != m_Slots.end(); itr++)
		{
			if (*itr == Index) break;
		}
		if (itr != m_Slots.end()) return;
		m_Slots.push_back(Index);

		m_BrushBuffer.BrushCenter[Index] = g_XMZero;
		m_BrushBuffer.BrushColor[Index] = Colors::Transparent;
	}

	void DisableBrush(unsigned int Index)
	{
		auto itr = m_Slots.begin();
		for (; itr != m_Slots.end(); itr++)
		{
			if (*itr == Index) break;
		}
		if (itr ==  m_Slots.end()) return;
		m_Slots.erase(itr);
		
		m_DirtyFlags &= ~(1<<Index);
		m_BrushBuffer.BrushCenter[Index] = g_XMZero;
		m_BrushBuffer.BrushColor[Index] = Colors::Transparent;
	}

	void Draw(ID3D11DeviceContext *pContext , DirectX::RenderTargetTexture2D* pCanvas)
	{

		assert(pCanvas);
		assert(m_pTarget);

		if (!m_DirtyFlags) return;
		auto N = m_Slots.size();
		if (N==0) return;
		bool IfDraw = true;
		for (size_t i = 0; i < N; i++)
		{
			IfDraw = IfDraw && (m_DirtyFlags & (1<<i));
		}
		if (!IfDraw) return;
		m_DirtyFlags = 0;


		pContext->OMSetBlendState(m_pBlendState.Get(),nullptr,0xFFFFFFFF);
		//pContext->RSSetState(m_States.Wireframe());
		pCanvas->SetAsRenderTarget(pContext,nullptr);

		m_BrushBuffer.ActiveBrushCount.x = N;
		m_GPUBuffer.SetData(pContext,m_BrushBuffer);
		pContext->VSSetConstantBuffers(1,1,m_GPUBuffer.GetBufferAddress());

		m_pEffect->Apply(pContext);
		m_pTarget->SkinMesh::Render(pContext);
	}

	void SetTarget(DirectX::TextureSkinMesh* pTarget)
	{
		m_pTarget = pTarget;
	}

public:
	static std::map<ID3D11Device*,std::weak_ptr<Impl>> ResourcePool; 

	static std::shared_ptr<Impl> CreateImpl(ID3D11Device* pDevice , DirectX::SkinMesh* m_pTarget)
	{
		if (ResourcePool.find(pDevice) == ResourcePool.end())
		{
			auto pImpl = std::make_shared<Impl>(pDevice,m_pTarget);
			ResourcePool[pDevice] = pImpl;
			return pImpl;
		}
		else
		{
			return ResourcePool[pDevice].lock();
		}
	}


private:
	unsigned int								m_DirtyFlags;
	BrushBuffer									m_BrushBuffer;
	ConstantBuffer<BrushBuffer>					m_GPUBuffer;
	std::unique_ptr<IEffect>					m_pEffect;
	std::vector<uint32_t>						m_Slots;
	ComPtr<ID3D11BlendState>					m_pBlendState;

	DirectX::SkinMesh*							m_pTarget;
};

std::map<ID3D11Device*,std::weak_ptr<GaussBrush::Impl>> GaussBrush::Impl::ResourcePool; 

GaussBrush::Impl::~Impl()
{
}

GaussBrush::GaussBrush(ID3D11Device* pDevice , DirectX::SkinMesh* pTarget , unsigned int _Index , float _Radius)
	: m_pImpl(Impl::CreateImpl(pDevice,pTarget))
	, Index(_Index)
{
	this->_radius = _Radius;
}

void GaussBrush::Finish(){
	m_pImpl->DisableBrush(Index);
}

void GaussBrush::Start()
{
	m_pImpl->EnableBrush(Index);
}

GaussBrush::~GaussBrush()
{}



bool GaussBrush::Draw(ID3D11DeviceContext* pContext , DirectX::RenderTargetTexture2D* pCanvas , DirectX::XMVECTOR PositionWS, DirectX::FXMVECTOR Color)
{
	m_pImpl->SetBrushData(Index,PositionWS,Color,Radius);
	m_pImpl->Draw(pContext,pCanvas);
	return true;
}

bool PaintBucket::Draw(ID3D11DeviceContext* pContext , DirectX::RenderTargetTexture2D* pCanvas , DirectX::XMVECTOR PositionWS, DirectX::FXMVECTOR Color)
{
	pCanvas->Clear(pContext,Color);
	return true;
}


