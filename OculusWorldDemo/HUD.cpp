#include "HUD.h"
#include "../CommonSrc/Render/Render_D3D11_Device.h"
#include <d3d11_2.h>
#include <wrl\client.h>
#include <boost\lexical_cast.hpp>

#include "../CommonSrc/Render/Render_Font.h"

namespace OVR
{
	namespace Render
	{
		extern Font DejaVu;
	}
}
//using namespace DirectX;

HUD::HUD()
{
}


HUD::~HUD()
{
}

void HUD::Render()
{

	using namespace OVR;
	using namespace OVR::Render;

	{
		//std::lock_guard<std::mutex> guard(m_Mutext);
		//for (auto const & p : Touches)
		//{
		//	float r = 10.0f;
		//	float x = (2*p.x - 1.0f) * TextureSize.w;
		//	float y = (2*p.y - 1.0f) * TextureSize.h;
		//	Vector4f v(x - r, y - r, x + r, y + r);
		//	Color c(p.z, 255 - p.z, 0);
		//	pDevice->FillRect(v.x, v.y, v.z, v.w, c);
		//}
		//if (Touches.size() > 0)
		//{
		//	const auto& p = Touches[0];
		//	std::string message = "(" + boost::lexical_cast<std::string>(p.x) + "," + boost::lexical_cast<std::string>(p.y) + ")";
		//	pDevice->RenderText(&OVR::Render::DejaVu, message.c_str(), 0, 0, 40, OVR::Color(255, 255, 255));
		//}
	}
	//pFill->SetTexture(0, pTexture);
	//pDevice->RenderWithAlpha(pFill, pVertexBuffer, NULL, Matrix4f(), 0, 6, Prim_Triangles);
}

void HUD::Initialize(OVR::Render::RenderDevice *pRender, ovrSizei size)
{
	using namespace OVR;
	using namespace OVR::Render;

	Color c = Color(255, 255, 255, 255);

	pDevice = pRender;
	TextureSize = size;


	int format = Texture_RGBA | Texture_RenderTarget | 4;
	pTexture = pDevice->CreateTexture(format, size.w, size.h, NULL);
	pTexture->SetSampleMode(Sample_ClampBorder | Sample_Linear);
	pVertexBuffer = pDevice->CreateBuffer();

	pVertexBuffer->Data(Buffer_Vertex, NULL, 6 * sizeof(Vertex));

	Vertex* vertices = (Vertex*) pVertexBuffer->Map(0, 6 * sizeof(Vertex), Map_Discard);
	if (!vertices)
	{
		return;
	}

	const float left   = -size.w * 0.65f;
	const float right  = size.w * 0.65f;
	const float top    = -size.h *0.5f;
	const float bottom = size.h * 0.5f;

	vertices[0] = Vertex(Vector3f(left, top, 0.0f), c, 0.0f, 1.0f);
	vertices[1] = Vertex(Vector3f(right, top, 0.0f), c, 1.0f, 1.0f);
	vertices[2] = Vertex(Vector3f(left, bottom, 0.0f), c, 0.0f, 0.0f);
	vertices[3] = Vertex(Vector3f(left, bottom, 0.0f), c, 0.0f, 0.0f);
	vertices[4] = Vertex(Vector3f(right, top, 0.0f), c, 1.0f, 1.0f);
	vertices[5] = Vertex(Vector3f(right, bottom, 0.0f), c, 1.0f, 0.0f);

	pVertexBuffer->Unmap(vertices);

	pFill = pDevice->CreateTextureFill(pTexture, true);

}

void HUD::Begin()
{
	using namespace Microsoft::WRL;
	pDevice->SetRenderTarget(pTexture);
	auto pDxTexture = static_cast<OVR::Render::D3D11::Texture*>(pTexture.GetPtr());
	
	float c [] = { 1.0f, .5f, 0, 0.2f};
	ComPtr<ID3D11Device> pDevice;
	ComPtr<ID3D11DeviceContext> pContext;
	pDxTexture->TexRtv->GetDevice(&pDevice);
	pDevice->GetImmediateContext(&pContext);
	pContext->ClearRenderTargetView(pDxTexture->TexRtv, c);
}

void HUD::End()
{
	pDevice->SetDefaultRenderTarget();
}
