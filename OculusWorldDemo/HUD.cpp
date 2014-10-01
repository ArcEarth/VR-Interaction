#include "HUD.h"
#include "../CommonSrc/Render/Render_D3D11_Device.h"
#include <d3d11_2.h>
#include <PrimitiveBatch.h>

using namespace DirectX;

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

	pFill->SetTexture(0, pTexture);
	pDevice->RenderWithAlpha(pFill, pVertexBuffer, NULL, Matrix4f(), 0, 6, Prim_Triangles);

	// Draw the touch view
	void DisplayViews::touchView(TactonicTouchFrame *touchFrame){
		float delta_theta = 0.4;
		float pi = 3.14;
		int i;
		float angle;
		TactonicTouch t;
		for (i = 0; i < touchFrame->numTouches; i++) {
			t = touchFrame->touches[i];
			glBegin(GL_POLYGON);
			setColor(t.force / (4 * RED_VALUE));
			for (angle = 0; angle < 2 * pi; angle += delta_theta)
				glVertex2d((t.x + 0.5f)*colSpacingPix + colSpacingPix*cos(angle) / 2.0 + xOffsetPix, (t.y + 0.5f)*colSpacingPix + colSpacingPix*sin(angle) / 2.0 + yOffsetPix);

			glEnd();
		}
	}
}

void HUD::Initialize(OVR::Render::RenderDevice *pRender, ovrSizei size)
{
	using namespace OVR;
	using namespace OVR::Render;

	Color c = Color(255, 255, 255, 255);

	pDevice = pRender;


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
	pDevice->SetRenderTarget(pTexture);
	auto pDxTexture = static_cast<OVR::Render::D3D11::Texture*>(pTexture.GetPtr());
	
	float c [] = { 1.0f, .5f, 0, 0.2f};
	ID3D11Device *pDevice;
	ID3D11DeviceContext *pContext;
	pDxTexture->TexRtv->GetDevice(&pDevice);
	pDevice->GetImmediateContext(&pContext);
	pContext->ClearRenderTargetView(pDxTexture->TexRtv, c);
}

void HUD::End()
{
	pDevice->SetDefaultRenderTarget();
}
