#pragma once
#include <OVR_Kernel.h>
#include "../CommonSrc/Render/Render_Device.h"


class HUD :
	public OVR::NewOverrideBase
{
public:
	HUD();

	void Initialize(OVR::Render::RenderDevice *pRender, ovrSizei size);

	void Begin();

	void End();

	OVR::Render::RenderDevice* GetRenderDevice() const;

	void Render();

	~HUD();

private:
	OVR::Render::RenderDevice			 *pDevice;
	OVR::Ptr<OVR::Render::Buffer>         pVertexBuffer;
	OVR::Ptr<OVR::Render::Fill>			  pFill;
	OVR::Ptr<OVR::Render::Texture>        pTexture;
};

