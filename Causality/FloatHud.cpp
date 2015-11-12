#include "pch_bcl.h"
#include "FloatHud.h"

using namespace Causality;
using namespace DirectX;

FloatingHud::FloatingHud(HUDElement * hudElem)
{

}

void FloatingHud::Update(time_seconds const & time_delta)
{
	SceneObject::Update(time_delta);
}

bool FloatingHud::IsVisible(const BoundingGeometry & viewFrustum) const
{
	auto pVisual = FirstAncesterOfType<IVisual>();
	return pVisual && pVisual->IsVisible(viewFrustum);
}

RenderFlags Causality::FloatingHud::GetRenderFlags() const
{
	return RenderFlags::SpecialEffects;
}

void FloatingHud::Render(IRenderContext * pContext, IEffect * pEffect)
{
}

void XM_CALLCONV FloatingHud::UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection)
{
}
