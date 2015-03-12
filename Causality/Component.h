#pragma once
#include "Common\BCL.h"
#include "SceneObject.h"

namespace Causality
{
	class IRenderContext;

	class IRenderable abstract
	{
	public:
		virtual void Render(IRenderContext *pContext) = 0;
	};

	// Interface for setting View/Projection Matrix
	// Represent a 3D object that depend on Camera view and 3D location
	class IViewable abstract
	{
	public:
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) = 0;
		//virtual void XM_CALLCONV UpdateProjectionMatrix(DirectX::FXMMATRIX projection) = 0;
	};

	// Interface for Time-dependent Animation
	class ITimeAnimatable abstract
	{
	public:
		virtual void UpdateAnimation(time_seconds const& time_delta) = 0;
	};

	class SceneObject;

	class Component : public Object
	{
	public:
		Component(SceneObject& owner)
			: Owner(owner)
		{}

		SceneObject&	Owner;
	};

	class Behavier : public Component
	{

	};
}