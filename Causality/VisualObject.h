#pragma once
#include "SceneObject.h"
#include "RenderFlags.h"
#include "RenderSystemDecl.h"

namespace DirectX
{
	class IEffect;

	namespace Scene
	{
		class IModelNode;
		class ISkinningModel;
	}
}

namespace Causality
{
	void XM_CALLCONV DrawGeometryOutline(const BoundingGeometry& geometry, FXMVECTOR color);

	class IVisual abstract
	{
	public:
		// Camera culling
		virtual RenderFlags GetRenderFlags() const = 0;
		virtual bool IsVisible(const BoundingGeometry& viewFrustum) const = 0;
		virtual void Render(IRenderContext *context, IEffect* pEffect = nullptr) = 0;
		virtual void XM_CALLCONV UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection) = 0;
	};

	// Scene object acts like entities for render
	class VisualObject : virtual public SceneObject, virtual public IVisual
	{
	public:
		VisualObject();
		//int										MaxLoD() const;
		//int										CurrentLoD() const;
		//void										SetLoD(int LoD);

		void										Hide() { m_isVisable = false; }
		void										Show() { m_isVisable = true; }

		virtual bool								IsVisible(const BoundingGeometry& viewFrustum) const override;
		bool										IsVisible() const { return m_isVisable; }
		bool										IsFocused() const {
			return m_isFocuesd;
		}

		float										Opticity() const { return m_opticity; }
		void										SetOpticity(float value) { m_opticity = value; }

		IModelNode*									RenderModel(int LoD = 0);
		const IModelNode*							RenderModel(int LoD = 0) const;
		virtual void								SetRenderModel(IModelNode* pMesh, int LoD = 0);

		//Bullet::CollisionShape&						CollisionShape(int LoD = 0);
		//const Bullet::CollisionShape&				CollisionShape(int LoD = 0) const;

		// Inherited via IVisual
		virtual RenderFlags GetRenderFlags() const override;
		virtual void Render(IRenderContext * pContext, IEffect* pEffect = nullptr) override;
		virtual void XM_CALLCONV UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection) override;

	protected:
		float										m_opticity;
		bool										m_isVisable;
		bool										m_isFocuesd;
		IModelNode*									m_pRenderModel;

		//std::shared_ptr<Bullet::CollisionShape>		m_CollisionShape;
	};

	class GlowingBorder : virtual public SceneObject, virtual public IVisual
	{
	public:
		GlowingBorder();

		GlowingBorder(const Color & color);

		// Inherited via IVisual
		virtual bool IsVisible(const BoundingGeometry& viewFrustum) const override;
		virtual RenderFlags GetRenderFlags() const override;
		virtual void Render(IRenderContext * pContext, IEffect* pEffect = nullptr) override;
		virtual void XM_CALLCONV UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection) override;

		XMVECTOR GetColor() const { return m_Color; }
		void XM_CALLCONV SetColor(FXMVECTOR color) { m_Color = color; }

		float Radius() const { return m_Radius; }
		void SetRadius(float value) { m_Radius = value; }
	private:
		float			m_Radius;
		Color			m_Color;
	};

	class CoordinateAxis : virtual public SceneObject, virtual public IVisual
	{
		// Inherited via IVisual
		virtual bool IsVisible(const BoundingGeometry & viewFrustum) const override;
		virtual void Render(IRenderContext * context, IEffect* pEffect = nullptr) override;
		virtual void XM_CALLCONV UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection) override;

		// Inherited via IVisual
		virtual RenderFlags GetRenderFlags() const override;
	};
}