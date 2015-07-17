#include "pch_bcl.h"
#include "CharacterObject.h"
#include <PrimitiveVisualizer.h>

using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;

CharacterObject::frame_type & CharacterObject::MapCurrentFrameForUpdate()
{
	return m_CurrentFrame;
	m_DirtyFlag = true;
}

void CharacterObject::ReleaseCurrentFrameFrorUpdate()
{
	m_DirtyFlag = true;
}

IArmature & CharacterObject::Armature() { return m_pBehavier->Armature(); }

const IArmature & CharacterObject::Armature() const { return m_pBehavier->Armature(); }

BehavierSpace & CharacterObject::Behavier() { return *m_pBehavier; }

const BehavierSpace & CharacterObject::Behavier() const { return *m_pBehavier; }

void CharacterObject::SetBehavier(BehavierSpace & behaver) { m_pBehavier = &behaver; }

bool CharacterObject::StartAction(const string & key, time_seconds begin_time, bool loop, time_seconds transition_time)
{
	auto& anim = (*m_pBehavier)[key];
	m_pCurrentAction = &anim;
	m_CurrentActionTime = begin_time;
	m_LoopCurrentAction = loop;
	return true;
}

bool CharacterObject::StopAction(time_seconds transition_time)
{
	m_pCurrentAction = nullptr;
	m_LoopCurrentAction = false;
	return true;
}

void CharacterObject::SetFreeze(bool freeze)
{
}

void CharacterObject::SetRenderModel(DirectX::Scene::IModelNode * pMesh, int LoD)
{
	m_pSkinModel = dynamic_cast<ISkinningModel*>(pMesh);
	if (m_pSkinModel == nullptr && pMesh != nullptr)
	{
		throw std::exception("Render model doesn't support Skinning interface.");
	}

	VisualObject::SetRenderModel(pMesh, LoD);
}

void CharacterObject::Update(time_seconds const & time_delta)
{
	SceneObject::Update(time_delta);
	if (m_pCurrentAction != nullptr)
	{
		m_CurrentActionTime += time_delta;
		m_pCurrentAction->GetFrameAt(m_CurrentFrame, m_CurrentActionTime);
	}

	if (m_pSkinModel)
	{
		auto pBones = reinterpret_cast<XMFLOAT4X4*>(m_pSkinModel->GetBoneTransforms());
		AffineFrame::TransformMatrix(pBones, Armature().default_frame(), m_CurrentFrame, m_pSkinModel->GetBonesCount());
	}
}

RenderFlags Causality::CharacterObject::GetRenderFlags() const
{
	return RenderFlags::OpaqueObjects;
}

bool CharacterObject::IsVisible(const BoundingGeometry & viewFrustum) const
{
	return VisualObject::IsVisible(viewFrustum);
}

void CharacterObject::Render(RenderContext & pContext, DirectX::IEffect* pEffect)
{
	if (g_DebugView)
	{
		using namespace DirectX;
		using Visualizers::g_PrimitiveDrawer;
		const auto& frame = m_CurrentFrame;
		//g_PrimitiveDrawer.Begin();
		//const auto& dframe = Armature().default_frame();
		DirectX::XMVECTOR color = DirectX::Colors::Yellow.v;
		color = DirectX::XMVectorSetW(color, Opticity());

		auto trans = this->GlobalTransformMatrix();
		DrawArmature(this->Armature(), frame, color, trans);
	}

	if (g_ShowCharacterMesh)
		VisualObject::Render(pContext, pEffect);
}

void XM_CALLCONV CharacterObject::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	if (g_DebugView)
	{
		using namespace DirectX;
		using Visualizers::g_PrimitiveDrawer;
		g_PrimitiveDrawer.SetView(view);
		g_PrimitiveDrawer.SetProjection(projection);
	}
	VisualObject::UpdateViewMatrix(view, projection);
}


CharacterObject::CharacterObject()
{
}


CharacterObject::~CharacterObject()
{
}

void Causality::DrawArmature(const IArmature & armature, const AffineFrame & frame, const Color & color, const Matrix4x4 & world)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;

	XMMATRIX transform = world.Load();
	for (auto& joint : armature.joints())
	{
		auto& bone = frame[joint.ID()];
		XMVECTOR ep = bone.GblTranslation;
		ep = XMVector3TransformCoord(ep, transform);

		if (!joint.is_root())
		{
			auto& pbone = frame[joint.parent()->ID()];
			XMVECTOR sp = pbone.GblTranslation;
			sp = XMVector3TransformCoord(sp, transform);

			g_PrimitiveDrawer.DrawCylinder(sp, ep, 0.015f, color);
		}
		g_PrimitiveDrawer.DrawSphere(ep, 0.03f, color);
	}
}
