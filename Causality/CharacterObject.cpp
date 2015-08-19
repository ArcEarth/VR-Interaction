#include "pch_bcl.h"
#include "CharacterObject.h"
#include <PrimitiveVisualizer.h>

using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;

const CharacterObject::frame_type & Causality::CharacterObject::GetCurrentFrame() const
{
	return m_CurrentFrame;
}

CharacterObject::frame_type & CharacterObject::MapCurrentFrameForUpdate()
{
	m_DirtyFlag = true;
	return m_CurrentFrame;
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

const ArmatureFrameAnimation * Causality::CharacterObject::CurrentAction() const { return m_pCurrentAction; }

string Causality::CharacterObject::CurrentActionName() const { return m_pCurrentAction ? m_pCurrentAction->Name : ""; }

bool Causality::CharacterObject::StartAction(const string & key, time_seconds begin_time, bool loop, time_seconds transition_time)
{
	auto& anim = (*m_pBehavier)[key];
	if (&anim == nullptr) return false;
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
		m_DirtyFlag = true;
	}

	if (m_pSkinModel && m_DirtyFlag)
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
	if (g_ShowCharacterMesh)
		VisualObject::Render(pContext, pEffect);

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

		ID3D11DepthStencilState *pDSS = NULL;
		UINT StencilRef;
		pContext->OMGetDepthStencilState(&pDSS, &StencilRef);
		pContext->OMSetDepthStencilState(g_PrimitiveDrawer.GetStates()->DepthNone(), StencilRef);
		DrawArmature(this->Armature(), frame, color, trans, g_DebugArmatureThinkness / this->GetGlobalTransform().Scale.x);
		pContext->OMSetDepthStencilState(pDSS, StencilRef);

		//color = Colors::LimeGreen.v;
		//DrawArmature(this->Armature(), this->Armature().default_frame(), color, trans);
	}
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

void Causality::DrawArmature(const IArmature & armature, const AffineFrame & frame, const Color & color, const Matrix4x4 & world, float thinkness)
{
	using DirectX::Visualizers::g_PrimitiveDrawer;

	// Invaliad frame
	if (frame.size() < armature.size()) 
		return;

	g_PrimitiveDrawer.SetWorld(world);
	//g_PrimitiveDrawer.Begin();
	for (auto& joint : armature.joints())
	{
		auto& bone = frame[joint.ID()];
		XMVECTOR ep = bone.GblTranslation;

		if (!joint.is_root())
		{
			auto& pbone = frame[joint.parent()->ID()];
			XMVECTOR sp = pbone.GblTranslation;

			//g_PrimitiveDrawer.DrawLine(sp, ep, color);
			g_PrimitiveDrawer.DrawCylinder(sp, ep, thinkness, color);
		}
		g_PrimitiveDrawer.DrawSphere(ep, thinkness * 1.5f, color);
	}
	//g_PrimitiveDrawer.End();
}
