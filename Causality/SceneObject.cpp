#include "pch_bcl.h"
#include "SceneObject.h"
#include "Scene.h"

using namespace Causality;
using namespace DirectX;
//using namespace DirectX::Scene;

SceneObject::~SceneObject()
{
	int *p = nullptr;
}

SceneObject::SceneObject() {
	m_IsEnabled = true;
	m_IsStatic = false;
	m_TransformDirty = false;
}

void SceneObject::AddChild(SceneObject * child)
{
	if (child != nullptr)
	{
		std::lock_guard<std::mutex> guard(Scene->ContentMutex());
		auto oldParent = child->parent();
		append_children_back(child);
		child->OnParentChanged(oldParent);
	}
}

void SceneObject::OnParentChanged(SceneObject* oldParent)
{
}

void SceneObject::Update(time_seconds const & time_delta) {
	UpdateTransformsChildWard();
}

DirectX::XMMATRIX SceneObject::GlobalTransformMatrix() const
{
	return this->m_Transform.GlobalTransform().TransformMatrix();
	//if (parent() != nullptr)
	//{
	//	DirectX::XMMATRIX mat = parent()->GetPosition();
	//	mat *= TransformMatrix();
	//	return mat;
	//}
	//else
	//{
	//	return TransformMatrix();
	//}
}

void XM_CALLCONV SceneObject::Move(FXMVECTOR p)
{
	SetPosition((XMVECTOR)GetPosition() + XMVector3Rotate(p, GetOrientation()));
}

void XM_CALLCONV SceneObject::Rotate(FXMVECTOR q)
{
	SetOrientation(XMQuaternionMultiply(q, GetOrientation()));
}

void SceneObject::SetTransformDirty()
{
	if (!m_TransformDirty)
	{
		m_TransformDirty = true;
		for (auto& child : children())
		{
			child.SetTransformDirty();
		}
	}
}

Vector3 SceneObject::GetPosition() const {
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GblTranslation;
}

Quaternion SceneObject::GetOrientation() const {
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GblRotation;
}

Vector3 SceneObject::GetScale() const {
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GblScaling;
}

void SceneObject::SetPosition(const Vector3 & p)
{
	if (!parent())
	{
		m_Transform.LclTranslation = m_Transform.GblTranslation = p;
		m_TransformDirty = false;
	}
	else
	{
		XMVECTOR refQ = parent()->GetOrientation(); // parent global orientation
		XMVECTOR V = parent()->GetPosition();
		V = p.Load() - V;
		refQ = XMQuaternionConjugate(refQ);
		V = XMVector3Rotate(V, refQ); // V *= Inverse(ref.Orientation)
		m_Transform.LclTranslation = V;
	}
	SetTransformDirty();
}

void SceneObject::SetOrientation(const Quaternion & q)
{
	if (!parent())
	{
		m_Transform.LclRotation = m_Transform.GblRotation = q;
		m_TransformDirty = false;
	}
	else
	{
		XMVECTOR refQ = parent()->GetOrientation(); // parent global orientation
		refQ = XMQuaternionConjugate(refQ);
		m_Transform.LclRotation = XMQuaternionMultiply(q, refQ);
	}
	SetTransformDirty();
}

void SceneObject::SetScale(const Vector3 & s)
{
	if (!parent())
	{
		m_Transform.LclScaling = m_Transform.GblScaling = s;
	}
	else
	{
		XMVECTOR refS = parent()->GetScale();
		m_Transform.LclScaling = s.Load() / refS;
	}
	SetTransformDirty();
}

void SceneObject::SetLocalTransform(const DirectX::IsometricTransform & lcl)
{
	m_Transform.LocalTransform() = lcl;
	SetTransformDirty();
}

const DirectX::IsometricTransform & SceneObject::GetGlobalTransform() const
{
	if (m_TransformDirty)
		UpdateTransformsParentWard();
	return m_Transform.GlobalTransform();
}

void SceneObject::UpdateTransformsParentWard() const
{
	if (!m_TransformDirty) return;
	const SceneObject *pObj = parent();

	if (pObj)
	{
		if (pObj->m_TransformDirty)
			pObj->UpdateTransformsParentWard();
		m_Transform.UpdateGlobalTransform(pObj->m_Transform);
	}
	else
		m_Transform.GlobalTransform() = m_Transform.LocalTransform();
	m_TransformDirty = false;
}

void SceneObject::UpdateTransformsChildWard()
{
	if (m_TransformDirty)
	{
		UpdateTransformsParentWard();
		for (auto& child : children())
		{
			child.UpdateTransformsChildWard();
		}
	}
}