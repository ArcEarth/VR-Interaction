#include "pch_bcl.h"
#include "Scene.h"
#include "SceneParser.h"
#include "Common\Model.h"
#include <boost\filesystem.hpp>
#include <tinyxml2.h>
#include "Common\SkyDome.h"
#include "PlayerProxy.h"

using namespace tinyxml2;
using namespace Causality;
using namespace DirectX::Scene;
using namespace std;

void ParseCameraAttributes(Camera *pCamera, tinyxml2::XMLElement * node, Causality::Scene & scene, Causality::RenderDevice &device);
void ParseRenderableObjectAttributes(RenderableSceneObject* pObj, tinyxml2::XMLElement * node, Causality::AssetDictionary & assets);
void ParseCreatureAttributes(KinematicSceneObject *pCreature, tinyxml2::XMLElement * node, Causality::AssetDictionary & assets);
void ParseSceneObjectAttributes(SceneObject *pObj, XMLElement* node);
void ParseSceneAssets(AssetDictionary& assets, XMLElement* node);

AssetDictionary::mesh_type & ParseMeshAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::texture_type & ParseTextureAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::audio_clip_type & ParseAudioAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::armature_type & ParseArmatureAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::animation_clip_type ParseAnimationClip(XMLElement * node);

AssetDictionary::behavier_type & ParseBehavierAsset(AssetDictionary & assets, XMLElement * node);

Causality::BehavierSpace & LoadBehavierFbx(const char * attr, Causality::AssetDictionary & assets, tinyxml2::XMLElement * node);

std::unique_ptr<SceneObject> ParseSceneObject(Scene& scene, XMLElement* node, SceneObject* parent);

std::unique_ptr<Scene> Scene::LoadSceneFromXML(const string& xml_file)
{
	uptr<Scene> pScene = make_unique<Scene>();
	pScene->LoadFromXML(xml_file);
	return pScene;
}

void Causality::Scene::LoadFromXML(const string & xml_file)
{
	using namespace DirectX;
	tinyxml2::XMLDocument sceneDoc;
	auto error = sceneDoc.LoadFile(xml_file.c_str());

	assert(error == XMLError::XML_SUCCESS);

	auto nScene = sceneDoc.FirstChildElement("scene");
	auto nAssets = nScene->FirstChildElement("scene.assets");

	ParseSceneAssets(this->Assets(), nAssets);

	auto nContent = nScene->FirstChildElement("scene.content");
	nContent = nContent->FirstChildElement();
	this->content = ParseSceneObject(*this, nContent, nullptr);

	UpdateRenderViewCache();

	is_loaded = true;
}

void Causality::Scene::UpdateRenderViewCache()
{
	if (!camera_dirty) return;
	lock_guard<mutex> guard(content_mutex);
	for (auto& obj : content->nodes())
	{
		auto pCamera = obj.As<Camera>();
		if (pCamera != nullptr)
			cameras.push_back(pCamera);
		auto pRenderable = obj.As<IRenderable>();
		if (pRenderable != nullptr)
			renderables.push_back(pRenderable);
	}
	camera_dirty = false;
}

void ParseSceneAssets(AssetDictionary& assets, XMLElement* node)
{
	node = node->FirstChildElement();
	while (node)
	{
		if (!strcmp(node->Name(), "mesh"))
			ParseMeshAsset(assets, node);
		else if (!strcmp(node->Name(), "texture"))
			ParseTextureAsset(assets, node);
		else if (!strcmp(node->Name(), "audio_clip"))
			ParseAudioAsset(assets, node);
		else if (!strcmp(node->Name(), "armature"))
			ParseArmatureAsset(assets, node);
		else if (!strcmp(node->Name(), "behavier"))
			ParseBehavierAsset(assets, node);
		else if (!strcmp(node->Name(), "animation_clip"))
			ParseAudioAsset(assets, node);
		node = node->NextSiblingElement();
	}
}

AssetDictionary::mesh_type& ParseMeshAsset(AssetDictionary& assets, XMLElement* node)
{
	if (!strcmp(node->Name(), "box"))
	{
	}
	else if (!strcmp(node->Name(), "cylinder"))
	{
	}
	else if (!strcmp(node->Name(), "sphere"))
	{
	}
	else if (!strcmp(node->Name(), "mesh"))
	{
		auto src = node->Attribute("src");
		if (src != nullptr && strlen(src) != 0)
		{
			boost::filesystem::path ref(src);
			if (ref.extension().string() == ".obj")
			{
				//auto task = assets.LoadMeshAsync(src);
				//task.wait();
				//return task.get();
				auto& mesh = assets.LoadMesh(node->Attribute("name"), src);
				return mesh;
			}
			else
			{
				// throw;
			}
		}
	}
	return assets.GetMesh("default");
}

AssetDictionary::texture_type& ParseTextureAsset(AssetDictionary& assets, XMLElement* node)
{
	if (!strcmp(node->Name(), "texture"))
	{
		return assets.LoadTexture(node->Attribute("name"), node->Attribute("src"));
	}
	return assets.GetTexture("default");
}

AssetDictionary::audio_clip_type& ParseAudioAsset(AssetDictionary& assets, XMLElement* node)
{
	return assets.GetAudio("default");
}

AssetDictionary::armature_type& ParseArmatureAsset(AssetDictionary& assets, XMLElement* node)
{
	using armature_type = AssetDictionary::armature_type;
	auto src = node->Attribute("src");
	auto name = node->Attribute("name");
	if (src != nullptr)
	{
		return assets.LoadArmature(name, src);
	}
	else // no file armature define
	{
		node->FirstChildElement("joint");
	}
	return assets.GetAsset<armature_type>("default");
}

AssetDictionary::animation_clip_type& ParseAnimationClip(AssetDictionary& assets, XMLElement* node)
{
	using clip_type = AssetDictionary::animation_clip_type;
	auto src = node->Attribute("src");
	return assets.LoadAnimation(node->Attribute("name"), src);
}

AssetDictionary::behavier_type& ParseBehavierAsset(AssetDictionary& assets, XMLElement* node)
{
	using behavier_type = AssetDictionary::behavier_type;
	if (!strcmp(node->Name(), "behavier"))
	{
		auto attr = node->Attribute("src");
		if (attr)
		{
			return assets.LoadBehavierFbx(node->Attribute("name"), attr);
		}
		//else
		//{
		//	auto& behavier = assets.AddBehavier(node->Attribute("name"), new behavier_type);
		//	attr = node->Attribute("armature");
		//	if (attr[0] == '{')
		//	{
		//		string key(attr + 1, attr + strlen(attr) - 1);
		//		auto& armature = assets.GetAsset<AssetDictionary::armature_type>(key);
		//		behavier.SetArmature(armature);
		//	}

		//	auto clip = node->FirstChildElement("animation_clip");
		//	while (clip)
		//	{
		//		auto& aniClip = ParseAnimationClip(assets, clip);
		//		aniClip.SetArmature(behavier.Armature());
		//		behavier.AddAnimationClip(clip->Attribute("name"), move(aniClip));
		//		clip = node->NextSiblingElement("animation_clip");
		//	}
		//	return behavier;
		//}
	}
	return assets.GetAsset<behavier_type>("default");
}

template <typename T>
void GetAttribute(_In_ XMLElement* node, _In_ const char* attr, _Inout_ T& value)
{
	static_assert(false, "unsupported type T")
}

template <>
void GetAttribute<Vector3>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ Vector3& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr)
	{
		string str(attrval);
		if (str.find_first_of(',') != string::npos)
		{
			stringstream ss(str);
			char ch;
			ss >> value.x >> ch >> value.y >> ch >> value.z;
		}
		else
		{
			value.x = value.y = value.z = (float)atof(attrval); // replicate
		}
	}
}

template <>
void GetAttribute<DirectX::Vector4>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ DirectX::Vector4& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr)
	{
		stringstream ss(attrval);
		char ch;
		ss >> value.x >> ch >> value.y >> ch >> value.z >> ch >> value.w;
	}
}


template <>
// Value format : "#AARRGGBB"
void GetAttribute<DirectX::Color>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ DirectX::Color& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr && attrval[0] == '#')
	{
		char* end;
		auto val = strtoul(attrval + 1, &end, 16);
		value.w = ((float)((val & 0xff000000U) >> 24)) / (float)0xff;
		value.x = ((float)((val & 0x00ff0000U) >> 16)) / (float)0xff;
		value.y = ((float)((val & 0x0000ff00U) >> 8)) / (float)0xff;
		value.z = ((float)((val & 0x000000ffU) >> 0)) / (float)0xff;
	}
}

template <>
void GetAttribute<bool>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ bool& value)
{
	node->QueryBoolAttribute(attr, &value);
}

template <>
void GetAttribute<float>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ float& value)
{
	node->QueryFloatAttribute(attr, &value);
}

template <>
void GetAttribute<int>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ int& value)
{
	node->QueryIntAttribute(attr, &value);
}

template <>
void GetAttribute<unsigned>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ unsigned& value)
{
	node->QueryUnsignedAttribute(attr, &value);
}

template <>
void GetAttribute<string>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ string& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr)
	{
		value = attrval;
	}
}


void ParseSceneObjectAttributes(SceneObject *pObj, XMLElement* node)
{
	GetAttribute(node, "name", pObj->Name);
	GetAttribute(node, "tag", pObj->Tag);

	Vector3 scale(1.0f);
	Vector3 pos;
	Vector3 eular;

	GetAttribute(node, "position", pos);
	GetAttribute(node, "scale", scale);
	GetAttribute(node, "orientation", eular);

	pObj->SetPosition(pos);
	pObj->SetScale(scale);
	pObj->SetOrientation(Quaternion::CreateFromYawPitchRoll(eular.x, eular.y, eular.z));
}

std::unique_ptr<SceneObject> ParseSceneObject(Scene& scene, XMLElement* node, SceneObject* parent)
{
	using namespace DirectX;

	auto& assets = scene.Assets();
	RenderDevice device;
	uptr<SceneObject> pObj;

	// parent property
	if (*std::find(node->Name(), node->Name() + strlen(node->Name()), '.'))
		return nullptr;

	if (!strcmp(node->Name(), "object"))
	{
		auto pEntity = make_unique<RenderableSceneObject>();
		ParseRenderableObjectAttributes(pEntity.get(), node, assets);
		pObj = move(pEntity);
	}
	else if (!strcmp(node->Name(), "camera"))
	{
		auto pCamera = make_unique<Camera>();
		ParseCameraAttributes(pCamera.get(), node, scene, assets.GetRenderDevice());
		pObj = move(pCamera);
	}
	else if (!strcmp(node->Name(), "skydome"))
	{
		//auto pSky = make_unique<SkyDome>();
	}
	else if (!strcmp(node->Name(), "creature"))
	{
		auto pCreature = make_unique<KinematicSceneObject>();
		ParseCreatureAttributes(pCreature.get(), node, assets);

		pObj = move(pCreature);
	}
	else if (!strcmp(node->Name(), "first_person_keyboard_mouse_control"))
	{
		auto pControl = make_unique<KeyboardMouseFirstPersonControl>();
		pControl->SetTarget(parent);

		pObj = move(pControl);
		ParseSceneObjectAttributes(pObj.get(), node);
	}
	else if (!strcmp(node->Name(), "coordinate_axis"))
	{
		auto pControl = make_unique<CoordinateAxis>();
		pObj = move(pControl);
		ParseSceneObjectAttributes(pObj.get(), node);
	}
	else if (!strcmp(node->Name(), "player_controller"))
	{
		auto pControl = make_unique<PlayerProxy>();
		pObj = move(pControl);
		//pObj.reset();
		ParseSceneObjectAttributes(pObj.get(), node);
	}
	else if (!strcmp(node->Name(), "kinect_visualizer"))
	{
		auto pControl = make_unique<KinectVisualizer>();
		pObj = move(pControl);
		ParseSceneObjectAttributes(pObj.get(), node);
	}
	else
	{
		pObj = make_unique<SceneObject>();
		ParseSceneObjectAttributes(pObj.get(), node);
	}

	if (pObj)
		pObj->ParentScene = &scene;

	node = node->FirstChildElement();
	while (node)
	{
		auto pChild = ParseSceneObject(scene, node, pObj.get()).release();
		{
			std::lock_guard<mutex> guard(scene.ContentMutex());
			pObj->AddChild(pChild);
			//if (pChild->Is<Camera>())
			scene.SignalCameraCache();
		}
		node = node->NextSiblingElement();
	}

	// suffix construction 
	auto pController = dynamic_cast<PlayerProxy*>(pObj.get());
	if (pController)
		pController->Initialize();

	return pObj;
}

void ParseCreatureAttributes(KinematicSceneObject* pCreature, tinyxml2::XMLElement * node, Causality::AssetDictionary & assets)
{
	ParseRenderableObjectAttributes(pCreature, node, assets);

	auto path = node->Attribute("behavier");
	if (path != nullptr && strlen(path) != 0)
	{
		if (path[0] == '{') // asset reference
		{
			const std::string key(path + 1, path + strlen(path) - 1);
			pCreature->SetBehavier(assets.GetBehavier(key));
			if (pCreature->Behavier().Clips().size() > 0)
				pCreature->StartAction(pCreature->Behavier().Clips().front().Name);
		}
	}
	else
	{
		auto inlineBehave = node->FirstChildElement("creature.behavier");
		if (inlineBehave)
		{
			inlineBehave = inlineBehave->FirstChildElement();
			auto& behavier = ParseBehavierAsset(assets, inlineBehave);
			pCreature->SetBehavier(behavier);
		}
	}
}

void ParseCameraAttributes(Camera *pCamera, tinyxml2::XMLElement * node, Causality::Scene & scene, Causality::RenderDevice &device)
{
	using namespace DirectX;
	ParseSceneObjectAttributes(pCamera, node);
	auto rtnode = node->FirstChildElement("camera.render_target");
	bool enable_stereo;
	node->QueryBoolAttribute("enable_stereo", &enable_stereo);
	float fov = 70, aspect = 1, hfov, wfov;
	float _near = 0.01f, _far = 100.0f;
	Vector3 focus = (XMVECTOR)pCamera->GetPosition() + XMVector3Rotate(Camera::Foward, pCamera->GetOrientation());

	GetAttribute(node, "background", pCamera->Background);
	GetAttribute(node, "fov", fov);
	GetAttribute(node, "near", _near);
	GetAttribute(node, "far", _far);
	GetAttribute(node, "focus", focus);
	GetAttribute(node, "aspect", aspect);

	pCamera->SetRenderContext(scene.GetRenderContext());
	pCamera->FocusAt(focus, g_XMIdentityR1.v);

	if (fov != 0)
	{
		pCamera->SetPerspective(XMConvertToRadians(fov), aspect);
	}
	else
	{
		GetAttribute(node, "hfov", hfov);
		GetAttribute(node, "wfov", wfov);
		pCamera->SetOrthographic(wfov, hfov);
	}

	if (!rtnode)
	{
		pCamera->SetRenderTarget(scene.Canvas());
	}
	else
	{
		int width, height;
		rtnode = rtnode->FirstChildElement("render_target");
		GetAttribute(rtnode, "width", width);
		GetAttribute(rtnode, "height", height);
		pCamera->SetRenderTarget(RenderTarget(device, width, height));
	}
}

void ParseRenderableObjectAttributes(RenderableSceneObject* pObj, tinyxml2::XMLElement * node, Causality::AssetDictionary & assets)
{
	ParseSceneObjectAttributes(pObj, node);

	float mass = 1.0f;
	GetAttribute(node, "mass", mass);

	auto path = node->Attribute("mesh");
	if (path != nullptr && strlen(path) != 0)
	{
		if (path[0] == '{') // asset reference
		{
			const std::string key(path + 1, path + strlen(path) - 1);
			pObj->SetRenderModel(&assets.GetMesh(key));
		}
	}
	else
	{
		auto nMesh = node->FirstChildElement("object.mesh");
		if (nMesh)
		{
			nMesh = nMesh->FirstChildElement();
			auto& model = ParseMeshAsset(assets, nMesh);
			pObj->SetRenderModel(&model);
		}
	}
}