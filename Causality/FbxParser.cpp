#include <fbxsdk.h>
#include "FbxParser.h"
namespace fbx = fbxsdk_2015_1;

using namespace Causality;
using namespace fbx;

struct FbxModel::Impl
{
public:

	weak_ptr<FbxManager> sw_SdkManager;
	shared_ptr<FbxManager> m_SdkManger;

	FbxImporter* m_Importer;
	FbxScene*	 m_Scene;
	AnimationSpace* m_AnimationSpace;
	StaticArmature* m_Armature;

	typedef Eigen::Array<Eigen::Array<float, 2, 3>, Eigen::Dynamic, Eigen::Dynamic> FramesMatrix;

	FramesMatrix			m_FrameBuffer;
	vector<FbxNode*>		m_SkeletonNodes;
	vector<int>				m_ParentMap;
	int						m_FrameCount;
	FbxTime					m_FrameTime;
	FbxTime					m_AnimationTime;
	int						m_NodeIdx = 0;

	void ParseNode(FbxNode* pNode) {
		const char* nodeName = pNode->GetName();
		auto translation = pNode->LclTranslation.Get();
		auto rotation = pNode->LclRotation.Get();
		auto scaling = pNode->LclScaling.Get();

#if defined(_DEBUG)
		printf("<node name='%s' id='%d' translation='(%f, %f, %f)' rotation='(%f, %f, %f)' scaling='(%f, %f, %f)'>\n",
			nodeName,
			m_NodeIdx,
			translation[0], translation[1], translation[2],
			rotation[0], rotation[1], rotation[2],
			scaling[0], scaling[1], scaling[2]
			);
#endif

		bool isSkeleton = false;
		// Print the node's attributes.
		for (int i = 0; i < pNode->GetNodeAttributeCount(); i++)
		{
			auto attr = pNode->GetNodeAttributeByIndex(i);
			if (attr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
			{
				isSkeleton = true;

			}
		}

		if (isSkeleton)
		{
			m_SkeletonNodes.push_back(pNode);
			++m_NodeIdx;
		}
		// Recursively print the children nodes.
		for (int j = 0; j < pNode->GetChildCount(); j++)
			ParseNode(pNode->GetChild(j));
	}

	bool LoadFromFile(const string & file)
	{
		if (sw_SdkManager.expired())
		{
			m_SdkManger.reset(FbxManager::Create(),
				[](FbxManager* sdkManager) { sdkManager->Destroy();});
			sw_SdkManager = m_SdkManger;

			// Create the io settings object.
			fbx::FbxIOSettings *ios = FbxIOSettings::Create(m_SdkManger.get(), IOSROOT);
			m_SdkManger->SetIOSettings(ios);
		}
		else
		{
			m_SdkManger = sw_SdkManager.lock();
		}

		auto lSdkManager = m_SdkManger.get();
		auto lFilename = file.c_str();

		// Create an importer using our sdk manager.
		fbx::FbxImporter* lImporter = fbx::FbxImporter::Create(lSdkManager, "");

		// Use the first argument as the filename for the importer.
		if (!lImporter->Initialize(lFilename, -1, lSdkManager->GetIOSettings())) {
			printf("Call to FbxImporter::Initialize() failed.\n");
			printf("Error returned: %s\n\n", "What a f**k");
			exit(-1);
		}

		// Create a new scene so it can be populated by the imported file.
		fbx::FbxScene* lScene = FbxScene::Create(lSdkManager, "default_scene");

		// Import the contents of the file into the scene.
		lImporter->Import(lScene);

		// The file has been imported; we can get rid of the importer.
		lImporter->Destroy();

		fbx::FbxNode* lRootNode = lScene->GetRootNode();
		m_NodeIdx = 0;
		if (lRootNode) {
			for (int i = 0; i < lRootNode->GetChildCount(); i++)
				ParseNode(lRootNode->GetChild(i));
		}

		// Build Parent Map and names to create the Armature
		auto numBones = m_SkeletonNodes.size();
		m_ParentMap.resize(numBones);
		std::vector<const char*> boneNames(numBones);
		for (size_t nodeIdx = 0; nodeIdx < numBones; nodeIdx++)
		{
			auto pNode = m_SkeletonNodes[nodeIdx];
			boneNames[nodeIdx] = pNode->GetName();

			m_ParentMap[nodeIdx] = -1;
			auto pParent = pNode->GetParent();
			while (pParent)
			{
				auto pItr = std::find(m_SkeletonNodes.rbegin() + (numBones - nodeIdx - 1), m_SkeletonNodes.rend(), pParent);
				if (pItr != m_SkeletonNodes.rend())
				{
					auto parentIdx = pItr - m_SkeletonNodes.rbegin();
					parentIdx = numBones - 1 - parentIdx;
					m_ParentMap[nodeIdx] = parentIdx;
					assert(m_SkeletonNodes[parentIdx] == pParent);
					break;
				}
				else
				{
					pParent = pParent->GetParent();
				}
			}
		}

		m_Armature = new StaticArmature(numBones, m_ParentMap.data(), boneNames.data());

		//1.Extract the animation stacks using a pointer to an instance of the FbxScene (pScene).
		int numStacks = lScene->GetSrcObjectCount<fbx::FbxAnimStack>();

		for (size_t stackIdx = 0; stackIdx < numStacks; stackIdx++)
		{
			fbx::FbxAnimStack* pAnimStack = lScene->GetSrcObject<fbx::FbxAnimStack>(stackIdx);
			auto pEvaluator = lScene->GetAnimationEvaluator();

			m_AnimationTime = pAnimStack->GetLocalTimeSpan().GetDuration();
			auto stackName = pAnimStack->GetName();

			m_FrameTime.SetFrame(1);
			m_FrameCount = m_AnimationTime.GetFrameCount();

			pAnimStack->BakeLayers(lScene->GetAnimationEvaluator(), 0, m_AnimationTime, m_FrameTime);
			lScene->SetCurrentAnimationStack(pAnimStack);

			auto& anim = m_AnimationSpace->AddAnimationClip(stackName);

			RasterizeFramesBuffer(pAnimStack, m_FrameBuffer);
		}
	}

	void RasterizeFramesBuffer(_In_ FbxAnimStack* pAnimStack, _Out_ FramesMatrix& frames)
	{
		auto numBones = m_Armature->size();
		frames.resize(m_FrameCount, numBones);
		for (size_t nodeIdx = 0; nodeIdx < numBones; nodeIdx++)
		{
			auto pNode = m_SkeletonNodes[nodeIdx];
			FbxTime time = pAnimStack->GetLocalTimeSpan().GetStart();

			for (int i = 0; i < m_FrameCount; i++)
			{
				using namespace Eigen;
				auto& transform = m_FrameBuffer(i, nodeIdx);

				//auto t = pNode->LclTranslation.EvaluateValue(time);
				auto r = pNode->LclRotation.EvaluateValue(time);
				//auto s = pNode->LclScaling.EvaluateValue(time);
				auto glbM = pNode->EvaluateGlobalTransform(time);
				auto t = glbM.GetT();

				FbxQuaternion fbxq;
				fbxq.ComposeSphericalXYZ(r);
				//auto fbxq = glbM.GetQ();
				DirectX::Quaternion q(fbxq[0], fbxq[1], fbxq[2], fbxq[3]);
				q = XMQuaternionLn(q);

				transform << t[0], t[1], t[2],
					//r[0], r[1], r[2];
					q.x, q.y, q.z;
					//s[0], s[1], s[2];

				time += m_FrameTime;
			}
		}
	}
};

bool Causality::FbxModel::LoadFromFile(const string & file)
{
	return m_pImpl->LoadFromFile(file);
}
