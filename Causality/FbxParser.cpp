#include "pch_bcl.h"
#include <fbxsdk.h>
#include "FbxParser.h"
#include "EigenExtension.h"
#include <iostream>
namespace fbx = fbxsdk_2015_1;

using namespace fbx;
namespace Causality
{
	struct FbxAnimationParser::Impl
	{
	public:

		weak_ptr<FbxManager> sw_SdkManager;
		shared_ptr<FbxManager> m_SdkManger;

		FbxImporter*           m_Importer;
		FbxScene*	           m_Scene;
		BehavierSpace*         m_Behavier;
		StaticArmature*        m_Armature;

		static const Eigen::DenseIndex FeaturePerBone = 6;
		typedef Eigen::Array<Eigen::Array<float, 2, 3>, Eigen::Dynamic, Eigen::Dynamic> FramesMatrix;


		vector<FbxNode*>		m_SkeletonNodes;
		vector<int>				m_ParentMap;
		int						m_FrameCount;
		FbxTime					m_FrameInterval;
		FbxTime					m_AnimationTime;
		int						m_NodeIdx = 0;

		void ParseNode(FbxNode* pNode) {
			const char* nodeName = pNode->GetName();
			auto translation = pNode->LclTranslation.Get();
			auto rotation = pNode->LclRotation.Get();
			auto scaling = pNode->LclScaling.Get();

//#if defined(_DEBUG)
//			printf("<node name='%s' id='%d' translation='(%f, %f, %f)' rotation='(%f, %f, %f)' scaling='(%f, %f, %f)'>\n",
//				nodeName,
//				m_NodeIdx,
//				translation[0], translation[1], translation[2],
//				rotation[0], rotation[1], rotation[2],
//				scaling[0], scaling[1], scaling[2]
//				);
//#endif

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

			m_Behavier = new BehavierSpace();

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
			m_Behavier->SetArmature(*m_Armature);

			//1.Extract the animation stacks using a pointer to an instance of the FbxScene (pScene).
			int numStacks = lScene->GetSrcObjectCount<fbx::FbxAnimStack>();

			for (size_t stackIdx = 0; stackIdx < numStacks; stackIdx++)
			{
				fbx::FbxAnimStack* pAnimStack = lScene->GetSrcObject<fbx::FbxAnimStack>(stackIdx);
				auto pEvaluator = lScene->GetAnimationEvaluator();

				m_AnimationTime = pAnimStack->GetLocalTimeSpan().GetDuration();
				auto stackName = pAnimStack->GetName();

				m_FrameInterval.SetFrame(1);
				m_FrameCount = m_AnimationTime.GetFrameCount();

				pAnimStack->BakeLayers(lScene->GetAnimationEvaluator(), 0, m_AnimationTime, m_FrameInterval);
				lScene->SetCurrentAnimationStack(pAnimStack);

				auto& anim = m_Behavier->AddAnimationClip(stackName);

				RasterizeFramesBuffer(pAnimStack, anim);
			}
		}

		// Two feature get in : Glbal postion / Log map of Local Rotation
		void RasterizeFramesBuffer(_In_ FbxAnimStack* pAnimStack, _Out_ BehavierSpace::animation_type& anim)
		{
			auto numBones = m_Armature->size();
			auto& buffer = anim.GetFrameBuffer();
			buffer.resize(m_FrameCount);
			for (auto& f : buffer)
			{
				f.resize(numBones);
			}

			anim.Duration = time_seconds(pAnimStack->GetLocalTimeSpan().GetDuration().GetSecondDouble());
			anim.FrameInterval = time_seconds(m_FrameInterval.GetSecondDouble());

			for (size_t nodeIdx = 0; nodeIdx < numBones; nodeIdx++)
			{
				auto pNode = m_SkeletonNodes[nodeIdx];
				FbxTime time = pAnimStack->GetLocalTimeSpan().GetStart();
				auto pParent = pNode->GetParent();
				const char *name = pNode->GetName();
				const char *parName = nullptr;
				if (pParent)
				{
					parName = pParent->GetName();
				}

				for (int i = 0; i < m_FrameCount; i++)
				{
					//auto t = pNode->LclTranslation.EvaluateValue(time);
					//auto r = pNode->LclRotation.EvaluateValue(time);
					//auto s = pNode->LclScaling.EvaluateValue(time);

					//FbxEuler::EOrder lRotationOrder;
					//pNode->GetRotationOrder(FbxNode::eSourcePivot, lRotationOrder);
					//FbxAMatrix parM;

					//auto parM = pParent->EvaluateGlobalTransform(time);

					auto lclM = pNode->EvaluateLocalTransform(time);
					auto t = lclM.GetT();
					auto q = lclM.GetQ();
					auto s = lclM.GetS();
					
					//parM *= lclM;

					//auto glbM = pNode->EvaluateGlobalTransform(time);
					//auto gt = glbM.GetT();
					//auto gq = glbM.GetQ();
					//auto gs = glbM.GetS();

					//FbxVector4 rd(r);
					////rd *= (180.0 / DirectX::XM_PI);
					//FbxQuaternion fbxq;
					//fbxq.ComposeSphericalXYZ(rd);
					//DirectX::Quaternion q(fbxq[0], fbxq[1], fbxq[2], fbxq[3]);

					auto& bone = buffer[i][nodeIdx];
					bone.LclRotation = Quaternion(q[0], q[1], q[2], q[3]);//q;
					bone.LclScaling = Vector3(s[0], s[1], s[2]);
					bone.LclTranslation = Vector3(t[0], t[1], t[2]);
					//bone.GblScaling = Vector3(gs[0], gs[1], gs[2]);
					//bone.GblRotation = Quaternion(gq[0], gq[1], gq[2], gq[3]);
					//bone.EndPostion = Vector3(gt[0], gt[1], gt[2]);

					time += m_FrameInterval;
				}
			}

			MatrixX Y (m_FrameCount, numBones * FeaturePerBone);
			FbxTime time = pAnimStack->GetLocalTimeSpan().GetStart();
			DirectX::Vector3 sq[2];
			auto& mapped = Eigen::Matrix<float, 1, FeaturePerBone>::Map(&sq[0].x);
			for (size_t i = 0; i < m_FrameCount; i++)
			{
				buffer[i].Time = time_seconds(time.GetSecondDouble());
				buffer[i].RebuildGlobal(*m_Armature);

				for (size_t j = 0; j < numBones; j++)
				{
					using namespace DirectX;
					using namespace Eigen;
					auto& feature = Y.block<1, FeaturePerBone>(i, j * FeaturePerBone);
					auto& bone = buffer[i][j];
					XMVECTOR q = bone.LclRotation.LoadA();
					q = XMQuaternionLn(q);
					sq[0] = q;
					sq[1] = bone.EndPostion;
					feature = mapped;
				}
				time += m_FrameInterval;
			}

			static const size_t	ScaledFramesCount = 90U;

			using namespace std;
			//cout << "Y : min = " << Y.minCoeff() << " , max = " << Y.maxCoeff() << endl;
			Y = Eigen::resample(Y, Y.rows(), ScaledFramesCount);
			//cout << "Y_resample : min = " << Y.minCoeff() << " , max = " << Y.maxCoeff() << endl;
			//cout << "Y_centerd : min = " << Y.minCoeff() << " , max = " << Y.maxCoeff() << endl;

			anim.AnimMatrix() = Y;

			//Y.rowwise() -= Y.colwise().mean();
			anim.QrYs.resize(numBones);
			for (size_t boneIdx = 0; boneIdx < numBones; boneIdx++)
			{
				auto Yb = Y.middleCols<3>(boneIdx*FeaturePerBone);
				anim.QrYs[boneIdx].compute(Yb,false);
			}
		}
	};

	unique_ptr<BehavierSpace> FbxAnimationParser::LoadFromFile(const string & file)
	{
		if (!m_pImpl)
			m_pImpl.reset(new FbxAnimationParser::Impl);
		m_pImpl->LoadFromFile(file);
		return unique_ptr<BehavierSpace>(m_pImpl->m_Behavier);
	}

	FbxAnimationParser::~FbxAnimationParser()
	{

	}
	FbxAnimationParser::FbxAnimationParser()
	{

	}
}