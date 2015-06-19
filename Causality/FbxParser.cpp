#include "pch_bcl.h"
#include <fbxsdk.h>
#include "FbxParser.h"
#include "EigenExtension.h"
#include "BoneFeatures.h"
#include <iostream>
#include <DirectXMathSSE4.h>
namespace fbx = fbxsdk_2015_1;

//using namespace fbx;
namespace Causality
{
	static const float PcaCutoff = 0.01f; // 0.2^2
	const size_t MaxBlendSize = 16U;
	const size_t ReducedBlendSize = 4U;


	struct FbxAnimationParser::Impl
	{
	public:
		weak_ptr<FbxManager>   sw_SdkManager;
		shared_ptr<FbxManager> m_SdkManger;
		vector<FbxMesh*>		m_MeshNodes;
		vector<FbxNode*>		m_BoneNodes;

		BehavierSpace*         m_Behavier;
		StaticArmature*        m_Armature;

		static const Eigen::DenseIndex DimPerBone = CharacterFeature::Dimension;

		std::list<SkinMeshData>  m_SkinnedMeshes;
		vector<int>				m_ParentMap;
		int						m_FrameCount;
		FbxTime					m_FrameInterval;
		FbxTime					m_AnimationTime;
		int						m_NodeIdx = 0;

		int GetBoneNodeId(const FbxNode* pBone) const
		{
			auto itr = std::find(m_BoneNodes.begin(), m_BoneNodes.end(), pBone);
			if (itr == m_BoneNodes.end())
				return -1;
			else
				return itr - m_BoneNodes.begin();
		}

		template <typename T, size_t N>
		void writeVector(std::ostream & binaryFile, const std::array<T, N> & v)
		{
			for (auto i = 0; i < N; i++)
			{
				binaryFile << v[i];
			}
		}
		void writeVector(std::ostream & binaryFile, const fbxsdk_2015_1::FbxVector4 & v)
		{
			binaryFile << (float)v[0] << (float)v[1] << (float)v[2] << (float)v[3];
		}
		void writeVector(std::ostream & binaryFile, const fbxsdk_2015_1::FbxVector2 & v)
		{
			binaryFile << (float)v[0] << (float)v[1];
		}


		SkinMeshData BuildSkinnedMesh(const FbxMesh* pMesh)
		{
			auto numVertices = pMesh->GetControlPointsCount();

			// Vertices
			auto vertices = pMesh->GetControlPoints();

			// Polygons
			auto indices = pMesh->GetPolygonVertices();

			// Assume all polygons are triangle
			bool triangleized = true;
			auto numPolygons = pMesh->GetPolygonCount();
			for (size_t i = 0; i < numPolygons; i++)
			{
				if (pMesh->GetPolygonSize(i) != 3)
				{
					triangleized = false;
					break;
				}
			};
			assert(triangleized);
			auto numIndices = numPolygons * 3;

			// Get UV & Normals
			FbxLayerElementArrayTemplate<fbx::FbxVector4> *pNormals;
			FbxLayerElementArrayTemplate<fbx::FbxVector4> *pTangents;
			FbxLayerElementArrayTemplate<fbx::FbxVector2> *pUVs;
			pMesh->GetNormals(&pNormals);
			pMesh->GetTextureUV(&pUVs);
			pMesh->GetTangents(&pTangents);
			//if (pTangents == nullptr)
			//{
			//	pMesh->GenerateTangentsData(0);
			//	pMesh->GetTangents(&pTangents);
			//}
			auto& normals = *pNormals;
			auto& uvs = *pUVs;
			auto& tagents = *pTangents;


			// Get Skinning data
			std::vector<std::array<float, MaxBlendSize>> blendWeights(numVertices);
			std::vector<std::array<uint8_t, MaxBlendSize>> blendIndices(numVertices);
			std::vector<int> filled(numVertices);
			std::fill_n(filled.begin(), numVertices, 0);

			// Maxium blend bones per vertex
			ZeroMemory(blendWeights.data(), sizeof(float[MaxBlendSize])*numVertices);
			ZeroMemory(blendIndices.data(), sizeof(uint8_t[MaxBlendSize])*numVertices);

			auto numDef = pMesh->GetDeformerCount();
			size_t numBones;
			assert(numDef <= 1);
			for (int i = 0; i < numDef; i++)
			{
				fbx::FbxStatus states;
				auto pDeformer = pMesh->GetDeformer(i,&states);
				assert (pDeformer->GetDeformerType() == FbxDeformer::eSkin);
				fbx::FbxSkin* pSkin = static_cast<fbx::FbxSkin*>(pDeformer);
				auto numClusters = pSkin->GetClusterCount();
				numBones = numClusters;
				assert(numBones < 256);

				for (size_t cId = 0; cId < numClusters; cId++)
				{
					auto pCluster = pSkin->GetCluster(cId);
					auto pBone = pCluster->GetLink();
					auto boneID = GetBoneNodeId(pBone);

					auto numPoints = pCluster->GetControlPointIndicesCount();
					auto indices = pCluster->GetControlPointIndices();
					auto weights = pCluster->GetControlPointWeights();
					for (size_t i = 0; i < numPoints; i++)
					{
						auto idx = indices[i];
						auto bidx = filled[idx]++;
						blendWeights[idx][bidx] = weights[i];
						blendIndices[idx][bidx] = boneID;
					}
				}
			}

			SkinMeshData mesh;
			mesh.Name = pMesh->GetName();
			mesh.BonesCount = numBones;
			mesh.IndexCount = numIndices;
			mesh.VertexCount = numVertices;
			mesh.Vertices = new SkinMeshData::VertexType[numVertices];
			mesh.Indices = new SkinMeshData::IndexType[mesh.IndexCount];
			std::copy(indices, indices + numIndices, mesh.Indices);

			for (size_t i = 0; i < numVertices; i++)
			{
				auto & v = mesh.Vertices[i];
				v.position = DirectX::XMFLOAT3(vertices[i][0], vertices[i][1], vertices[i][2]);
				v.normal = DirectX::XMFLOAT3(normals[i][0], normals[i][1], normals[i][2]);
				if (pTangents)
					v.tangent = DirectX::XMFLOAT4(tagents[i][0], tagents[i][1], tagents[i][2], tagents[i][3]);
				v.SetColor(DirectX::Colors::White);
				v.textureCoordinate = DirectX::XMFLOAT2(uvs[i][0], uvs[i][1]);

				if (filled[i] > ReducedBlendSize)
				{
					for (size_t j = 0; j < 4; j++)
					{
						for (size_t k = j + 1; k < filled[i]; k++)
						{
							if (blendWeights[i][k] > blendWeights[i][j])
							{
								std::swap(blendWeights[i][j], blendWeights[i][k]);
								std::swap(blendIndices[i][j], blendIndices[i][k]);
							}
						}
					}
					float total = blendWeights[i][0] + blendWeights[i][1] + blendWeights[i][2] + blendWeights[i][3];

					// if not, it will be significant artificts
					assert(total > 0.6f);
					auto v = DirectX::XMLoadFloat4(&blendWeights[i][0]);
					v /= total;
					DirectX::XMStoreFloat4(&blendWeights[i][0], v);
				}

				v.SetBlendIndices(DirectX::XMUINT4(blendIndices[i][0], blendIndices[i][1], blendIndices[i][2], blendIndices[i][3]));
				v.SetBlendWeights(reinterpret_cast<DirectX::XMFLOAT4&>(blendWeights[i]));
			}
			return mesh;
		}

		void FindBoneAndMesh(FbxScene* lScene)
		{
			m_MeshNodes.clear();
			m_BoneNodes.clear();
			fbx::FbxNode* lRootNode = lScene->GetRootNode();
			m_NodeIdx = 0;
			if (lRootNode) {
				for (int i = 0; i < lRootNode->GetChildCount(); i++)
					ParseNode(lRootNode->GetChild(i));
			}
		}

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
			bool isMesh = false;
			// Print the node's attributes.
			for (int i = 0; i < pNode->GetNodeAttributeCount(); i++)
			{
				auto attr = pNode->GetNodeAttributeByIndex(i);
				if (attr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
				{
					isSkeleton = true;
				}
				if (attr->GetAttributeType() == FbxNodeAttribute::eMesh)
				{
					isMesh = true;
				}
			}

			if (isMesh)
			{
				auto pMesh = pNode->GetMesh();
				m_MeshNodes.push_back(pMesh);
			}

			if (isSkeleton)
			{
				m_BoneNodes.push_back(pNode);
				++m_NodeIdx;
			}
			// Recursively print the children nodes.
			for (int j = 0; j < pNode->GetChildCount(); j++)
				ParseNode(pNode->GetChild(j));
		}

		StaticArmature* BuildArmatureFromBoneNodes()
		{
			auto numBones = m_BoneNodes.size();
			m_ParentMap.resize(numBones);
			std::vector<const char*> boneNames(numBones);
			for (size_t nodeIdx = 0; nodeIdx < numBones; nodeIdx++)
			{
				auto pNode = m_BoneNodes[nodeIdx];
				boneNames[nodeIdx] = pNode->GetName();

				m_ParentMap[nodeIdx] = -1;
				auto pParent = pNode->GetParent();
				while (pParent)
				{
					auto pItr = std::find(m_BoneNodes.rbegin() + (numBones - nodeIdx - 1), m_BoneNodes.rend(), pParent);
					if (pItr != m_BoneNodes.rend())
					{
						auto parentIdx = pItr - m_BoneNodes.rbegin();
						parentIdx = numBones - 1 - parentIdx;
						m_ParentMap[nodeIdx] = parentIdx;
						assert(m_BoneNodes[parentIdx] == pParent);
						break;
					}
					else
					{
						pParent = pParent->GetParent();
					}
				}
			}

			auto pArmature = new StaticArmature(numBones, m_ParentMap.data(), boneNames.data());
			return pArmature;
		}

		FbxScene* ImportSceneFromFile(const string& file)
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
				return nullptr;
			}

			// Create a new scene so it can be populated by the imported file.
			fbx::FbxScene* lScene = FbxScene::Create(lSdkManager, "default_scene");

			// Import the contents of the file into the scene.
			lImporter->Import(lScene);

			// The file has been imported; we can get rid of the importer.
			lImporter->Destroy();
			return lScene;
		}

		bool Load(const string & file, unsigned mode)
		{

			auto lScene = ImportSceneFromFile(file);
			if (lScene == nullptr)
				return false;

			if (mode & (unsigned)Mode::CreateBehavierAndArmature)
			{
				m_Behavier = new BehavierSpace();
			}

			FindBoneAndMesh(lScene);

			if (mode & (unsigned)Mode::CreateBehavierAndArmature)
			{
				m_Armature = BuildArmatureFromBoneNodes();

				m_Behavier->SetArmature(*m_Armature);
			}

			if (mode & (unsigned)Mode::ImportMeshs)
			{
				for (auto pMesh : m_MeshNodes)
				{
					m_SkinnedMeshes.emplace_back(BuildSkinnedMesh(pMesh));
				}
			}

			if (mode & (unsigned)Mode::ImportAnimations)
				ImportAnimtionsToBehavierProfile(lScene);

			m_MeshNodes.clear();
			m_BoneNodes.clear();
			lScene->Destroy(true);
			return true;
		}

		void ImportAnimtionsToBehavierProfile(fbxsdk_2015_1::FbxScene * lScene, const char* anim_name = nullptr)
		{

			//1.Extract the animation stacks using a pointer to an instance of the FbxScene (pScene).
			int numStacks = lScene->GetSrcObjectCount<fbx::FbxAnimStack>();
			for (size_t stackIdx = 0; stackIdx < numStacks; stackIdx++)
			{
				fbx::FbxAnimStack* pAnimStack = lScene->GetSrcObject<fbx::FbxAnimStack>(stackIdx);
				auto pEvaluator = lScene->GetAnimationEvaluator();

				m_AnimationTime = pAnimStack->GetLocalTimeSpan().GetDuration();
				if (anim_name != nullptr && numStacks == 1)
					pAnimStack->SetName(anim_name);
				auto stackName = pAnimStack->GetName();

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

			m_Armature->set_default_frame(*(new AffineFrame));
			auto& default_frame = m_Armature->default_frame();
			bool builtDefault = default_frame.empty();;

			if (builtDefault)
				default_frame.resize(numBones);

			static const size_t	StretchedSampleCount = ANIM_STANDARD::CLIP_FRAME_COUNT;
			auto frameCount = StretchedSampleCount;

			anim.Duration = time_seconds(pAnimStack->GetLocalTimeSpan().GetDuration().GetSecondDouble());
			auto interval = pAnimStack->GetLocalTimeSpan().GetDuration() / frameCount;

			//anim.FrameInterval = time_seconds(m_FrameInterval.GetSecondDouble());
			anim.FrameInterval = anim.Duration / frameCount;

			buffer.resize(frameCount);
			for (auto& f : buffer)
			{
				f.resize(numBones);
			}

			for (size_t nodeIdx = 0; nodeIdx < numBones; nodeIdx++)
			{
				auto pNode = m_BoneNodes[nodeIdx];
				FbxTime time = pAnimStack->GetLocalTimeSpan().GetStart();
				auto pParent = pNode->GetParent();
				const char *name = pNode->GetName();
				const char *parName = nullptr;
				if (pParent)
				{
					parName = pParent->GetName();
				}

				if (builtDefault)
				{
					auto lclM = pNode->EvaluateLocalTransform();
					auto t = lclM.GetT();
					auto q = lclM.GetQ();
					auto s = lclM.GetS();
					auto& bone = default_frame[nodeIdx];
					bone.LclRotation = Quaternion(q[0], q[1], q[2], q[3]);//q;
					bone.LclScaling = Vector3(s[0], s[1], s[2]);
					bone.LclTranslation = Vector3(t[0], t[1], t[2]);
				}

				float prev_ang;
				DirectX::XMVECTOR prev_axis;
				for (int i = 0; i < frameCount; i++)
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

					// Fix Quaternion 
					{
						using namespace DirectX;
						XMVECTOR q = bone.LclRotation.LoadA();
						XMVECTOR axis;
						float ang;
						XMQuaternionToAxisAngle(&axis, &ang, q);

						if (i > 0
							&& XMVector4Less(XMVector3Dot(prev_axis, axis), XMVectorZero())
							&& abs(ang - prev_ang) > 0.2f
							&& abs(ang + prev_ang - XM_2PI) < 0.2f)
						{
							q = -q;
							bone.LclRotation.StoreA(q);
							XMQuaternionToAxisAngle(&axis, &ang, q);
						}

						prev_ang = ang;
						prev_axis = axis;
					}

					bone.LclScaling = Vector3(s[0], s[1], s[2]);
					bone.LclTranslation = Vector3(t[0], t[1], t[2]);
					//bone.GblScaling = Vector3(gs[0], gs[1], gs[2]);
					//bone.GblRotation = Quaternion(gq[0], gq[1], gq[2], gq[3]);
					//bone.EndPostion = Vector3(gt[0], gt[1], gt[2]);

					time += interval;
				}
			}

			if (builtDefault)
			{
				default_frame.RebuildGlobal(*m_Armature);
			}

			MatrixX Y(frameCount, numBones * DimPerBone);
			auto time = pAnimStack->GetLocalTimeSpan().GetStart();
			for (size_t i = 0; i < frameCount; i++)
			{
				buffer[i].Time = time_seconds(time.GetSecondDouble());
				buffer[i].RebuildGlobal(*m_Armature);
				time += frameCount;
			}

			DirectX::Vector3 sq[2];
			auto mapped = Eigen::Matrix<float, 1, DimPerBone>::Map(&sq[0].x);
			for (size_t i = 0; i < frameCount; i++)
			{
				for (size_t j = 0; j < numBones; j++)
				{
					using namespace DirectX;
					using namespace Eigen;
					auto& feature = Y.block<1, DimPerBone>(i, j * DimPerBone);
					auto& bone = buffer[i][j];

					CharacterFeature::Get(feature, bone);
				}
			}

			using namespace std;
			//cout << "Y : min = " << Y.minCoeff() << " , max = " << Y.maxCoeff() << endl;
			//Y = Eigen::resample(Y, Y.rows(), StretchedSampleCount);
			//cout << "Y_resample : min = " << Y.minCoeff() << " , max = " << Y.maxCoeff() << endl;
			//cout << "Y_centerd : min = " << Y.minCoeff() << " , max = " << Y.maxCoeff() << endl;

			anim.animMatrix = Y;

			//if (anim.Name == "walk")
			//{
			//	ofstream fout("Ylog.txt");
			//	fout << Y;
			//	fout.close();
			//}
			//Y.rowwise() -= Y.colwise().mean();
			//anim.QrYs.resize(m_Behavier->Blocks().size());
			//for (size_t boneIdx = 0; boneIdx < numBones; boneIdx++)
			//{
			//	auto Yb = Y.middleCols<3>(boneIdx*DimPerBone);
			//	anim.QrYs[boneIdx].compute(Yb,false);
			//}

			Eigen::FFT<float> fft;
			Eigen::MatrixXcf Yf(Y.rows(), Y.cols());
			for (size_t i = 0; i < Y.cols(); i++)
			{
				fft.fwd(Yf.col(i).data(), Y.col(i).data(), Y.rows());
			}

			{
				Eigen::VectorXf Ecd = Yf.middleRows(1, 5).cwiseAbs2().colwise().sum();
				auto Ecjm = Eigen::Matrix<float, DimPerBone, -1>::Map(Ecd.data(), DimPerBone, numBones);
				anim.Ecj = Ecjm.colwise().sum();
				// Do not normalize normalize yet.
			}

			//? We can do better by using a permutation matrix
			const auto& blocks = m_Behavier->Blocks();
			auto bSize = blocks.size();

			anim.Ecb.resize(bSize);
			anim.QrYs.resize(bSize);
			anim.PcaYs.resize(bSize);
			anim.Ys.resize(bSize);
			anim.Ysp.setZero(DimPerBone, bSize);
			for (auto& block : blocks)
			{
				auto i = block->Index;
				auto& joints = block->Joints;

				auto lastJid = joints.back()->ID();
				auto vtc = anim.Ysp.col(i);
				for (const auto& frame : buffer)
				{
					vtc += Eigen::Vector3f::MapAligned(&frame[lastJid].EndPostion.x);
				}
				vtc /= frameCount;

				Eigen::MatrixXf Yb(Y.rows(), block->GetFeatureDim<CharacterFeature>());
				for (size_t j = 0; j < joints.size(); j++)
				{
					Yb.middleCols<DimPerBone>(j * DimPerBone) = Y.middleCols<DimPerBone>(joints[j]->ID() * DimPerBone);
					anim.Ecb(i) = max(anim.Ecb(i), anim.Ecj(joints[j]->ID()));
				}

				anim.Ys[i] = Yb;

				if (i == 14 && anim.Name == "walk")
				{
					const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
					ofstream b14lnq("J14.csv");
					b14lnq << Yb.format(CSVFormat) << endl;
					b14lnq.close();
				}

				auto & pca = anim.PcaYs[i];
				pca.compute(Yb, true);
				auto d = pca.reducedRank(PcaCutoff);
				anim.QrYs[i].compute(pca.coordinates(d), true);

				using namespace DirectX;
				using DirectX::operator+=;
				auto jid = block->Joints.back()->ID();
				auto sum = XMVectorZero();
				for (auto& frame : buffer)
				{
					auto v = frame[jid].EndPostion.LoadA();
					sum += v;
				}
				sum /= buffer.size();
				auto pData = reinterpret_cast<XMFLOAT3*>(anim.Ysp.col(i).data());
				XMStoreFloat3(pData, sum);

				//sum = XMVector3Normalize(sum);
			}

			for (auto& block : blocks)
			{
				auto i = block->Index;
				auto& joints = block->Joints;
				if (block->parent() != nullptr)
				{
					auto pi = block->parent()->Index;
					anim.Ysp.col(i) -= anim.Ysp.col(pi);
				}
			}
			anim.Ysp.colwise().normalize();
		}
	};

	FbxAnimationParser::FbxAnimationParser(const string & file, unsigned mode)
	{
		Load(file, mode);
	}

	void FbxAnimationParser::SetBehavierProfile(BehavierSpace * pBehav)
	{
		m_pImpl->m_Behavier = pBehav;
	}

	bool FbxAnimationParser::Load(const string & file, unsigned mode)
	{
		if (!m_pImpl)
			m_pImpl.reset(new FbxAnimationParser::Impl);
		return m_pImpl->Load(file, mode);
	}

	bool FbxAnimationParser::ImportBehavier(const string & file)
	{
		if (!m_pImpl)
			m_pImpl.reset(new FbxAnimationParser::Impl);
		return m_pImpl->Load(file, (unsigned)Mode::CreateBehavierAndArmature | (unsigned)Mode::ImportAnimations);
	}

	bool FbxAnimationParser::ImportMesh(const string & file)
	{
		if (!m_pImpl)
			m_pImpl.reset(new FbxAnimationParser::Impl);
		return m_pImpl->Load(file, (unsigned)Mode::ImportMeshs);
	}

	bool FbxAnimationParser::ImportAnimation(const string & file, const string & animationName)
	{
		if (!m_pImpl)
			m_pImpl.reset(new FbxAnimationParser::Impl);
		return m_pImpl->Load(file, (unsigned)Mode::ImportAnimations);
	}

	const std::list<SkinMeshData>& FbxAnimationParser::GetMeshs()
	{
		return m_pImpl->m_SkinnedMeshes;
	}

	StaticArmature * FbxAnimationParser::GetArmature()
	{
		return m_pImpl->m_Armature;
	}

	BehavierSpace* FbxAnimationParser::GetBehavier()
	{
		return m_pImpl->m_Behavier;
	}

	FbxAnimationParser::~FbxAnimationParser()
	{

	}
	FbxAnimationParser::FbxAnimationParser()
	{

	}

	ArmatureFrameAnimation BinaryFileLoader::LoadAnimationClipFromFile(const string & filename)
	{
		return ArmatureFrameAnimation();
	}

	SkinMeshData BinaryFileLoader::LoadSkinnedMeshFromFile(const string & filename)
	{
		SkinMeshData mesh;
		using namespace std;
		ifstream fin(filename, ios::binary | ios::in);
		mesh.Deserialize(fin);
		return mesh;
	}

	SkinMeshData::SkinMeshData()
	{
		VertexCount = 0;
		IndexCount = 0;
		BonesCount = 0;
		Vertices = nullptr;
		Indices = nullptr;
	}

	void SkinMeshData::Release()
	{
		if (Vertices)
		{
			delete Vertices;
			Vertices = nullptr;
		}
		if (Indices)
		{
			delete Indices;
			Indices = nullptr;
		}
	}

	void SkinMeshData::Serialize(std::ostream & binary) const
	{
		binary << VertexCount << IndexCount << BonesCount << (uint32_t)sizeof(SkinMeshData::VertexType) << (uint32_t)sizeof(SkinMeshData::IndexType);
		binary.write(reinterpret_cast<const char*>(Vertices), sizeof(SkinMeshData::VertexType)*VertexCount);
		binary.write(reinterpret_cast<const char*>(Indices), sizeof(SkinMeshData::IndexType)*IndexCount);
	}

	void SkinMeshData::Deserialize(std::istream & binary)
	{
		uint32_t VertexSizeInByte, IndexSizeInByte;
		binary >> VertexCount >> IndexCount >> BonesCount >> VertexSizeInByte >> IndexSizeInByte;

		assert(IndexCount % PolygonSize == 0);
		assert(sizeof(SkinMeshData::VertexType) == VertexSizeInByte);
		assert(sizeof(SkinMeshData::IndexType) == IndexSizeInByte);

		binary.read(reinterpret_cast<char*>(Vertices), sizeof(SkinMeshData::VertexType)*VertexCount);
		binary.read(reinterpret_cast<char*>(Indices), sizeof(SkinMeshData::IndexType)*IndexCount);
	}
}