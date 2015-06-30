#include "auto_pch.h"
#include "Source.h"
#include "..\Causality\CCA.h"
#include "..\Causality\EigenExtension.h"
#include <iomanip>

using namespace DirectX;
using namespace std;
using namespace boost::filesystem;

/* Tab character ("\t") counter */
int numTabs = 0;

/**
* Print the required number of tabs.
*/
void PrintTabs() {
	for (int i = 0; i < numTabs; i++)
		printf("\t");
}

/**
* Return a string-based representation based on the attribute type.
*/
FbxString GetAttributeTypeName(FbxNodeAttribute::EType type) {
	switch (type) {
	case FbxNodeAttribute::eUnknown: return "unidentified";
	case FbxNodeAttribute::eNull: return "null";
	case FbxNodeAttribute::eMarker: return "marker";
	case FbxNodeAttribute::eSkeleton: return "skeleton";
	case FbxNodeAttribute::eMesh: return "mesh";
	case FbxNodeAttribute::eNurbs: return "nurb";
	case FbxNodeAttribute::ePatch: return "patch";
	case FbxNodeAttribute::eCamera: return "camera";
	case FbxNodeAttribute::eCameraStereo:    return "stereo";
	case FbxNodeAttribute::eCameraSwitcher: return "camera switcher";
	case FbxNodeAttribute::eLight: return "light";
	case FbxNodeAttribute::eOpticalReference: return "optical reference";
	case FbxNodeAttribute::eOpticalMarker: return "marker";
	case FbxNodeAttribute::eNurbsCurve: return "nurbs curve";
	case FbxNodeAttribute::eTrimNurbsSurface: return "trim nurbs surface";
	case FbxNodeAttribute::eBoundary: return "boundary";
	case FbxNodeAttribute::eNurbsSurface: return "nurbs surface";
	case FbxNodeAttribute::eShape: return "shape";
	case FbxNodeAttribute::eLODGroup: return "lodgroup";
	case FbxNodeAttribute::eSubDiv: return "subdiv";
	case FbxNodeAttribute::eCachedEffect: return "cached effect";
	case FbxNodeAttribute::eLine: return "line";
	default: return "unknown";
	}
}

/**
* Print an attribute.
*/
void PrintAttribute(FbxNodeAttribute* pAttribute) {
	if (!pAttribute) return;

	FbxString typeName = GetAttributeTypeName(pAttribute->GetAttributeType());
	FbxString attrName = pAttribute->GetName();
	PrintTabs();
	// Note: to retrieve the chararcter array of a FbxString, use its Buffer() method.
	printf("<attribute type='%s' name='%s'/>\n", typeName.Buffer(), attrName.Buffer());
}

void PrintCurveNode(const char* propertyName, FbxAnimCurveNode* pCurveNode)
{
	PrintTabs();

	printf("<animation property='%s'>\n",propertyName);
	++numTabs;
	int numChannels = pCurveNode->GetChannelsCount();
	for (int chlIdx = 0; chlIdx < numChannels; chlIdx++)
	{
		auto chlName = pCurveNode->GetChannelName(chlIdx);
		PrintTabs();
		printf("<channel id='%d' name='%s'>\n", chlIdx, chlName.Buffer());
		++numTabs;
		PrintTabs();
		int numCurv = pCurveNode->GetCurveCount(chlIdx);
		for (int curvIdx = 0; curvIdx < numCurv; curvIdx++)
		{
			auto pCurve = pCurveNode->GetCurve(chlIdx, curvIdx);
			int numKey = pCurve->KeyGetCount();
			for (int key = 0; key < numKey; key++)
			{
				auto &frame = pCurve->KeyGet(key);
				printf(" (%s : %.3f)", frame.GetTime().GetTimeString(), frame.GetValue());
			}
		}
		--numTabs;
		printf("\n");
		PrintTabs();
		printf("</channel>\n", chlIdx, chlName.Buffer());
	}

	--numTabs;
	PrintTabs();
	printf("</animation>\n");
}

std::vector<const char*> g_NodeNames;
Eigen::Array<Eigen::Array<float,2,3>, Eigen::Dynamic, Eigen::Dynamic> g_AnimationFrameBuffer;
std::vector<FbxNode*>	g_SkeletonNodes;

int g_Frames;
FbxTime g_FrameTime;
FbxTime g_AnimationTime;
int g_NodeIdx = 0;

/**
* Print a node, its attributes, and all its children recursively.
*/
void PrintNode(FbxNode* pNode) {
	PrintTabs();
	const char* nodeName = pNode->GetName();
	auto translation = pNode->LclTranslation.Get();
	auto rotation = pNode->LclRotation.Get();
	auto scaling = pNode->LclScaling.Get();

	// print the contents of the node.
	g_NodeNames.push_back(nodeName);
	printf("<node name='%s' id='%d' translation='(%f, %f, %f)' rotation='(%f, %f, %f)' scaling='(%f, %f, %f)'>\n",
		nodeName,
		g_NodeIdx,
		translation[0], translation[1], translation[2],
		rotation[0], rotation[1], rotation[2],
		scaling[0], scaling[1], scaling[2]
		);
	numTabs++;

	bool isSkeleton = false;
	// Print the node's attributes.
	for (int i = 0; i < pNode->GetNodeAttributeCount(); i++)
	{
		auto attr = pNode->GetNodeAttributeByIndex(i);
		PrintAttribute(attr);
		if (attr->GetAttributeType() == FbxNodeAttribute::eSkeleton)
		{
			isSkeleton = true;

			//PrintCurveNode("translation", pNode->LclTranslation.GetCurveNode());
			//auto pSk = dynamic_cast<FbxSkeleton*>(attr);
		}
	}

	if (isSkeleton)
	{
		g_SkeletonNodes.push_back(pNode);
		++g_NodeIdx;
	}
	// Recursively print the children nodes.
	for (int j = 0; j < pNode->GetChildCount(); j++)
		PrintNode(pNode->GetChild(j));

	numTabs--;
	PrintTabs();
	printf("</node>\n");
}

/**
* Main function - loads the hard-coded fbx file,
* and prints its contents in an xml format to stdout.
*/

int main(int argc, wchar_t** argv)
{
	using namespace Eigen;

	// Resample Test
	Eigen::VectorXf v(6);
	v << 0, 1, 3, 7, 3, 1;
	cout << " X = " << v.transpose() << endl;;
	laplacian_smooth(v);
	cout << " Y = " << v.transpose() << endl;;

	// CCA Test
	Eigen::MatrixXf X(6,3), Y(6,3);
	//X<< 1, 0, 0,
	//	0, 1, 0,
	//	0, 0, 1,
	//	1, 0, 0,
	//	0, 1, 0,
	//	0, 0, 1;
	//Y<< 0, 2, 0,
	//	0, 0, 2,
	//	2, 0, 0,
	//	0, 2, 0,
	//	0, 0, 2,
	//	2, 0, 0;
	X <<
		0.8147, 0.2785, 0.9572,
		0.9058, 0.5469, 0.4854,
		0.1270, 0.9575, 0.8003,
		0.9134, 0.9649, 0.1419,
		0.6324, 0.1576, 0.4218,
		0.0975, 0.9706, 0.9157;
	Y <<
		0.7922, 0.6787, 0.7060,
		0.9595, 0.7577, 0.0318,
		0.6557, 0.7431, 0.2769,
		0.0357, 0.3922, 0.0462,
		0.8491, 0.6555, 0.0971,
		0.9340, 0.1712, 0.8235;

	cout << setprecision(3) << setw(5);
	cout << "X = \n" << X << endl;
	cout << "Y = \n" << Y << endl;

	//PCA test
	Eigen::Pca<MatrixXf> pca(X);
	cout << "principle compoenets = \n" << pca.components() << endl;
	cout << "projected coordinates = \n" << pca.coordinates() << endl;


	system("PAUSE");
	exit(0);

	Eigen::MeanThinQr<Eigen::MatrixXf> qrX(X),qrY(Y);
	cout << "QR(X) : " << endl;
	cout << "Qx = \n" << qrX.matrixQ() << endl;
	cout << "Rx = \n" << qrX.m_R << endl;
	cout << "Px = " << qrX.colsPermutation().indices().transpose() << endl;

	cout << "QR(Y) : " << endl;
	cout << "Qy = \n" << qrY.matrixQ() << endl;
	cout << "Ry = \n" << qrY.m_R << endl;
	cout << "Py = " << qrY.colsPermutation().indices().transpose() << endl;
	cout << "Qy * Ry = \n" << qrY.matrixQ() * qrY.matrixR() << endl;

	Eigen::Cca<float> cca;
	cca.computeFromQr(qrX, qrY,true);
	cout << "CCA : " << endl;
	cout << "A = \n" << cca.matrixA() << endl;
	cout << "B = \n" << cca.matrixB() << endl;
	cout << "r = \n" << cca.correlaltions().transpose() << endl;

	MatrixXf Xz = X.rowwise() - qrX.mean();
	MatrixXf Yz = Y.rowwise() - qrY.mean();
	cout << DebugLog(Xz);
	cout << DebugLog(Yz);

	MatrixXf U = Xz * cca.matrixA();
	MatrixXf V = Yz * cca.matrixB();
	cout << DebugLog(U);
	cout << DebugLog(V);

	exit(0);


	path animation_file(R"(D:\User\Yupeng\Documents\GitHub\VR-Interaction\Causality\Resources\Animations\Horse_Run.fbx)");
	//path animation_file(
	//	R"(D:\User\Yupeng\Documents\GitHub\VR-Interaction\Causality\Resources\Models\spider.fbx)");

	path modelName = animation_file.filename();

	auto str = animation_file.string();
	const char *  lFilename = str.c_str();
	FILE* output = nullptr;

	namespace fbx = fbxsdk_2015_1;
	// Initialize the sdk manager. This object handles all our memory management.
	fbx::FbxManager* lSdkManager = fbx::FbxManager::Create();

	// Create the io settings object.
	fbx::FbxIOSettings *ios = fbx::FbxIOSettings::Create(lSdkManager, IOSROOT);
	lSdkManager->SetIOSettings(ios);

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

	// Print the nodes of the scene and their attributes recursively.
	// Note that we are not printing the root node, because it should
	// not contain any attributes.
	freopen_s(&output, "Spider.xml", "w", stdout);

	fbx::FbxNode* lRootNode = lScene->GetRootNode();
	g_NodeIdx = 0;
	if (lRootNode) {
		for (int i = 0; i < lRootNode->GetChildCount(); i++)
			PrintNode(lRootNode->GetChild(i));
	}

	auto numBones = g_SkeletonNodes.size();
	{
		auto outputName = modelName;
		ofstream nameout(outputName.replace_extension(".bone.csv").c_str());
		for (auto pNode : g_SkeletonNodes)
		{
			auto name = pNode->GetName();
			nameout << '"' << name << "\",";
		}
		nameout.close();
	}


	//1.Extract the animation stacks using a pointer to an instance of the FbxScene (pScene).
	int numStacks = lScene->GetSrcObjectCount<fbx::FbxAnimStack>();

	for (size_t stackIdx = 0; stackIdx < numStacks; stackIdx++)
	{
		fbx::FbxAnimStack* pAnimStack = lScene->GetSrcObject<fbx::FbxAnimStack>(stackIdx);
		auto pEvaluator = lScene->GetAnimationEvaluator();

		g_AnimationTime = pAnimStack->GetLocalTimeSpan().GetDuration();
		auto stackName = pAnimStack->GetName();

		g_FrameTime.SetFrame(1);
		g_Frames = g_AnimationTime.GetFrameCount();

		pAnimStack->BakeLayers(lScene->GetAnimationEvaluator(), 0, g_AnimationTime, g_FrameTime);
		//fbx::FbxAnimLayer* lAnimLayer = pAnimStack->GetMember<fbx::FbxAnimLayer>(0);
		lScene->SetCurrentAnimationStack(pAnimStack);
		//stackName = lScene->GetCurrentAnimationStack()->GetName();

		g_AnimationFrameBuffer.resize(g_Frames, numBones);

		auto outputName = modelName;
		if (stackName)
			outputName.replace_extension('.' + string(stackName) + ".csv");

		std::vector<size_t> g_ParentMap(numBones);

		for (size_t nodeIdx = 0; nodeIdx < numBones; nodeIdx++)
		{
			FbxTime time = pAnimStack->GetLocalTimeSpan().GetStart();
			auto pNode = g_SkeletonNodes[nodeIdx];

			g_ParentMap[nodeIdx] = -1;
			auto pParent = pNode->GetParent();
			while (pParent)
			{
				auto pItr = std::find(g_SkeletonNodes.rbegin() + (numBones - nodeIdx - 1), g_SkeletonNodes.rend(), pParent);
				if (pItr != g_SkeletonNodes.rend())
				{
					auto parentIdx = pItr - g_SkeletonNodes.rbegin();
					parentIdx = numBones - 1 - parentIdx;
					g_ParentMap[nodeIdx] = parentIdx;
					assert(g_SkeletonNodes[parentIdx] == pParent);
					break;
				}
				else
				{
					pParent = pParent->GetParent();
				}
			}

			for (int i = 0; i < g_Frames; i++)
			{
				using namespace Eigen;
				auto& transform = g_AnimationFrameBuffer(i, nodeIdx);

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
							 q.x,  q.y,  q.z;
				//s[0], s[1], s[2];
				time += g_FrameTime;
			}
		}

		ofstream fout(outputName.c_str());

		for (int t = 0; t < g_Frames; t++)
		{
			for (int bone = 0; bone < numBones; bone++)
			{
				auto& m = g_AnimationFrameBuffer(t, bone);
				if (g_ParentMap[bone] > -1)
				{
					const auto& pm = g_AnimationFrameBuffer(t, g_ParentMap[bone]);

					//! This is different from "Local Translation"
					// it is "relative position" in global orientation
					//m.block<3, 1>(0, 0) -= pm.block<3, 1>(0, 0);
				}

				fout << m(0, 0) << ',' << m(0, 1) << ',' << m(0, 2) << ','
					 << m(1, 0) << ',' << m(1, 1) << ',' << m(1, 2) << ',';
			}
			fout << endl;
		}
		fout.close();
	}


	//ofstream nameout("bone_names.csv");
	//for (auto& name : g_NodeNames)
	//{
	//	nameout <<'"' << name << "\",";
	//}
	//nameout.close();



	//3.Retrieve the number of animation layers in an animation stack.

	//int numAnimLayers = pAnimStack->GetMemberCount<fbx::FbxAnimLayer>();

	//for (int layIdx = 0; layIdx < numAnimLayers; layIdx++)
	//{
	//	//4.Retrieve the nth animation layer from the animation stack.
	//	fbx::FbxAnimLayer* lAnimLayer = pAnimStack->GetMember<fbx::FbxAnimLayer>(layIdx);
	//	//After retrieving an animation layer, you can access the animation curves for the properties of nodes and node attribute.
	//	int numCurves = lAnimLayer->GetMemberCount<fbx::FbxAnimCurveNode>();

	//	for (int curvIdx = 0; curvIdx < numCurves; curvIdx++)
	//	{
	//		//5.Retrieve the animation curves.Given a node(lNode) and an animation layer(lAnimLayer), you can retrieve the animation curves for node properties such as the local translation as shown in the following example.
	//		fbx::FbxAnimCurveNode* lAnimCurveNode = lAnimLayer->GetMember<fbx::FbxAnimCurveNode>(curvIdx);
	//			//lNode->LclTranslation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_X);

	//	}

	//}

	// Destroy the sdk manager and all other objects it was handling.
	lSdkManager->Destroy();

	//system("PAUSE");
}

void LogMapLinertyTest()
{
	XMVECTOR Q0 = XMQuaternionIdentity();
	XMVECTOR Q1 = XMQuaternionRotationNormal(g_XMIdentityR1.v, XM_PI / 1.0f);
	XMVECTOR Q2 = XMQuaternionRotationNormal(g_XMIdentityR2.v, XM_PI / 1.0f);
	XMVECTOR Q3 = XMQuaternionRotationNormal(g_XMIdentityR0.v, XM_PI / 1.0f);
	//XMVECTOR Q1 = XMQuaternionRotationNormal(g_XMIdentityR1.v, XM_PI / 8.0f);
	//XMVECTOR Q2 = XMQuaternionRotationNormal(g_XMIdentityR2.v, XM_PI / 8.0f);

	XMVECTOR Lq1 = XMQuaternionLn(Q1);
	XMVECTOR Lq2 = XMQuaternionLn(Q2);
	XMVECTOR Lq3 = XMQuaternionLn(Q3);

	Lq3 = Lq1 - Lq2;
	Lq3 = XMQuaternionExp(Lq3);
	Lq3 = XMQuaternionNormalize(Lq3);
	Quaternion lq = Lq3;

	XMVECTOR Sq3 = XMQuaternionMultiply(XMQuaternionConjugate(Q2), Q1);
	Quaternion sq = Sq3;
	cout << "Log interpolate : " << lq << endl;
	cout << "Sphere interpolate : " << sq << endl;
}
