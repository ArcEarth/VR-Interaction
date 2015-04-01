#include "auto_pch.h"
#include "Source.h"

#ifndef _AUTO_PCH
#include <iostream>
#include <DirectXMath.h>
#include "..\Causality\Common\DirectXMathExtend.h"
#include <fbxsdk.h>
#include <boost\filesystem.hpp>
#endif

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
	printf("<node name='%s' translation='(%f, %f, %f)' rotation='(%f, %f, %f)' scaling='(%f, %f, %f)'>\n",
		nodeName,
		translation[0], translation[1], translation[2],
		rotation[0], rotation[1], rotation[2],
		scaling[0], scaling[1], scaling[2]
		);
	numTabs++;

	// Print the node's attributes.
	for (int i = 0; i < pNode->GetNodeAttributeCount(); i++)
		PrintAttribute(pNode->GetNodeAttributeByIndex(i));

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

int main()
{
	path animation_file(R"(D:\User\Yupeng\Documents\GitHub\VR-Interaction\Causality\Resources\Horse\Animation\Horse_Run.fbx)");
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

	freopen_s(&output, "Horse.xml", "w", stdout);

	fbx::FbxNode* lRootNode = lScene->GetRootNode();
	if (lRootNode) {
		for (int i = 0; i < lRootNode->GetChildCount(); i++)
			PrintNode(lRootNode->GetChild(i));
	}

	auto pScene = lScene;

	//1.Extract the animation stacks using a pointer to an instance of the FbxScene (pScene).
	int numStacks = pScene->GetSrcObjectCount<fbx::FbxAnimStack>();

	fbx::FbxAnimStack* pAnimStack = pScene->GetSrcObject<fbx::FbxAnimStack>(0);

	//3.Retrieve the number of animation layers in an animation stack.

	int numAnimLayers = pAnimStack->GetMemberCount<fbx::FbxAnimLayer>();

	for (size_t layIdx = 0; layIdx < numAnimLayers; layIdx++)
	{
		//4.Retrieve the nth animation layer from the animation stack.
		fbx::FbxAnimLayer* lAnimLayer = pAnimStack->GetMember<fbx::FbxAnimLayer>(layIdx);
		//After retrieving an animation layer, you can access the animation curves for the properties of nodes and node attribute.
		int numCurves = lAnimLayer->GetMemberCount<fbx::FbxAnimCurveNode>();

		for (size_t curvIdx = 0; curvIdx < numCurves; curvIdx++)
		{
			//5.Retrieve the animation curves.Given a node(lNode) and an animation layer(lAnimLayer), you can retrieve the animation curves for node properties such as the local translation as shown in the following example.
			fbx::FbxAnimCurveNode* lAnimCurveNode = lAnimLayer->GetMember<fbx::FbxAnimCurveNode>(curvIdx);
				//lNode->LclTranslation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_X);

		}

	}




	// Destroy the sdk manager and all other objects it was handling.
	lSdkManager->Destroy();

	system("PAUSE");
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
