#include <iostream>
#include <DirectXMath.h>
#include "..\Causality\Common\DirectXMathExtend.h"

using namespace DirectX;
using namespace std;
int main()
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

	XMVECTOR Sq3 = XMQuaternionMultiply(XMQuaternionConjugate(Q2),Q1);
	Quaternion sq = Sq3;
	cout << "Log interpolate : " << lq << endl;
	cout << "Sphere interpolate : " << sq << endl;

	system("PAUSE");
}