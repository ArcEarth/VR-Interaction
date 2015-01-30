#include "SkeletonClass.h"

using namespace std;

static const int jointReadArray[] = {
	0, 1, 1, 2, 2, 3,   //the body
	2, 8, 8, 9, 9, 11,  //the right arm
	2, 4, 4, 5, 5, 7,  //the left arm
	0, 16, 16, 17, 17, 19,  //the right leg
	0, 12, 12, 13, 13, 15   //the left leg
};	// TODO: why do this? Also, wrists and ankles are not used.

//static const int LeftArmReadArray[]={4,5,7};
//static const int RightArmReadArray[]={8,9,11};

#define HistoryCapacity 10

SkeletonClass::SkeletonClass(void)
{
	m_JointsCount = 0;
	m_JointsDefaultPosition = 0;
	m_TransformScale = 0.0f;
	m_TransformCutoff = 0.75f;
	m_LeftArmIndecis.clear();
	m_RightArmIndecis.clear();
	m_LeftArmIndecis.push_back(4);
	m_LeftArmIndecis.push_back(5);
	m_LeftArmIndecis.push_back(7);
	m_RightArmIndecis.push_back(8);
	m_RightArmIndecis.push_back(9);
	m_RightArmIndecis.push_back(11);
	Available = false;
	m_DefaultAvailable = false;
	TransformFlag = true;
}


SkeletonClass::~SkeletonClass(void)
{
	if (m_JointsDefaultPosition){
		delete [] m_JointsDefaultPosition;
		m_JointsDefaultPosition = 0;
	}
	if (m_JointsDisplacement){
		delete [] m_JointsDisplacement;
		m_JointsDisplacement = 0;
	}
}

//
//float DistancePointToSegment(const D3DXVECTOR3 &p,const D3DXVECTOR3 &s0,const D3DXVECTOR3 &s1){
//	D3DXVECTOR3 s=s1-s0;
//	D3DXVECTOR3 v=p-s0,u;
//	float Ps = D3DXVec3Dot(&v,&s);
//	if (Ps<0) 
//		//p-s0 is the shortest distance
//		return D3DXVec3Length(&v);
//
//	float Ds = D3DXVec3Dot(&s,&s);
//	if (Ps>Ds) 	
//	{
//		//p-s1 is the shortest distance
//		u = p-s1;
//		return D3DXVec3Length(&u);
//	}
//	//find the projection point on line segment U
//	u = s*Ps/Ds;
//	v -= u;
//	return D3DXVec3Length(&v);
//}


//Find the closest bone in space and return as the binding bone
SkeletonClass::HumanBoneEnum SkeletonClass::FindBindingBone(const D3DXVECTOR3 &p,const std::set<HumanBoneEnum> &ExceptionList)
{
	//We don't wanna to bind such a far bone, this is basicly means that we are just adding part according to the reletive position but not bone
	float minDis = 0.2f,dis;
	int Index = Illegal;
	for (int i = 0; i < Bones.size(); i++)
	{
		if (ExceptionList.find((HumanBoneEnum)i)==ExceptionList.end()) 
		{
			dis = FindDistancePointToSegment(p,Joints[Bones[i].first],Joints[Bones[i].second]);
			if (dis<minDis) 
			{
				minDis = dis;
				Index = i;
			}
		}
	}
	return (HumanBoneEnum)Index;
}

bool SkeletonClass::InverseTransformPointBindToBone(D3DXVECTOR3 &Point, HumanBoneEnum BindingBone)
{
	if (BindingBone == Illegal ) return true;
	D3DXVECTOR3 Displacement = Point - Joints[Bones[BindingBone].first];

	D3DXVECTOR3 BoneNow = Joints[Bones[BindingBone].second] - Joints[Bones[BindingBone].first];
	D3DXVECTOR3 BoneDefault = m_JointsDefaultPosition[Bones[BindingBone].second] - m_JointsDefaultPosition[Bones[BindingBone].first];

	//Get the rotate matrix form the direction bone-now to bone-default
	D3DXMATRIX TransformMatrix(GetRotateMatrixFromVectorToVector(BoneNow,BoneDefault));

	D3DXVec3TransformCoord(&Displacement,&Displacement,&TransformMatrix);

//	Point = Displacement;
	Point = Displacement + D3DXVECTOR3(m_JointsDefaultPosition[Bones[BindingBone].first]);
	return true;
}

D3DXMATRIX SkeletonClass::GetBindingBoneTransformMatrix(HumanBoneEnum BindingBone){
	D3DXMATRIX TranslationMatrix;
	if (BindingBone == Illegal ) {
		D3DXMatrixIdentity(&TranslationMatrix);
		return TranslationMatrix;
	}
	D3DXVECTOR3 BoneNow = Joints[Bones[BindingBone].second] - Joints[Bones[BindingBone].first];
	D3DXVECTOR3 BoneDefault = m_JointsDefaultPosition[Bones[BindingBone].second] - m_JointsDefaultPosition[Bones[BindingBone].first];

	D3DXMATRIX RotateMatrix(GetRotateMatrixFromVectorToVector(BoneDefault,BoneNow));
	//It's right version to if the input is relative displace about binding bone's first joint
	//D3DXVECTOR3 Displacement = Joints[Bones[BindingBone].first];
	//D3DXMatrixTranslation(&TranslationMatrix,Displacement.x,Displacement.y,Displacement.z);
	//D3DXMatrixMultiply(&TranslationMatrix,&RotateMatrix,&TranslationMatrix);

	// Transform = Minus defalt position -> Rotate form defalut to now -> Move to now position
	D3DXVECTOR3 Displacement = -m_JointsDefaultPosition[Bones[BindingBone].first];
	D3DXMatrixTranslation(&TranslationMatrix,Displacement.x,Displacement.y,Displacement.z);
	D3DXMatrixMultiply(&RotateMatrix,&TranslationMatrix,&RotateMatrix);
	Displacement = Joints[Bones[BindingBone].first];
	D3DXMatrixTranslation(&TranslationMatrix,Displacement.x,Displacement.y,Displacement.z);
	D3DXMatrixMultiply(&TranslationMatrix,&RotateMatrix,&TranslationMatrix);
	return TranslationMatrix;
}

bool SkeletonClass::Initialize(unsigned JointsCount,unsigned BonesCount,int * BonesDescription)
{
	m_JointsCount = JointsCount;
	m_BonesCount = BonesCount;
	m_JointsDefaultPosition = new D3DXVECTOR4[m_JointsCount];
	m_JointsDisplacement = new D3DXVECTOR4[m_JointsCount];
	Joints.resize(m_JointsCount);
	Bones.clear();
	for (int i = 0; i < BonesCount*2; i+=2)
	{
		Bones.push_back(std::pair<int,int>(BonesDescription[i],BonesDescription[i+1]));
	}
	return true;
}

inline D3DXVECTOR3 Cast2DXVector3(const Vector4 v){
	return D3DXVECTOR3(v.x,v.y,v.z);
}

inline D3DXVECTOR4 tranform(const Vector4 &v,const D3DXVECTOR3 &centre,const D3DXMATRIX &reverseRotateMatrix){
	D3DXVECTOR3 p = Cast2DXVector3(v);
	p.z -= 2.0f;
	p -= centre;
	D3DXVECTOR4 p2 = p;
	D3DXVec3Transform(&p2,&p,&reverseRotateMatrix);
	return p2;
}

inline D3DXQUATERNION Cast2DXQuaternion (const Vector4 &v){
	return D3DXQUATERNION(v.x,v.y,v.z,v.w);
}

bool SkeletonClass::Update(const NUI_SKELETON_DATA &SkeletonData){
//	m_IsUpdating = true;
//	Available = false;
	D3DXVECTOR3 r1,r2,r3,r4;

	r1 = Cast2DXVector3(SkeletonData.SkeletonPositions[4]);
	r2 = Cast2DXVector3(SkeletonData.SkeletonPositions[8]);

	r1 = r2 - r1;
	r1.y = 0;
	D3DXVec3Normalize(&r1,&r1);

	r2.x = 1.0f,r2.y = 0.0f,r2.z = 0.0f;
	D3DXVec3Cross(&r1,&r1,&r2);
	m_FacingAngle = -asinf(r1.y);

	//This is the "reverse" RotateMatrix
	D3DXMatrixRotationY(&m_FacingMatrix,-m_FacingAngle);

	// Update the skeleton point information
	m_Position = D3DXVECTOR3(SkeletonData.SkeletonPositions[0].x,SkeletonData.SkeletonPositions[0].y,SkeletonData.SkeletonPositions[0].z-2.0f);
	Joints[0] = D3DXVECTOR3(0.0f,0.0f,0.0f);
	for (int j = 1; j < m_JointsCount; j ++) 
	{
		// Determine if the left shoulder or the right shoulder should be the anchor point 
		D3DXVECTOR3 point = tranform(SkeletonData.SkeletonPositions[j],m_Position,m_FacingMatrix);

		// Update the j'th joint position , why reverse the z data?
//		point.z=(point.z - 2.0) * (-1);
//		point.z=point.z-2.0f;

		Joints[j] = point;
		//Update the displacement data for graphic parts
		if (m_DefaultAvailable) 
		{
			m_JointsDisplacement[j].x = (Joints[j].x - m_JointsDefaultPosition[j].x);
			m_JointsDisplacement[j].y = (Joints[j].y - m_JointsDefaultPosition[j].y);
			m_JointsDisplacement[j].z = (Joints[j].z - m_JointsDefaultPosition[j].z);
			m_JointsDisplacement[j].w = 1.0f;
		}
	}

	if (m_DefaultAvailable && TransformFlag){
//		float factor = Dot_Vec3(GetBoneVector(Radius_Left),GetBoneVector(Humerus_Left));
		float factor = Length_Vec3(Joints[HAND_LEFT] - Joints[SHOULDER_LEFT]) / GetArmLength(true) - m_TransformCutoff;
		if (factor > 0) {
			//Transform arm's length with the given factor
			TransformJoint(Joints[HAND_LEFT],Joints[SHOULDER_LEFT],factor,true);
			TransformJoint(Joints[ELBOW_LEFT],Joints[SHOULDER_LEFT],factor,true);
			m_JointsDisplacement[HAND_LEFT].x = (Joints[HAND_LEFT].x - m_JointsDefaultPosition[HAND_LEFT].x);
			m_JointsDisplacement[HAND_LEFT].y = (Joints[HAND_LEFT].y - m_JointsDefaultPosition[HAND_LEFT].y);
			m_JointsDisplacement[HAND_LEFT].z = (Joints[HAND_LEFT].z - m_JointsDefaultPosition[HAND_LEFT].z);
			m_JointsDisplacement[ELBOW_LEFT].x = (Joints[ELBOW_LEFT].x - m_JointsDefaultPosition[ELBOW_LEFT].x);
			m_JointsDisplacement[ELBOW_LEFT].y = (Joints[ELBOW_LEFT].y - m_JointsDefaultPosition[ELBOW_LEFT].y);
			m_JointsDisplacement[ELBOW_LEFT].z = (Joints[ELBOW_LEFT].z - m_JointsDefaultPosition[ELBOW_LEFT].z);
		}
		factor = Length_Vec3(Joints[HAND_RIGHT] - Joints[SHOULDER_RIGHT]) / GetArmLength(false) - m_TransformCutoff;
		if (factor > 0) {
			//Transform arm's length with the given factor
			TransformJoint(Joints[HAND_RIGHT],Joints[SHOULDER_RIGHT],factor,false);
			TransformJoint(Joints[ELBOW_RIGHT],Joints[SHOULDER_RIGHT],factor,false);
			m_JointsDisplacement[HAND_RIGHT].x = (Joints[HAND_RIGHT].x - m_JointsDefaultPosition[HAND_RIGHT].x);
			m_JointsDisplacement[HAND_RIGHT].y = (Joints[HAND_RIGHT].y - m_JointsDefaultPosition[HAND_RIGHT].y);
			m_JointsDisplacement[HAND_RIGHT].z = (Joints[HAND_RIGHT].z - m_JointsDefaultPosition[HAND_RIGHT].z);
			m_JointsDisplacement[ELBOW_RIGHT].x = (Joints[ELBOW_RIGHT].x - m_JointsDefaultPosition[ELBOW_RIGHT].x);
			m_JointsDisplacement[ELBOW_RIGHT].y = (Joints[ELBOW_RIGHT].y - m_JointsDefaultPosition[ELBOW_RIGHT].y);
			m_JointsDisplacement[ELBOW_RIGHT].z = (Joints[ELBOW_RIGHT].z - m_JointsDefaultPosition[ELBOW_RIGHT].z);
		}
	}

	m_SkeletonCentreHistory.push_back(m_Position);
	if (m_SkeletonCentreHistory.size()>HistoryCapacity) 
		m_SkeletonCentreHistory.pop_front();

	D3DXMatrixRotationY(&m_FacingMatrix,m_FacingAngle);
//	Available = true;
//	m_IsUpdating = false;
	return true;
}

bool SkeletonClass::SetJointsDefaultPosition(){
	for (int i=0;i<m_JointsCount;i++){
		m_JointsDefaultPosition[i]=D3DXVECTOR4(Joints[i].x,Joints[i].y,Joints[i].z,0.0f);
//		m_JointsDefaultPosition[i]-=m_JointsDefaultPosition[0];
		m_JointsDisplacement[i]=D3DXVECTOR4(0.0f,0.0f,0.0f,1.0f);
	}
	m_DefaultAvailable = true;
	return true;
}

bool SkeletonClass::BuildDeaulftSkeleton(){
	throw exception("No implement expception");
}

void SkeletonClass::ClearDefaultSkeleton(){
	if (!m_DefaultAvailable) return;
	for (int i=0;i<m_JointsCount;i++){
		m_JointsDefaultPosition[i]=D3DXVECTOR4(0.0f,0.0f,0.0f,0.0f);
		m_JointsDisplacement[i]=D3DXVECTOR4(0.0f,0.0f,0.0f,1.0f);
	}
	m_DefaultAvailable = false;
}

//A linear transform
bool SkeletonClass::TransformJoint(D3DXVECTOR3 &TransformPoint,const D3DXVECTOR3 &AnchorPoint){
	TransformPoint = (TransformPoint-AnchorPoint)*m_TransformScale + AnchorPoint;
	return true;
}

bool SkeletonClass::TransformJoint(D3DXVECTOR3 &TransformPoint,const D3DXVECTOR3 &AnchorPoint , const float Factor ,bool Isleft){
	float a = max(m_TransformScale/GetArmLength(Isleft) - 1,0);
	float F = 1 + a*Factor*Factor*16.0f;
	TransformPoint.x = (TransformPoint.x-AnchorPoint.x)*F + AnchorPoint.x;
	TransformPoint.z = (TransformPoint.z-AnchorPoint.z)*F + AnchorPoint.z;
	TransformPoint.y = (TransformPoint.y-AnchorPoint.y)*F + AnchorPoint.y;
	return true;
}

bool SkeletonClass::SetArmTransformScale(float scale){
	if (scale <= 0.0f) 
		return false;
	m_TransformScale = scale;
}

float SkeletonClass::GetBoneLength(HumanBoneEnum bone)
{
	D3DXVECTOR3 BoneVector = Joints[Bones[bone].second] - Joints[Bones[bone].first];
	return Length_Vec3(BoneVector);
}

D3DXVECTOR3 SkeletonClass::GetBoneVector(HumanBoneEnum bone) const
{
	return Joints[Bones[bone].second] - Joints[Bones[bone].first];
}

float SkeletonClass::GetArmLength(){
	float RadiusLength = 0.5 * (GetBoneLength(Radius_Left) + GetBoneLength(Radius_Right));
	float HumerusLength = 0.5 * (GetBoneLength(Humerus_Left) + GetBoneLength(Humerus_Right));
	return RadiusLength + HumerusLength;
}

float SkeletonClass::GetArmLength(bool IsLeft){
	if (IsLeft)
		return GetBoneLength(Humerus_Left) + GetBoneLength(Radius_Left);
	else
		return GetBoneLength(Humerus_Right) + GetBoneLength(Radius_Right);
}


bool SkeletonClass::TurnArmTransform(bool Switch){
	TransformFlag = Switch;
	return true;
}



const D3DXVECTOR4 *SkeletonClass::GetSkeletonDisplacement(){
	return m_JointsDisplacement;
}

const D3DXVECTOR4 *SkeletonClass::GetSkeletonDefaultPosition(){
	return m_JointsDefaultPosition;
}

const D3DXVECTOR3 SkeletonClass::GetBodyCentreDisplaceMent(){
	if (m_SkeletonCentreHistory.empty()) 
		return D3DXVECTOR3(0.0f,0.0f,0.0f);
//	while (m_IsUpdating) ;
//	m_IsUpdating = true;	
	return m_SkeletonCentreHistory.back() - m_SkeletonCentreHistory.front();
//	m_IsUpdating = false;	
}

const D3DXVECTOR3 SkeletonClass::GetPosition(){
	return m_Position;
}

const D3DXMATRIX &SkeletonClass::GetFacingMatrix(){
	return m_FacingMatrix;
}

const float &SkeletonClass::GetFacingAngle(){
	return m_FacingAngle;
}

const unsigned &SkeletonClass::GetJointsCount(){
	return m_JointsCount;
}

const unsigned &SkeletonClass::GetBonesCount(){
	return m_BonesCount;
}

const bool SkeletonClass::IsDefaultSkeletonAvailable() const{
	return m_DefaultAvailable;
}

const D3DXVECTOR3 SkeletonClass::GetJointPosition(HumanJointEnum Joint,bool IsAbsulote){
	//Sync the update
//	while (m_IsUpdating);
	if (IsAbsulote) {
		D3DXVECTOR4 p;
		D3DXVec3Transform(&p,&(Joints[Joint]),&m_FacingMatrix);
		return D3DXVECTOR3(p) + m_Position;
	} else {
		return Joints[Joint];
	}
}