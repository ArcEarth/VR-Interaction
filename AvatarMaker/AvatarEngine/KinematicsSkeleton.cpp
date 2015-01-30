#include "stdafx.h"
#include "KinematicsSkeleton.h"
#include "DXMathExtend.h"
#include <stack>

using namespace DirectX;
using namespace std;

namespace Kinematics
{
	ISerializable::Blob IndexedSkeleton::Serialize() const
	{
		auto size = Index.size();
		Blob blob;
		blob.size = sizeof(size_t) + (sizeof(KinematicsData[2]) + sizeof(unsigned int)) * size;
		blob.pData = (unsigned char*)malloc(blob.size);
		if (blob.pData == nullptr) return std::move(blob);
		*reinterpret_cast<size_t*>(blob.pData) = size;
		KinematicsData (*Entities)[2] = reinterpret_cast<KinematicsData(*)[2]>(blob.pData + sizeof(size_t));
		unsigned int *Parents = reinterpret_cast<unsigned int*>(blob.pData + sizeof(size_t) + sizeof(KinematicsData[2]) * size);

		map<unsigned int,unsigned int> mapper;
		unsigned int key = 0; 
		for (auto& idx : Index)
		{
			memcpy(Entities[key],idx.second->Entity,sizeof(KinematicsData[2]));
			mapper[idx.first] = key++;
		}
		key = 0;
		for (auto& idx : Index)
		{
			if (!idx.second->Parent)
			{
				Parents[key] = key;
				++key;
			} else
				Parents[key++] = mapper[idx.second->Parent->ID];
		}

		return blob;
	}

	void IndexedSkeleton::Deserialize(const Blob& blob)
	{
		size_t size = *reinterpret_cast<size_t*>(blob.pData);
		KinematicsData (*Entities)[2] = reinterpret_cast<KinematicsData(*)[2]>(blob.pData + sizeof(size_t));
		unsigned int *Parents = reinterpret_cast<unsigned int*>(blob.pData + sizeof(size_t) + sizeof(KinematicsData[2]) * size);
		Construct(size,Entities,Parents);
	}


	IndexedSkeleton::IndexedSkeleton(){
		_root = nullptr;
	}
	IndexedSkeleton::~IndexedSkeleton(){
		for (auto& idx : Index)
		{
			if (idx.second)
			{
				delete idx.second;
				idx.second = nullptr;
			}
		}
	}
	IndexedSkeleton::IndexedSkeleton(const IndexedSkeleton& rhs)
	{
		if (rhs._root)
			_root = new Joint(*rhs._root);
		else
		{
			_root = nullptr;
			return;
		}
		for_all([&](Joint* pJoint){
			Index.insert(Index_Item(pJoint->ID,pJoint));
		});
	}
	IndexedSkeleton::IndexedSkeleton(IndexedSkeleton&& rhs)
	{
		_root = nullptr;
		*this = std::move(rhs);
	}

	IndexedSkeleton& IndexedSkeleton::operator = (const IndexedSkeleton& rhs)
	{
		clear();
		_root = new Joint(*rhs._root);
		Index.clear();
		for_all([&](Joint* pJoint){
			Index.insert(Index_Item(pJoint->ID,pJoint));
		});
		return *this;
	}
	IndexedSkeleton& IndexedSkeleton::operator = (IndexedSkeleton&& rhs)
	{
		clear();
		_root = rhs._root;
		rhs._root = nullptr;
		Index = std::move(rhs.Index);
		rhs.Index.clear();
		return *this;
	}

	void IndexedSkeleton::clear(){
		if (_root)
			delete _root;
		_root = nullptr;
		Index.clear();
	}

	void IndexedSkeleton::Construct(const size_t JointCount , KinematicsData pEntity[][2]  ,const unsigned int Parents[]){
		clear();
		auto Joints = new Joint*[JointCount];
		for (unsigned int i = 0; i < JointCount; i++)
		{
			Joints[i] = new Joint;
		}
		for (unsigned int i = 0; i < JointCount; i++)
		{
			memcpy(Joints[i]->Entity,pEntity[i],sizeof(KinematicsData[2]));
			Joints[i]->ID = i;
			Joints[i]->Radius = 0.05f;
			Index.insert(make_pair(i,Joints[i]));
			if (Parents[i] != i)
			{
				Joints[Parents[i]]->Children_Append(Joints[i]);
			} else
			{
				_root = Joints[i];
			}
		}
	}

	void IndexedSkeleton::Construct(const int JointCount ,const int Connections[]){
		clear();
		std::vector<Joint*> Joints(JointCount);
		for (int i = 0; i < JointCount; i++)
		{
			Joints[i] = new Joint();
			Joints[i]->ID = i;
			Joints[i]->Radius = 0.05f;
			Index.insert(Index_Item(i,Joints[i]));
		}
		for (int i = 0; i < JointCount-1; i++)
		{
			Joints[Connections[i*2]]->Children_Append(Joints[Connections[i*2+1]]);
		}

		int k;
		for (k = 0; k < JointCount && !Joints[k]->is_root(); k++);
		_root = Joints[k];
	}

	// Return value , the index map form src joints to dst joints
	std::unique_ptr<std::map<unsigned int,unsigned int>> IndexedSkeleton::AppendSubSkeleton(Kinematics::Joint* pTargetJoint,const Kinematics::Joint* pSrcSubSkeleton,bool IsRelativeCoordinate)
	{
		if (pSrcSubSkeleton==nullptr)
			return nullptr;

		if (pTargetJoint!=nullptr && !this->containts(pTargetJoint))
		{
			cout<<"Error : pTargetJoint don't belong to the skeleton."<<endl;
			return nullptr;
		}

		std::unique_ptr<std::map<unsigned int,unsigned int>> mapper(new map<unsigned int,unsigned int>());
		unsigned int Key = 0;

		// Copy the sub-skeleton
		Joint* pRoot = new Joint(*pSrcSubSkeleton);

		if (this->empty())
		{
			this->_root = pRoot;
		}

		// If don't give the append target , somehow, find one instead
		bool DefaultInvalidFlag = false;
		if (!pTargetJoint) 
		{
			cout<<"Warning : Don't get a specific binding bone , find it instead."<<endl;
			// We can't give a relative coordinate if we don't know it , right?
			IsRelativeCoordinate = false;
			pTargetJoint = this->FindClosestJoint(pRoot->Entity[Current].Position,Current);
			DefaultInvalidFlag = true;
		}

		pTargetJoint->Children_Add(pRoot);

		pRoot->for_all_in_sub_skeleton([&](Joint* pJoint){
			// Fix the ID
			while (containtsKey(Key))
				Key++;
			mapper->insert(pair<unsigned int,unsigned int>(pJoint->ID,Key));
			pJoint->ID = Key;
			this->Index.insert(Kinematics::IndexedSkeleton::Index_Item(Key,pJoint));

			// Deduce the hierarchical data if needed
			if (!IsRelativeCoordinate){
				pJoint->Deduce_Hierarchical_from_Global(Current);
				// If default skeleton is invalid , so , use current instead.
				if (!DefaultInvalidFlag){
					pJoint->Deduce_Hierarchical_from_Global(Default);
				}	else	{
					pJoint->Snap_Default_to_Current();
				}


			}
		});

		pTargetJoint->ReBuildSubSkeleton_Global_from_Hierarchical(Current);
		pTargetJoint->ReBuildSubSkeleton_Global_from_Hierarchical(Default);
		return mapper;
	}

	Joint* IndexedSkeleton::FindClosestBone( DirectX::FXMVECTOR point , State state , float MaxiumBindDistance )
	{
		float MinDis = DirectX::g_INFINITY;
		Kinematics::Joint* Bone = nullptr;
		this->Root->for_all_descendants([&](Kinematics::Joint* itr){
			float Dis = DirectX::FindDistancePointToSegment(point,itr->Parent->Entity[state].Position,itr->Entity[state].Position);
			if (Dis < MinDis){
				MinDis = Dis;
				Bone = itr;
			}
		});
		if (MinDis > MaxiumBindDistance) Bone = this->Root;
		return Bone;
	}

	Joint* Kinematics::IndexedSkeleton::FindClosestJoint( DirectX::FXMVECTOR point , State state , float MaxiumBindDistance )
	{
		float MinDis = DirectX::g_INFINITY;
		Kinematics::Joint* pJoint = nullptr;
		this->Root->for_all_descendants([&](Kinematics::Joint* itr){
			float Dis = DirectX::Vector3::Distance(point,itr->Entity[state].Position);
			if (Dis < MinDis){
				MinDis = Dis;
				pJoint = itr;
			}
		});
		if (MinDis > MaxiumBindDistance) pJoint = this->Root;
		return pJoint;
	}


	IndexedSkeleton::IndexedSkeleton(const int JointCount ,const int Connections[])
	{
		_root = nullptr;
		Construct(JointCount,Connections);
	}

	void IndexedSkeleton::Snap_Default_to_Current()
	{
		for_all([](Joint* pJoint)->void{
			pJoint->Snap_Default_to_Current();
		});

	}
	void IndexedSkeleton::Snap_Current_to_Default()
	{
		for_all([](Joint* pJoint)->void{
			pJoint->Snap_Current_to_Default();
		});
	}

	void IndexedSkeleton::Update()
	{
		if (_root)
			_root->ReBuildSubSkeleton_Global_from_Hierarchical(Current);
	}

	void IndexedSkeleton::RemoveJoint( unsigned int jointID )
	{
		RemoveJoint((*this)[jointID]);
	}

	void IndexedSkeleton::RemoveJoint( Joint* pJoint )
	{
		if (!containts(pJoint)) return;
		pJoint->for_all_in_sub_skeleton([&](Joint* itr)
		{
			Index.erase(itr->ID);
		});
		if (pJoint->IsRoot)
			_root = nullptr;
		delete pJoint;
	}

	void IndexedSkeleton::SelectNewRoot( Joint* pNewRoot)
	{
		auto pJoint = pNewRoot;
		std::stack<Joint*> Path;
		while (!pJoint->IsRoot)
		{
			Path.push(pJoint);
			pJoint = pJoint->Parent;
			//pJoint->Deduce_Hierarchical_from_Global(Current,Path.top());
			//pJoint->Deduce_Hierarchical_from_Global(Default,Path.top());
		}
		while (!Path.empty())
		{
			pJoint = Path.top();
			Path.pop();
			auto pParent = pJoint->Parent;
			pParent->children.remove(pJoint);
			pParent->parent = pJoint;
			pJoint->children.push_back(pParent);
			//pParent->Deduce_Hierarchical_from_Global(Current);
			//pParent->Deduce_Hierarchical_from_Global(Default);
		}
		pNewRoot->parent = nullptr;
		_root = pNewRoot;
		//Quaternion invDefaultOrientaion = _root->Entity[Default].Orientation.Inverse();
		XMVECTOR qInvDefault = XMQuaternionInverse(_root->Entity[Default].Orientation);
		_root->Entity[Current].Orientation = XMQuaternionMultiply(qInvDefault,_root->Entity[Current].Orientation);
		_root->Entity[Default].Orientation = XMQuaternionIdentity();
		_root->Entity[Current].Rotation = _root->Entity[Current].Orientation;
		_root->Entity[Default].Rotation = _root->Entity[Default].Orientation;

		_root->for_all_descendants([](Joint* pJoint){
			pJoint->Deduce_Hierarchical_from_Global(Current);
			pJoint->Deduce_Hierarchical_from_Global(Default);
		});
		this->Update();
	}


	std::map<unsigned int,unsigned int> IndexedSkeleton::OptimizeIndex()
	{
		std::map<unsigned int,unsigned int> mapper;
		int key = 0;
		for (auto itr = this->Index.begin(); itr != this->Index.end(); itr++ , key++)
		{
			itr->second->ID = key;
			mapper[key] = itr->first;
			//mapper.insert(pair<unsigned int,unsigned int>(key,itr->first));
		}
		Index.clear();
		for_all([&](Joint* pJoint)
		{
			assert(Index.find(pJoint->ID) == Index.end());
			Index[pJoint->ID] = pJoint;
		});
		return mapper;
	}


	Kinematics::Joint::Joint() 
		: parent(nullptr)
		, children()
		, Radius(0.05f)
		//,Position(Entity[Current].Position)
		//, Orientation(Entity[Current].Orientation)
		//, Rotation(Entity[Current].Rotation)
		//, Length(Entity[Current].Length)
	{
	}

	Kinematics::Joint::Joint(const Joint& rhs) 
		: parent(nullptr)
		, children()
		//,Position(Entity[Current].Position)
		//, Orientation(Entity[Current].Orientation)
		//, Rotation(Entity[Current].Rotation)
		//, Length(Entity[Current].Length)
	{
		*this = rhs;
	}

	void Kinematics::Joint::Children_Add( Joint* pJoint )
	{
		pJoint->parent = this;
		children.push_front(pJoint);
	}

	void Kinematics::Joint::Children_Append( Joint* pJoint )
	{
		pJoint->parent = this;
		children.push_back(pJoint);
	}

	void Kinematics::Joint::Children_Remove( Joint* pJoint )
	{
		pJoint->Isolate();
	}

	void Kinematics::Joint::Isolate()
	{
		if (parent) {
			parent->children.remove(this);
			this->parent = nullptr;
		}
		for (auto& pJoint : children)
		{
			pJoint->parent = nullptr;
		}
	}

	bool Kinematics::Joint::is_root() const
	{
		return !parent;
	}

	void Kinematics::Joint::Deduce_Global_from_Hierarchical( State state , Joint* pSpecificSource)
	{
		Joint* pSrc = pSpecificSource ? pSpecificSource : Parent;
		if (!pSrc) {
			//Entity[state].Orientation = Entity[state].Rotation;
			//Entity[state].Position = Vector3(0.0f,Entity[state].Length,0.0f);
		} else
		{
			//Entity[state].Orientation = pSrc->Entity[state].Orientation * Entity[state].Rotation;
			Entity[state].Orientation = XMQuaternionMultiply(Entity[state].Rotation,pSrc->Entity[state].Orientation);
			XMVECTOR RelativeVector = Vector3::Rotate(g_XMIdentityR1 * Entity[state].Scale,(XMVECTOR)Entity[state].Orientation);
			//XMVECTOR RelativeVector = Vector3::Rotate(Vector3(0.0f,Entity[state].Length,0.0f),(XMVECTOR)Entity[state].Orientation);
			Entity[state].Position = pSrc->Entity[state].Position+ RelativeVector;
		}
	}

	Joint& Kinematics::Joint::operator=( const Joint & rhs )
	{
		memcpy(Entity,rhs.Entity,sizeof(Entity));
		ID = rhs.ID;
		Radius = rhs.Radius;
		children.clear();
		for (auto itr : rhs.Children)
		{
			Joint* pJoint = new Joint(*itr);
			Children_Append(pJoint);
		}
		return *this;
	}

	DirectX::XMMATRIX Kinematics::Joint::BlendMatrix() const
	{
		//DirectX::XMVECTOR RotateQuaternion = DirectX::XMQuaternionMultiply(Entity[Default].Orientation.Inverse(),Entity[Current].Orientation);
		////DirectX::XMVECTOR Scale = DirectX::XMVectorSet(1.0f,Entity[Current].Length/Entity[Default].Length,1.0f,1.0f);
		DirectX::XMVECTOR vScale = Entity[Current].Scale / Entity[Default].Scale;
		////DirectX::XMVECTOR Scale = g_XMOne;
		//return DirectX::XMMatrixAffineTransformation(vScale,Entity[Default].Position,RotateQuaternion,Entity[Current].Position-Entity[Default].Position);

		
		XMMATRIX Transform = XMMatrixTranslationFromVector(-Entity[Default].Position); // Inverse translate to origin
		XMVECTOR invRotaion = XMQuaternionInverse(Entity[Default].Orientation);
		Transform *= XMMatrixRotationQuaternion(invRotaion); // Inverse Rotation
		//Transform *= XMMatrixScalingFromVector(vScale); // Scaling
		Transform *= XMMatrixRotationQuaternion(Entity[Current].Orientation); // Rotation
		Transform *= XMMatrixTranslationFromVector(Entity[Current].Position); // Translate to new origin
		return Transform;
	}

	DirectX::XMDUALVECTOR Kinematics::Joint::TransformDualQuaternion() const
	{
		XMVECTOR Rotaion = XMQuaternionInverse(Entity[Default].Orientation);
		Rotaion = XMQuaternionMultiply(Rotaion,Entity[Current].Orientation);
		XMVECTOR Translation = Entity[Current].Position;
		XMVECTOR Origin = Entity[Default].Position;
		Translation -= Origin;
		XMDUALVECTOR dqRes = XMDualQuaternionRigidTransform(Origin,Rotaion,Translation);
		return dqRes;
	}
	DirectX::XMDUALVECTOR Kinematics::Joint::InverseTransformDualQuaternion() const
	{
		XMVECTOR Rotaion = XMQuaternionInverse(Entity[Current].Orientation);
		Rotaion = XMQuaternionMultiply(Rotaion,Entity[Default].Orientation);
		XMVECTOR Translation = Entity[Default].Position;
		XMVECTOR Origin = Entity[Current].Position;
		Translation -= Origin;
		XMDUALVECTOR dqRes = XMDualQuaternionRigidTransform(Origin,Rotaion,Translation);
		return dqRes;
	}

	DirectX::XMMATRIX Kinematics::Joint::InverseBlendMatrix() const
	{
		//DirectX::XMVECTOR RotateQuaternion = DirectX::XMQuaternionMultiply(Entity[Current].Orientation.Inverse(),Entity[Default].Orientation);
		////DirectX::XMVECTOR Scale = DirectX::XMVectorSet(1.0f,Entity[Default].Length/Entity[Current].Length,1.0f,1.0f);
		DirectX::XMVECTOR vScale = Entity[Default].Scale / Entity[Current].Scale;
		////DirectX::XMVECTOR vScale = g_XMOne;
		//return DirectX::XMMatrixAffineTransformation(vScale,Entity[Current].Position,RotateQuaternion,Entity[Default].Position-Entity[Current].Position);

		XMMATRIX Transform = XMMatrixTranslationFromVector(-Entity[Current].Position); // Inverse translate to origin
		XMVECTOR invRotaion = XMQuaternionInverse(Entity[Current].Orientation);
		Transform *= XMMatrixRotationQuaternion(invRotaion); // Inverse Rotation
		//Transform *= XMMatrixScalingFromVector(vScale); // Scaling
		Transform *= XMMatrixRotationQuaternion(Entity[Default].Orientation); // Rotation
		Transform *= XMMatrixTranslationFromVector(Entity[Default].Position); // Translate to new origin
		return Transform;
	}

	void Kinematics::Joint::Snap_Default_to_Current()
	{
		Entity[Default] = Entity[Current];
	}

	void Kinematics::Joint::Snap_Current_to_Default()
	{
		Entity[Current] = Entity[Default];
	}

	void Kinematics::Joint::ReBuildSubSkeleton_Global_from_Hierarchical( State state , Joint* pSpecificSource /*= nullptr*/ )
	{
		this->Deduce_Global_from_Hierarchical(state,pSpecificSource);
		for_all_in_sub_skeleton([=](Joint* pJoint)->void{
			if (pJoint!=this)
				pJoint->Deduce_Global_from_Hierarchical(state,nullptr);
		});
	}

	/// <summary>
	/// Perform A IK compute with the constraint of No-Yaw Rotation(Self-Rotation)
	/// </summary>
	/// <param name="state">The state.</param>
	/// <param name="pSpecificSource">The specific parent joint for IK compute.</param>
	void Kinematics::Joint::Deduce_Hierarchical_from_Global( State state , Joint* pSpecificSource)
	{
		Joint* pSrc = pSpecificSource ? pSpecificSource : Parent;
		if (!pSrc) {
			std::cout << "Warning : IK deduction without source" <<std::endl;
			XMVECTOR v0 = Entity[state].Position;
			Entity[state].Orientation = XMQuaternionRotationVectorToVector(g_XMIdentityR1,v0);
			Entity[state].Rotation = Entity[state].Orientation;
			//Entity[state].Length = Vector3::Length(v0);
			Entity[state].Scale.Set(1.0f,Vector3::Length(v0),1.0f);
		}	else	{
			XMVECTOR v0 = (XMVECTOR)(Entity[state].Position - pSrc->Entity[state].Position);
			v0 = DirectX::XMVector3InverseRotate(v0 , pSrc->Entity[state].Orientation);

			Entity[state].Scale.Set(1.0f,Vector3::Length(v0),1.0f);

			// with Constraint No-Yaw
			v0 = DirectX::XMVector3Normalize(v0);
			XMFLOAT4A Sp;
			DirectX::XMStoreFloat4A(&Sp,v0);
			float Roll = -std::asinf(Sp.x);
			float Pitch = std::atan2f(Sp.z,Sp.y);
			Entity[state].Rotation = XMQuaternionRotationRollPitchYaw(Pitch,0.0f,Roll);
			//XMVECTOR vRot = XMVector3Rotate(g_XMIdentityR1,Entity[state].Rotation);

			//Entity[state].Rotation = Quaternion::RotationQuaternion(g_XMIdentityR1,v0);

			Entity[state].Orientation = XMQuaternionMultiply(Entity[state].Rotation,pSrc->Entity[state].Orientation);
			//Entity[state].Orientation = Quaternion::RotationQuaternion(g_XMIdentityR1,v0);
			//Entity[state].Rotation = XMQuaternionMultiply(Entity[state].Orientation , pSrc->Entity[state].Orientation.Inverse());
			//Entity[state].Length = Vector3::Length(v0);
		}	
	}

	Kinematics::Joint::~Joint()
	{
		Isolate();
	}


	DirectX::XMVECTOR Joint::BlendPoint( DirectX::FXMVECTOR Point) const
	{
		XMVECTOR p = Point - Entity[Default].Position;
		p = XMVector3InverseRotate(p,Entity[Default].Orientation);
		p = XMVector3Rotate(p,Entity[Current].Orientation);
		p = p + Entity[Current].Position;
		return p;
	}

	DirectX::XMVECTOR Joint::InverseBlendPoint( DirectX::FXMVECTOR Point) const
	{
		XMVECTOR p = Point - Entity[Current].Position;
		p = XMVector3InverseRotate(p,Entity[Current].Orientation);
		p = XMVector3Rotate(p,Entity[Default].Orientation);
		p = p + Entity[Default].Position;
		return p;
	}

}