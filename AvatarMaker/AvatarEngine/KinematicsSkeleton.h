#pragma once
#ifndef KINEMATICS_SKELETON_H
#define KINEMATICS_SKELETON_H
#include "MathHelper.h"
#include "Serialization.h"
#include <list>
#include <vector>
//#include "tree.h"
#include <map>
#include <memory>

namespace Kinematics{

	class IndexedSkeleton;

	enum State
	{
		Default = 0,
		Current = 1,
	};

	struct KinematicsData
	{
		DirectX::Quaternion Orientation;
		DirectX::Quaternion Rotation;
		DirectX::Vector3 Position;
		DirectX::Vector3 Scale;

		KinematicsData()
		{
			ZeroMemory(this,sizeof(KinematicsData));
			Scale.Set(1.0f,1.0f,1.0f);
		}
	};

	class Joint
	{
		friend IndexedSkeleton;
	public:
		enum KinematicsType
		{
			Passive,
			Active,
		};
	public:

		// Perform a simple DFS for lazy coding
		template <typename _Func>
		void for_all_in_sub_skeleton(_Func Func)
		{
			Func(this);
			for (auto itr : children)
			{
				itr->for_all_in_sub_skeleton(Func);
			}
		}
		template <typename _Func>
		void for_all_in_sub_skeleton(_Func Func) const
		{
			Func(this);
			for (const Joint* itr : children)
			{
				itr->for_all_in_sub_skeleton(Func);
			}
		}
		// Perform a simple DFS for lazy coding
		template <typename _Func>
		void for_all_descendants(_Func Func)
		{
			for (auto itr : children)
			{
				Func(itr);
				itr->for_all_descendants(Func);
			}
		}
		template <typename _Func>
		void for_all_descendants(_Func Func) const
		{
			for (const Joint* itr : children)
			{
				Func(itr);
				itr->for_all_descendants(Func);
			}
		}

		explicit Joint();
		explicit Joint(const Joint& rhs);
		~Joint();

		__declspec(property(get = getParent) )
			Joint* const Parent;

		__declspec(property(get = getChildren)) 
		const std::list<Joint*>&  Children;

		__declspec(property(get = is_root)) 
			bool  IsRoot;

		Joint* const getParent() const { return parent; }
		const std::list<Joint*>& getChildren() const { return children; }


		void Children_Add(Joint* child);
		void Children_Append(Joint* child);
		void Children_Remove(Joint* child);

		// Separate this joint from it's parent
		void Separate();
		// Ioslate this joint form it's parent and it's children
		void Isolate();
		bool is_root() const;

		Joint& operator= (const Joint & rhs);

		////Access Helpers for Current Data
		//DirectX::Vector3& Position;
		//DirectX::Quaternion& Orientation;
		//float& Length;
		//DirectX::Quaternion& Rotation;

		void Deduce_Global_from_Hierarchical(State state , Joint* pSpecificSource = nullptr);
		void Deduce_Hierarchical_from_Global(State state , Joint* pSpecificSource = nullptr);
		//************************************
		// Method:    BuildSubSkeleton
		// FullName:  Kinematics::Joint::BuildSubSkeleton
		// Access:    public 
		// Returns:   void
		// Parameter: State state
		// Parameter: Joint * pSpecificSource
		// Usage:     Re-build the global data for the whole sub skeleton with it's local data
		//************************************
		void ReBuildSubSkeleton_Global_from_Hierarchical(State state , Joint* pSpecificSource = nullptr);

		//void BuildGlobalDataFromHierarchicalData_Recursive(State state);
		//void BuildHierarchicalDataFromGlobalData_Recursive(State state);

		void Snap_Default_to_Current();
		void Snap_Current_to_Default();

		// return the Matrix used for skinning animation
		// Transform a vertex in the coordinate of Default => Current
		DirectX::XMMATRIX BlendMatrix() const;
		DirectX::XMMATRIX InverseBlendMatrix() const;
		DirectX::XMDUALVECTOR TransformDualQuaternion() const;
		DirectX::XMDUALVECTOR InverseTransformDualQuaternion() const;

		DirectX::XMVECTOR BlendPoint(DirectX::FXMVECTOR Point) const;
		DirectX::XMVECTOR InverseBlendPoint(DirectX::FXMVECTOR Point) const;

	public:
		unsigned int ID;
		float Radius;
		KinematicsData Entity[2];
	protected:
		/************************************************************************/
		/* Tree Structure														*/
		/************************************************************************/
		Joint* parent;
		std::list<Joint*> children;
		//Joint* _child;
		//Joint* _brother;
	};

	struct Chain
	{
		Chain(Joint* Begin,Joint* End)
		{
			Joint* pJoint = End;
			while (pJoint && pJoint!=Begin)
			{
				m_JointList.push_front(pJoint);
				pJoint=pJoint->Parent;
			}
			if (pJoint == Begin)
			{
				m_JointList.push_front(Begin);
			} else
			{
				throw std::runtime_error("Begin Joint and End Joint is not on a kinematics chain.");
			}
		}

		//template <typename Itr>
		//Chain(Itr begin_itr , Itr end_itr)
		//{

		//}


		const std::list<Joint*> JointList() const;


	private:
		std::list<Joint*> m_JointList;
	};

	//class KinematicsSkeleton
	//{
	//public:
	//	typedef std::vector<KinematicsData> frame;
	//	typedef uint16_t					joint_descriptor;

	//	boost::adjacency_list<>				connections;
	//	std::vector<frame>					frames;
	//};


	class IndexedSkeleton
		: public ISerializable
	{
	public:
		virtual Blob Serialize() const;
		virtual void Deserialize(const Blob& Data); 

		IndexedSkeleton();
		IndexedSkeleton(const IndexedSkeleton& rhs);
		IndexedSkeleton(IndexedSkeleton&& rhs);
		~IndexedSkeleton();
		IndexedSkeleton(const int JointCount ,const int Connections[]);
		void Construct(const int JointCount ,const int Connections[]);
		void Construct(const size_t JointCount , KinematicsData pData[][2] , const unsigned int Parents[]);

		void clear();
		bool empty() const {return !_root;}
		int size() const {return Index.size();}
		bool containts(const Joint* pJoint) const{
			if (!pJoint) return false;
			auto itr = this->Index.find(pJoint->ID);
			return (itr!=Index.end()) && (itr->second==pJoint);
		}
		bool containtsKey(unsigned int jointID) const{
			return (Index.find(jointID)!=Index.end());
		}

		Joint* at(unsigned int index){
			auto itr = Index.find(index);
			if (itr!=Index.end()) 
				return itr->second;
			else
				return nullptr;
		}
		const Joint* at(unsigned int index) const{
			auto itr = Index.find(index);
			if (itr!=Index.cend()) 
				return itr->second;
			else
				return nullptr;
		}

		Joint* operator[](unsigned int index){
			return at(index);
		}
		const Joint* operator[](unsigned int index) const{
			return at(index);
		}


		void RemoveJoint(unsigned int jointID);
		void RemoveJoint(Joint* pJoint);

		std::unique_ptr<std::map<unsigned int,unsigned int>> AppendSubSkeleton(Kinematics::Joint* pTargetJoint,const Kinematics::Joint* pSrcSubSkeleton,bool IsRelativeCoordinate);
		// Anyway , lets just return the value since we have Rvalue && move sementic now
		std::map<unsigned int,unsigned int> OptimizeIndex();

		IndexedSkeleton& operator= (const IndexedSkeleton& rhs);
		IndexedSkeleton& operator= (IndexedSkeleton&& rhs);

		// Rebuild Skeleton
		void Snap_Default_to_Current();
		// Reset Skeleton 
		void Snap_Current_to_Default();

		void Update();

		const Joint* getRoot() const { return _root; }
		Joint* getRoot() {return _root;}

		__declspec(property(get = getRoot)) 
		Joint* Root;

		void SelectNewRoot(Joint* pNewRoot);

		template <typename _Func>
		void for_all(_Func Func){
			if (!_root) return;
			_root->for_all_in_sub_skeleton(Func);
		}
		template <typename _Func>
		void for_all(_Func Func) const{
			if (!_root) return;
			_root->for_all_in_sub_skeleton(Func);
		}

		// the return value represent the end joint of the bone
		// When the distance still greater than the threhold , it returns the root joint instead
		Joint* FindClosestBone( DirectX::FXMVECTOR point , State state , float MaxiumBindDistance = 1.0f );
		Joint* FindClosestJoint( DirectX::FXMVECTOR point , State state , float MaxiumBindDistance = 1.0f  );
	public:
		std::map<unsigned int,Joint*> Index;
		typedef std::pair<unsigned int,Joint*> Index_Item;
	protected:
		Joint* _root;

		//std::map<unsigned int,Joint*> Index;
		//typedef std::pair<unsigned int,Joint*> Index_Item;
	};

}
#endif //!KINEMATICS_SKELETON_H