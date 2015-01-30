#ifndef FOWARD_KINEMATICS_SKELETON_H
#define FOWARD_KINEMATICS_SKELETON_H
#pragma once

// The structure descript a bone's local data (hierarchical orientation & length).
// The structure descript a joint and bone 's local data (hierarchical orientation & length).
// The local coordinate's origin is the endjoint of the bone
// Using "Displacement" to acess the Displacement from start joint to end joint
// This data should always be like (Length,0,0)

// The structure provides a set of method for access the hierarchical structure of each joint


// The bone will be descripte by it's end joint
//
//class FKSkeleton
//{
//public:
//	FKSkeleton(void);
//	FKSkeleton(int JointCount);
//	FKSkeleton(int JointCount , const std::vector<std::pair<int,int>> & JointConnections);
//	FKSkeleton(int JointCount , const std::vector<int> & Connections);
//	FKSkeleton(const int JointCount ,const int BoneCount ,const int Connections[]);
//	virtual ~FKSkeleton(void);
//
//	const GeoJoint* Root() const {return m_Root;}
//	const std::vector<GeoJoint> &Joints() const {return m_Joints;}
//	std::vector<GeoJoint> &Joints() {return m_Joints;}
//
//	// Using this method every time after you update the local information
//	void Update(DirectX::RigidObject *BaseCoordinate);
//
//protected: 
//
//	// To-do : make this method only work on the part wich changes happend
//	void RefreshGlobalState(GeoJoint* joint);
//
//protected:
//	GeoJoint* m_Root;
//	std::vector<GeoJoint> m_Joints;
//};
#endif // !FOWARD_KINEMATICS_SKELETON_H
