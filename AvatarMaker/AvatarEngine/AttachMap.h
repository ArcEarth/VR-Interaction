#pragma once
#include "KinematicsSkeleton.h"
#include "ModelInterface.h"
#include "Carmera.h"
#include "MetaBallModel.h"
#include "MotionAdaptor.h"
#include "DynamicMetaBallModel.h"
#include <map>
//#include <boost\bimap.hpp>

namespace Kinematics
{
	class IMotionSource
	{
		virtual void Drive(Joint* pJoint) = 0;
	};
	namespace JointsBinding
	{
		static unsigned int NullJoint = -1;

		class BindingMap
		{
		public:
			typedef unsigned int IndexType;

			enum BindingType
			{
				Binding_Null,
				Binding_Rotation = 0x0,
				Binding_Orientation = 0x1,
				Binding_SR_ON = 0x2,
				Binding_SR_OFF = 0x0,
				Binding_Displacement,
				Binding_Position,
				Binding_Scale_ON,
				Binding_Scale_OFF,
			};

			struct Binding
			{
				IndexType TargetID;	// The Driven joint ID
				IndexType SourceID;	// The Driving joint ID
				BindingType Type;
			};

		public:
			void insert(IndexType SourceID , IndexType TargetID , BindingType Type);
			void earse(IndexType SourceID);
			const Binding& GetbySourceID(IndexType SourceID);
			const Binding& GetbyTargetID(IndexType TargetID);

		protected:
			std::map<IndexType,Binding> m_Dictionary;
		};
	}

	class AttachMap
	{
	public:
		AttachMap(ID3D11Device *pDevice , ID3D11DeviceContext *pContext ,const ICamera* pCamera);
		AttachMap();
		~AttachMap(void);

		bool IsAttached() const{
			return !Connections.empty();
		}

		const IndexedSkeleton* Source() const
		{
			return pSource;
		}

		const IndexedSkeleton* Destination() const
		{
			return pDestination;
		}


		void Initialize(Kinematics::IndexedSkeleton* pSrc , Kinematics::IndexedSkeleton* pDst);
		// Attach the given Joint to given target joint
		void Attach(Kinematics::Joint* pSrc,Kinematics::Joint* pDst);
		void Attach(unsigned int SrcIndex,unsigned int DstIndex);

		void ResetBaseState();
		// perform a auto Attach the src joints from the most similar joint in the sub skeleton defined by it's parent's attached bone or pDstRoot
		// for most time , set pDst = DstSkeleton.root()
		// Return the attached joint ID , -1 == NULLJOINT
		unsigned int AutoAttach(Kinematics::Joint* pSrc);
		unsigned int AutoAttach(unsigned int Index);
		void AttachAll();

		unsigned int FindDominateJointID(unsigned int Index);
		unsigned int FindDriveJointID(unsigned int Index);

		void Detach(Kinematics::Joint* pSrcSubskeleton);
		void Detach(unsigned int SrcIndex);
		void DetachAll();

		// return nullptr for not found
		Kinematics::Joint* FindAttachedJoint(const Kinematics::Joint* pJoint) const;
		Kinematics::Joint* FindAttachedJoint(unsigned int JointID) const;

		/// Use Src skeleton to drive destination skeleton
		void Drive();
		/// <summary>
		/// Resets the motion of target to the state while attach.
		/// </summary>
		void ResetMotion();

		// The geometry similarity of two joint
		float Distance(const Kinematics::Joint* pSrc,const Kinematics::Joint* pDst) const;

		float DistanceChainToBone();

	protected:
		void RelativeDrive(Joint* pSrc,Joint* pDst);
		void DirectDrive(Joint* pSrc,Joint* pDst);

	protected:
		// Try to find some suitable binding joint for SrcJoint in the DstSubskeleton(except the root of it) 
		std::pair<Kinematics::Joint*,float> FindSimilarJoint(const Kinematics::Joint* pSrcJoint, Kinematics::Joint* pDstSubSkeleton);
		const static float BindingThreshold;
		std::map<unsigned int,unsigned int> Connections;
		IndexedSkeleton *pSource,*pDestination;
		std::map<unsigned int,Kinematics::KinematicsData> BaseState;
		//std::vector<Kinematics::KinematicsData>	BaseState;

	}; 

	class HS_MB_Adaptor
		: public IMotionAdaptor , public IHumanAdaptor
	{
	public:
		enum AttachType
		{
			Detached = -1,
			//Attached >= 0,
		};

		HS_MB_Adaptor(Kinematics::IndexedSkeleton& _Source, Geometrics::DynamicMetaBallModel& _Receptor );

		// Motion Adaptor interface
		virtual ~HS_MB_Adaptor();

		virtual size_t UpdateTranformMatrices(_Out_ DirectX::XMFLOAT3X4* TransformMatricesBuffer) const;

		virtual DirectX::XMMATRIX TransformMatrix(const std::vector<float>& weights) const;

		virtual std::vector<DirectX::SimpleMath::DualQuaternion> TransformBones() const;

		virtual size_t TranformMatricesCount() const
		{
			return Source.Index.size();
		}


		virtual void Alter(_Outref_ Eigen::MatrixXf& WeightMatrix);

		// Interface for operating the connection

		virtual void Detach();

		virtual void Detach(unsigned int SourceJointIndex);

		virtual bool Attach();

		virtual bool Attach(unsigned int SourceJointIndex);

		virtual bool IsAttached(unsigned int SourceJointIndex) const;

	private:
		Kinematics::IndexedSkeleton&		Source;
		Kinematics::IndexedSkeleton			Intermediate;
		Geometrics::DynamicMetaBallModel&	Receptor;
		std::map<unsigned int,unsigned int>	AttachTable;
	};







}