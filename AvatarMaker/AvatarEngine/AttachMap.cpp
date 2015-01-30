#include "stdafx.h"
#include "AttachMap.h"
#include "PrimitiveBatch.h"
#include "VertexTypes.h"
#include "Effects.h"
#include <DirectXColors.h>
#include <wrl\client.h>
#include <dlib\optimization\max_cost_assignment.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "HumanSkeleton.h"
#include <Eigen\Sparse>

using namespace std;
using namespace DirectX;
using namespace Microsoft::WRL;
using namespace boost;

namespace Kinematics
{

	bool HS_MB_Adaptor::Attach()
	{
		// Clear states
		AttachTable.clear();
		auto& volume = Receptor.Volume();
		const auto N = Receptor.Volume().size();
		const auto M = Source.Index.size();
		dlib::matrix<int> weights(M,N);
		const int uplimit = (std::numeric_limits<int>::max()-1)/M;
		for (auto& idx : Source.Index)
		{
			if (idx.second == nullptr) continue;
			const auto& joint = *idx.second;

			DirectX::XMVECTOR vPos = joint.Entity[Current].Position;

			bool inside =  volume.Contains(vPos);
			if (!inside) 
			{
				for (size_t i = 0; i < N; i++)
				{
					weights(idx.first,i) = 0;
				}
			} else
			{
				for (size_t i = 0; i < N; i++)
				{
					float value = volume.Primitives[i].eval(vPos);
					int level = value*uplimit;
					weights(idx.first,i) = level;
				}
			}
		}

		auto assignment = dlib::max_cost_assignment(weights);
		for (size_t i = 0; i < M; i++)
		{
			if (weights(i,assignment[i]) > 0)
			{
				AttachTable[i] = assignment[i]; // insert
				Source[i]->Snap_Default_to_Current();
			}
		}

		Receptor.SetDirty(Geometrics::DynamicMetaBallModel::WEIGHT_DATA);
		return !AttachTable.empty();
	}

	void HS_MB_Adaptor::Detach()
	{
		AttachTable.clear();
	}


	void HS_MB_Adaptor::Detach(unsigned int SourceJointIndex)
	{
		auto itr = AttachTable.find(SourceJointIndex);
		if (itr != AttachTable.end())
		{
			AttachTable.erase(itr);
			Receptor.SetDirty(Geometrics::DynamicMetaBallModel::WEIGHT_DATA);
		}
	}

	bool HS_MB_Adaptor::Attach(unsigned int SourceJointIndex)
	{
		if (IsAttached(SourceJointIndex)) return true;

		auto& volume = Receptor.Volume();
		const auto N = Receptor.Volume().size();
		const auto M = Source.Index.size();

		auto pJoint = Source[SourceJointIndex];
		DirectX::XMVECTOR vPos = pJoint->Entity[Current].Position;
		bool inside =  volume.Contains(vPos);
		if (!inside) return false;
		std::vector<float> Values;
		for (size_t i = 0; i < volume.size(); i++)
			Values.emplace_back(volume[i].eval(vPos));
		auto max_itr = std::max_element(Values.begin(),Values.end());
		size_t index = max_itr - Values.begin();
		for (auto& attachment : AttachTable)
			if (attachment.second == index) return false;

		AttachTable[SourceJointIndex] = index;
		Source[SourceJointIndex]->Snap_Default_to_Current();
		Receptor.SetDirty(Geometrics::DynamicMetaBallModel::WEIGHT_DATA);
		return true;
		//std::priority_queue<std::pair<uint16_t,float>,std::vector<std::pair<uint16_t,float>>,std::less
	}

	bool HS_MB_Adaptor::IsAttached(unsigned int SourceJointIndex) const
	{
		return AttachTable.find(SourceJointIndex) != AttachTable.end();
	}

	HS_MB_Adaptor::HS_MB_Adaptor(Kinematics::IndexedSkeleton& MotionSource, Geometrics::DynamicMetaBallModel& MotionReceptor)
		: Source(MotionSource) , Receptor(MotionReceptor)
	{
	}

	HS_MB_Adaptor::~HS_MB_Adaptor(){}

	DirectX::XMMATRIX HS_MB_Adaptor::TransformMatrix(const std::vector<float>& weights) const
	{
		if (weights.size() != Source.Index.size())
		{
			cout<<"Weights is not math"<<endl;
			return XMMatrixIdentity();
		}

		XMMATRIX Transform = XMMatrixZero();
		for (auto& bone_itr : Intermediate.Index)
		{
			XMMATRIX BoneTransform = bone_itr.second->BlendMatrix();
			BoneTransform *= weights[bone_itr.first];
		}
		return Transform;
	}

	size_t HS_MB_Adaptor::UpdateTranformMatrices(_Out_ DirectX::XMFLOAT3X4* TransformMatricesBuffer) const
	{
		size_t index = 0;
		for (auto& bone_itr : Source.Index)
		{
			XMMATRIX Transform = bone_itr.second->BlendMatrix();
			Transform = XMMatrixTranspose(Transform);
			XMStoreFloat3x4(TransformMatricesBuffer + index,Transform);
			++index;
		}
		return index;
	}

	std::vector<DirectX::SimpleMath::DualQuaternion> HS_MB_Adaptor::TransformBones() const
	{
		std::vector<DirectX::SimpleMath::DualQuaternion> Bones(TranformMatricesCount());
		for (auto& bone_itr : Source.Index)
		{
			Bones[bone_itr.first] = bone_itr.second->TransformDualQuaternion();
		}
		return Bones;
	}



	////	To-do :
	////	This weight schedule is not so good , it can not handel
	//void HS_MB_Adaptor::Alter(_Outref_ vector<vector<float>>& WeightMatrix)
	//{
	//	typedef Geometrics::ConnectionGraph graph_t;
	//	typedef graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
	//	typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;
	//	const auto& g = Receptor.Connections;
	//	auto& vertices = Receptor.Volume.Primitives;

	//	const auto N = Receptor.Volume.size();
	//	const auto M = Source.Index.size();

	//	WeightMatrix.resize(N);
	//	for (auto& weights : WeightMatrix)
	//	{
	//		weights.resize(M);
	//	}


	//	for (auto& attachment : AttachTable)
	//	{ // Dijkstra
	//		const auto& bone = attachment.first;
	//		//property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);

	//		std::vector<vertex_descriptor> p(num_vertices(g));
	//		std::vector<float> d(num_vertices(g));
	//		vertex_descriptor s = vertex(attachment.second, g);

	//		dijkstra_shortest_paths(g, s, distance_map(&d[0]));

	//		for (vertex_descriptor v = 0; v < num_vertices(g); ++v)
	//		{
	//			if (v!=s)
	//			{
	//				assert(d[v]>0);
	//				WeightMatrix[v][bone] = 1.0f/(d[v]*d[v]);
	//			}
	//		}
	//	}

	//	// Regualerize the weights
	//	for (size_t v = 0; v<=N ; ++v)
	//	{
	//		float sum = 0.0f;
	//		
	//		for (const auto& w : WeightMatrix[v])
	//		{
	//			sum += w;
	//		}
	//		for (auto& w : WeightMatrix[v])
	//		{
	//			w /= sum;
	//		}
	//	}

	//	// Regualerize every direct attached vertex
	//	for (auto& attachment : AttachTable)
	//	{
	//		auto& weights = WeightMatrix[attachment.second];
	//		std::fill(weights.begin(),weights.end(),0.0f);

	//		weights[attachment.first] = 1.0f;
	//	}

	//	//Compute color
	//	for (size_t i = 0; i < N; i++)
	//	{
	//		XMVECTOR vColor = DirectX::XMVectorZero();
	//		for (size_t j = 0; j < M; j++)
	//		{
	//			XMVECTOR vColorBone = XMLoadFloat4A(&HumanSkeleton::HumanBoneColorSchedule[j]);
	//			vColor += WeightMatrix[i][j] * vColorBone;
	//		}
	//		vertices[i].Color = vColor;
	//	}

	//}


	template <typename _Itr0,typename _Itr1>
	typename _Itr0& begin(std::pair<_Itr0,_Itr1>& range)
	{
		return range.first;
	}
	template <typename _Itr0,typename _Itr1>
	typename _Itr0& begin(std::pair<_Itr0,_Itr1>&& range)
	{
		return range.first;
	}

	template <typename _Itr0,typename _Itr1>
	typename const _Itr1& end(const std::pair<_Itr0,_Itr1>& range)
	{
		return range.second;
	}

	//	To-do :
	//	This weight schedule is not so good , it can not handel
	void HS_MB_Adaptor::Alter(_Outref_ Eigen::MatrixXf& WeightMatrix)
	{
		typedef Geometrics::ConnectionGraph graph_t;
		typedef graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
		typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;
		const auto& g = Receptor.Connections();
		auto& g_w = get(boost::edge_weight_t(), g);

		// Metaball size
		const auto N = Receptor.Volume().size();
		const auto BonesCount = Source.Index.size();
		//M <= BonesCount , M == the attached vertex count
		const auto M = AttachTable.size();

		//if (M==0)
		//{
		//	WeightMatrix.resize(N,1);
		//	WeightMatrix.setOnes();
		//	Receptor.Colors().resize(N);
		//	//Compute color
		//	for (size_t i = 0; i < N; i++)
		//	{
		//		XMVECTOR vColor = DirectX::XMVectorZero();
		//		for (size_t j = 0; j < BonesCount; j++)
		//		{
		//			XMVECTOR vColorBone = XMLoadFloat4A(&HumanSkeleton::HumanBoneColorSchedule[j]);
		//			vColor += WeightMatrix(i,j) * vColorBone;
		//		}
		//		//using namespace DirectX::SimpleMath;
		//		Receptor.Colors()[i] = vColor;
		//	}
		//	return;
		//}


		const auto Vn = N-M;
		std::vector<size_t> Conv(static_cast<size_t>(N),0);
		std::vector<size_t> Conb(Vn);

		for (auto& attachment : AttachTable)
			Conv[attachment.second] = 1;
		for (size_t i = 1; i < N; i++)
			Conv[i] += Conv[i-1];
		for (size_t i = 0; i < N; i++)
			Conv[i] = i - Conv[i];
		for (auto& attachment : AttachTable)
			Conv[attachment.second] = -1;
		for (size_t i = 0; i < N; i++)
		{
			if (Conv[i] != -1)
				Conb[Conv[i]] = i;
		}

		WeightMatrix.resize(N,BonesCount);
		WeightMatrix.fill(.0f);
		//WeightMatrix.resize(N);
		//for (auto& weights : WeightMatrix)
		//	weights.assign(BonesCount,.0f);

		Eigen::SparseMatrix<float> Laplace(Vn,Vn);

		// Construct Laplace Matrix
		{
			std::vector<Eigen::Triplet<float>> triplets;
			std::vector<float> weights_sum(Vn,0.0f);
			decltype(edges(g).first) eitr,eend;
			for (std::tie(eitr,eend) = edges(g); eitr != eend; ++eitr)
			{
				int x = source(*eitr,g) , y = target(*eitr,g);
				float w = g_w[*eitr];// Convert distance to weights
				x = Conv[x];
				y = Conv[y];
				if (x!=-1 && y!=-1)
				{
					triplets.emplace_back(x,y,-w);
					triplets.emplace_back(y,x,-w);
					weights_sum[x] += w;
					weights_sum[y] += w;
				} else if( x!= -1 ) 
				{
					weights_sum[x] += w;
				} else if ( y!= -1 )
				{
					weights_sum[y] += w;
				}
			}

			for (size_t i = 0; i < Vn; i++)
			{
				assert(weights_sum[i] != .0f);
				triplets.emplace_back(i,i,weights_sum[i]);
			}

			Laplace.setFromTriplets(triplets.begin(),triplets.end());
		}

		Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> Cholesky(Laplace); // performs a Cholesky factorization of A
		// for each attached bone
		for (auto& attachment : AttachTable)
		{
			auto bone = attachment.first;
			auto v = attachment.second;

			// Target vector describe the boundry condition : 
			// W[v][bone] == 1 && W[u][bone] == 0 (u mapped to other bone)
			Eigen::VectorXf B(Vn);
			B.fill(.0f);
			//B(0) = 0;
			// Construct B
			{
				//decltype(in_edges(v,g).first) eitr,eend;
				for (auto range = in_edges(v,g); range.first != range.second; ++range.first)
				{
					auto& eitr = range.first;
					auto u = source(*eitr,g);
					if (Conv[u]!=-1)
					{
						float w = g_w[*eitr]; // Convert distance to weights
						B(Conv[u]) = w;
					}
				}
			}

			// Slove Lapace*A = B;
			Eigen::VectorXf A = Cholesky.solve(B);

			for (int i = 0; i < A.size(); i++)
			{
				WeightMatrix(Conb[i],bone) = A[i];
			}

			WeightMatrix(v,bone) = 1.0f;
		}

		// We can prove the weights is naturaly reguarlized by using the eigenvalue of Lapalace Matrix
		//// Regualerize the weights
//#ifdef _DEBUG

	for (size_t v = 0; v < N ; ++v)
	{
		float sum = WeightMatrix.row(v).sum();
		if (sum < XM_EPSILON)
		{
			WeightMatrix(v,0) = 1.0f;
		}

		//0.0f;
		//for (const auto& w : WeightMatrix[v])
		//	sum += w;
		//assert(std::abs(sum-1.0f) < 0.1f);
		//for (auto& w : WeightMatrix[v])
		//	w /= sum;
	}

//#endif
		Receptor.Colors().resize(N);
		//Compute color
		for (size_t i = 0; i < N; i++)
		{
			XMVECTOR vColor = DirectX::XMVectorZero();
			for (size_t j = 0; j < BonesCount; j++)
			{
				XMVECTOR vColorBone = XMLoadFloat4A(&HumanSkeleton::HumanBoneColorSchedule[j]);
				vColor += WeightMatrix(i,j) * vColorBone;
			}
			//using namespace DirectX::SimpleMath;
			Receptor.Colors()[i] = vColor;
		}

	}

	const float AttachMap::BindingThreshold = 2.0f;
	const float Direction_Threshold = XM_PI / 3;
	const float Displacement_Threshold = 2.0f;


	AttachMap::AttachMap()
		//: m_pViewer(nullptr)
	{
		pSource = nullptr;
		pDestination = nullptr;
	}

	AttachMap::~AttachMap(void)
	{
	}

	void AttachMap::Initialize(Kinematics::IndexedSkeleton* pSrc , Kinematics::IndexedSkeleton* pDst)
	{
		Connections.clear();
		pSource = pSrc;
		pDestination = pDst;
		for (auto& itr : pDestination->Index)
		{
			if (BaseState.find(itr.first) == BaseState.end())
				BaseState[itr.first] = itr.second->Entity[Current];
		}
	}

	float AttachMap::Distance(const Joint* pSrc,const Joint* pDst) const
	{
		float Displacement = (pSrc->Entity[Current].Position-pDst->Entity[Current].Position).Length()/pDst->Radius;
		Displacement /= Displacement_Threshold;
		// Count the direction's angle
		XMVECTOR dir0 = DirectX::XMVector3Rotate(g_XMIdentityR1,(XMVECTOR)pSrc->Entity[Current].Orientation);
		XMVECTOR dir1 = DirectX::XMVector3Rotate(g_XMIdentityR1,(XMVECTOR)pDst->Entity[Current].Orientation);
		float DirectionAngle = DirectX::Vector3::Dot(dir0,dir1);
		DirectionAngle = acosf(DirectionAngle) / Direction_Threshold;
		//		auto OrientationFactor = XMQuaternionMultiply((XMVECTOR)pSrc->Entity[Current].Orientation,pDst->Entity[Current].Orientation.Inverse());
		//float theta = XMVectorGetW(OrientationFactor);
		//theta = sqrtf(2-2*theta*theta);
		return Displacement * DirectionAngle ;
	}

	Kinematics::Joint* AttachMap::FindAttachedJoint(const Kinematics::Joint* pJoint) const
	{
		auto itr = Connections.find(pJoint->ID);
		if (itr == Connections.end() || !pSource->containts(pJoint))
			return nullptr;
		return (*pDestination)[itr->second];
	}
	Kinematics::Joint* AttachMap::FindAttachedJoint(unsigned int JointID) const
	{
		auto itr = Connections.find(JointID);
		if (itr == Connections.end() || !pSource->containtsKey(JointID))
			return nullptr;
		return (*pDestination)[itr->second];
	}


	void AttachMap::DetachAll()
	{
		if (pSource && !pSource->empty())
			Detach(pSource->Root);
	}


	void AttachMap::Detach(Joint* pSrc)
	{
		assert(pSrc);
		Detach(pSrc->ID);
	}

	void AttachMap::Detach(unsigned int SrcIndex)
	{
		if ((*pSource)[SrcIndex]==nullptr) 
			throw std::out_of_range("Source Index out of range.");
		if (Connections.find((SrcIndex))!=Connections.end())
		{
			// Recover the state to default
			// (*pDestination)[Connections[SrcIndex]]->Snap_Default_to_Current();
			auto DstIndex = Connections.at(SrcIndex);
			BaseState[Connections.at(SrcIndex)] = (*pDestination)[DstIndex]->Entity[Default];
			Connections.erase(SrcIndex);
		}
		for (Joint* pJoint : (*pSource)[SrcIndex]->Children)
		{
			Detach(pJoint);
		}
	}

	unsigned int AttachMap::AutoAttach(unsigned int Index)
	{
		return AutoAttach((*pSource)[Index]);
	}

	unsigned int AttachMap::AutoAttach(Joint* pSrc)
	{
		unsigned int ReturnCode = 0;
		if (pSrc==nullptr) return -2;
		if(Connections.find(pSrc->ID)!=Connections.end())
			//	Detach(pSrc);
				return Connections[pSrc->ID]; // Already attach to some other joint
		if (pSrc->is_root())
		{
			if (!pDestination->Root)
				return -2;
			//	throw new std::exception("Can't attach to null!");
			Attach(pSrc,pDestination->Root);
			ReturnCode = Connections[pSrc->ID];

			//XMVECTOR vRoot= pSrc->Entity[Current].Position;
			//float minDis = 1e6;
			//Joint* pRootCandinate = nullptr;
			//for (auto& idx : pDestination->Index)
			//{
			//	XMVECTOR vDis = vRoot - idx.second->Entity[Current].Position;
			//	float dis = Vector3::LengthSq(vDis);
			//	if (dis < minDis)
			//	{
			//		minDis = dis;
			//		pRootCandinate = idx.second;
			//	}
			//}
			//if (!pRootCandinate) ReturnCode = -2;
			//else
			//{
			//	pDestination->SelectNewRoot(pRootCandinate);
			//	assert (pDestination->Root == pRootCandinate);
			//	pDestination->Update();
			//	Attach(pSrc,pRootCandinate);
			//	ReturnCode = Connections[pRootCandinate->ID];
			//}
		}	else
		{
			// Find a attached parent
			auto q = pSrc->Parent;
			while (q!=nullptr && Connections.find(q->ID) == Connections.end())
				q = q->Parent;
			if (q==nullptr) 
				ReturnCode = -1;
			else
			{
				Joint* pDst = (*pDestination)[Connections.find(q->ID)->second];
				auto Candinate = FindSimilarJoint(pSrc,pDst);

				if (Candinate.second < BindingThreshold)
				{
					Attach((*pSource)[pSrc->ID],(*pDestination)[Candinate.first->ID]);
					ReturnCode = Connections[pSrc->ID];
				}
				else 
					ReturnCode = -1;
			}

			// Find the most "similar" joint among the descendents of it's parent
		}

		// Try Attach all the children
		for (Joint* pJoint : pSrc->Children)
		{
			AutoAttach(pJoint);
		}

		return ReturnCode;
	}

	std::pair<Joint*,float> AttachMap::FindSimilarJoint(const Joint* pSrc, Joint* pDst)
	{
		std::pair<Joint*,float> Candinate(nullptr,10000.0f);
		//	for (auto itr = pDst->Children.begin(); itr != pDst->Children.end(); itr++)
		for (Joint* pJoint : pDst->Children)
		{
			float Sim = Distance(pSrc,pJoint);

			auto SubInfo = FindSimilarJoint(pSrc,pJoint);
			if (Sim < SubInfo.second){
				SubInfo.first = pJoint;
				SubInfo.second = Sim;
			}
			if (SubInfo.second < Candinate.second){
				Candinate=SubInfo;
			}
		}
		return Candinate;
	}

	void AttachMap::RelativeDrive(Joint* pSrc,Joint* pDst)
	{
		//if (pDst->ID >= BaseState.size())
		//{
		//	auto maxitr = pDestination->Index.cend();
		//	--maxitr;
		//	auto size_o = BaseState.size();
		//	BaseState.resize(maxitr->first + 1);
		//	for (size_t i = size_o; i < BaseState.size(); i++)
		//	{
		//		if ((*pDestination)[i])
		//		BaseState[i] = (*pDestination)[i]->Entity[Current];
		//	}
		//}
		if (BaseState.find(pDst->ID) == BaseState.end())
		{
			BaseState[pDst->ID] = pDst->Entity[Current];
		}
		if (pDst->is_root()){
			pDst->Entity[Current].Position = BaseState[pDst->ID].Position + pSrc->Entity[Current].Position - pSrc->Entity[Default].Position;
			XMVECTOR qInvDefalt = XMQuaternionInverse(pSrc->Entity[Default].Orientation);
			pDst->Entity[Current].Orientation = XMQuaternionMultiply(BaseState[pDst->ID].Orientation,XMQuaternionMultiply(qInvDefalt,pSrc->Entity[Current].Orientation));
		} else	{
			// Type 1 : Relative Drive , since we have to consider the effect of self rotation , let's just use this
			XMVECTOR qInvDefalt = XMQuaternionInverse(pSrc->Entity[Default].Orientation);
			pDst->Entity[Current].Rotation = XMQuaternionMultiply(BaseState[pDst->ID].Rotation,XMQuaternionMultiply(qInvDefalt,pSrc->Entity[Current].Rotation));	
		}
	}

	void AttachMap::DirectDrive(Joint* pSrc,Joint* pDst)
	{
		if (pDst->is_root()){
			pDst->Entity[Current].Position = pSrc->Entity[Current].Position;
			pDst->Entity[Current].Orientation = pSrc->Entity[Current].Orientation;
		} else	{
			// Type 2 : Direct Drive
			pDst->Entity[Current].Rotation = pSrc->Entity[Current].Rotation;
		}
	}

	void AttachMap::ResetMotion()
	{
		if (!pDestination) return;
		for (auto& item : pDestination->Index)
		{
			if (BaseState.find(item.first) != BaseState.end())
			{
				item.second->Entity[Current] = BaseState.at(item.first);
			} else
			{
				item.second->Snap_Current_to_Default();
			}
		}
	}



	void AttachMap::Drive()
	{
		if (!pDestination || !pSource) 
			return;
		auto &Dst = *pDestination;
		auto &Src = *pSource;
		//Dst.for_all([&](Joint* pDst){
		//	auto pSrc = Src[FindDriveJointID(pDst->ID)];
		//	if (!pSrc) pDst->Deduce_Global_from_Hierarchical(Current);
		//	else
		//	{
		//		Joint* pParent = pDst->Parent;
		//		XMVECTOR qDeltaOrientation = XMQuaternionMultiply(pSrc->Entity[Default].Orientation.Inverse(),pSrc->Entity[Current].Orientation);
		//		pDst->Entity[Current].Orientation = XMQuaternionMultiply(pDst->Entity[Default].Orientation,qDeltaOrientation);
		//		if (pParent)
		//		{
		//			XMVECTOR RelativeVector = Vector3::Rotate(g_XMIdentityR1 * pDst->Entity[Current].Scale,(XMVECTOR)pDst->Entity[Current].Orientation);
		//			pDst->Entity[Current].Position = pParent->Entity[Current].Position+ RelativeVector;
		//			pDst->Entity[Current].Rotation = XMQuaternionMultiply(pDst->Entity[Current].Orientation,pParent->Entity[Current].Orientation.Inverse());
		//		} else
		//		{
		//			pDst->Entity[Current].Position = pDst->Entity[Default].Position + pSrc->Entity[Current].Position - pSrc->Entity[Default].Position;
		//		}
		//	}
		//});
		Dst.for_all([&](Joint* pDst){
			auto pSrc = Src[FindDriveJointID(pDst->ID)];
			if (!pSrc) pDst->Deduce_Global_from_Hierarchical(Current);
			else
			{
				Joint* pParent = pDst->Parent;
				XMVECTOR qInvDefalt = XMQuaternionInverse(pSrc->Entity[Default].Orientation);
				XMVECTOR qDeltaOrientation = XMQuaternionMultiply(qInvDefalt,pSrc->Entity[Current].Orientation);
				pDst->Entity[Current].Orientation = XMQuaternionMultiply(BaseState[pDst->ID].Orientation,qDeltaOrientation);
				if (pParent)
				{
					XMVECTOR RelativeVector = Vector3::Rotate(g_XMIdentityR1 * pDst->Entity[Current].Scale,(XMVECTOR)pDst->Entity[Current].Orientation);
					pDst->Entity[Current].Position = pParent->Entity[Current].Position + RelativeVector;
					pDst->Entity[Current].Rotation = XMQuaternionMultiply(pDst->Entity[Current].Orientation,XMQuaternionInverse(pParent->Entity[Current].Orientation));
				} else
				{
					pDst->Entity[Current].Position = BaseState[pDst->ID].Position + pSrc->Entity[Current].Position - pSrc->Entity[Default].Position;
				}
			}
		});


		//vector<unsigned int> InVailidKeys;
		//for (auto itr : Connections)
		//{
		//	if (Dst[itr.second] && Src[itr.first]) {
		//		RelativeDrive(Src[itr.first],Dst[itr.second]);
		//		//DirectDrive(Src[itr.first],Dst[itr.second]);

		//		//Drive Brothers
		//		//if (Dst[itr.second]->is_root()) 
		//		//	continue;
		//		//auto& Brothers = Dst[itr.second]->Parent->Children;
		//		//for (Joint* pBrother : Brothers)
		//		//{
		//		//	if (FindDriveJointID(pBrother->ID) == -1)
		//		//		RelativeDrive(Src[itr.first],pBrother);
		//		//}

		//	}
		//	else
		//		InVailidKeys.push_back(itr.first);
		//}
		//for (unsigned int Key : InVailidKeys)
		//{
		//	Connections.erase(Key);
		//}

		//Dst.Update();
	}

	unsigned int AttachMap::FindDominateJointID(unsigned int Index)
	{
		if (!pDestination->containtsKey(Index)) return -2;
		for (auto& itr : Connections)
		{
			if (itr.second == Index)
				return itr.first;
		}
		return FindDominateJointID((*pDestination)[Index]->Parent->ID);
	}

	unsigned int AttachMap::FindDriveJointID(unsigned int Index)
	{
		if (!pDestination->containtsKey(Index)) return -2;
		for (auto& itr : Connections)
		{
			if (itr.second == Index)
				return itr.first;
		}
		return -1;
	}

	void AttachMap::ResetBaseState()
	{
		BaseState.clear();
	}

	void AttachMap::Attach( Kinematics::Joint* pSrc,Kinematics::Joint* pDst )
	{
		if (Connections.find(pSrc->ID) == Connections.end())
			Connections[pSrc->ID] = pDst->ID;
		pSrc->Snap_Default_to_Current();
		//if (pDst->ID > BaseState.size())
		//{
		//	auto maxitr = pDestination->Index.cend();
		//	--maxitr;
		//	BaseState.resize(maxitr->first + 1);
		//}

		BaseState[pDst->ID] = pDst->Entity[Current];
		//pDst->Snap_Default_to_Current();
		//.insert(AttachMap::value_type(,pDst));
	}

	void AttachMap::Attach(unsigned int SrcIndex,unsigned int DstIndex)
	{
		if (!pSource->containtsKey(SrcIndex)) return;
		if (FindDriveJointID(DstIndex) != -1) return;
		if (Connections.find(SrcIndex) == Connections.end())
			Connections[SrcIndex] = DstIndex;
	}



}