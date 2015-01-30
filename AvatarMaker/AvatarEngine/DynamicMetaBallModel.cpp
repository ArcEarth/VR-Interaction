#include "stdafx.h"
#include "DynamicMetaBallModel.h"
#include "SafeDelete.h"
#include "DXMathExtend.h"
#include <iostream>
#include <Eigen\Sparse>
#include "HumanSkeleton.h"

using namespace DirectX;
using namespace Kinematics;
using namespace std;
using namespace boost;
using namespace Microsoft::WRL;

//#define Itr (*itr)
namespace Geometrics
{

	//	const float Metaball_Spare_Factor = 1.75f;
	//	const float Metaball_Spare_Factor = 1.25f;
	//	const float Metaball_Spare_Factor = 1.05f;
	//	const float Metaball_Spare_Factor = 0.875f;
	const float Metaball_Spare_Factor = 0.5f;

	const float Metaball_Radius_Superior_Limit = 0.5f;
	const float Metaball_Radius_Infieror_Limit = 0.03f;
	const unsigned int Default_Skin_Texture_Resolution = 1024;
	//static_assert();
	//static const CompositeFlag<DynamicMetaBallModel::RenderMode> VISIBLE_SKIN(DynamicMetaBallModel::SKIN_ANIMATED || DynamicMetaBallModel::SKIN_DEFAULT);
	//static const CompositeFlag<DynamicMetaBallModel::RenderMode> VISIBLE_METABALLS(DynamicMetaBallModel::METABALLS_ANIMATED || DynamicMetaBallModel::METABALLS_DEFAULT);
	//static const CompositeFlag<DynamicMetaBallModel::RenderMode> VISIBLE_SKELETON(DynamicMetaBallModel::SKELETON_ANIMATED || DynamicMetaBallModel::SKELETON_DEFAULT);

	//DirectX::Quaternion DynamicMetaBallModel::getOrientation() const
	//{
	//	return Skeleton->Root->Entity[Current].Orientation;
	//}

	//void DynamicMetaBallModel::setOrientation(const DirectX::Quaternion &NewOrientation) const
	//{
	//	Skeleton->Root->Entity[Current].Orientation = NewOrientation;
	//}


	//DirectX::Vector3 DynamicMetaBallModel::getPosition() const
	//{
	//	return Skeleton->Root->Entity[Current].Position;
	//}

	//void DynamicMetaBallModel::setPosition(const DirectX::Vector3& NewPosition)
	//{
	//	Skeleton->Root->Entity[Current].Position = NewPosition;
	//}


	//void DynamicMetaBallModel::Render(ID3D11DeviceContext *pContext)
	//{
	//	auto &BlendMatrics = m_SkinMesh.ConstantBuffer_W().BlendMatrices;

	//	//if (!m_pAdaptor)
	//	//	size_t N = m_pAdaptor->TranformMatricesCount();
	//	//ZeroMemory(BlendMatrics,sizeof(N * sizeof(XMFLOAT3X4)));
	//	//for (size_t i = 0; i < N; i++)
	//	//{
	//	//	BlendMatrics[i]._11 = 1.0f;
	//	//	BlendMatrics[i]._22 = 1.0f;
	//	//	BlendMatrics[i]._33 = 1.0f;
	//	//}

	//	//Microsoft::WRL::ComPtr<ID3D11Device> pDevice;
	//	//pContext->GetDevice(&pDevice);
	//	//m_SkinMesh.Refresh(pDevice.Get());
	//	m_SkinMesh.Render(pContext);
	//}

	void DynamicMetaBallModel::Clear()
	{
		m_RestVolume.clear();
		m_AnimatedVolume.clear();
		m_Connections.clear();
		m_VolumeWeights.resize(0,0);
		m_SkinMesh.Clear();
		XMFLOAT3X4 Identity;
		XMStoreFloat3x4(&Identity,XMMatrixIdentity());
		XMStoreFloat4x4(&m_SkinMesh.ConstantBuffer_W().WorldMatrix,XMMatrixIdentity());
		std::fill_n(m_SkinMesh.ConstantBuffer_W().BlendMatrices,MAX_BLEND_MATRIX_COUNT,Identity);
		m_pAdaptor = nullptr;
	}

	void DynamicMetaBallModel::Disconnect()
	{
		XMFLOAT3X4 Identity;
		XMStoreFloat3x4(&Identity,XMMatrixIdentity());
		XMStoreFloat4x4(&m_SkinMesh.ConstantBuffer_W().WorldMatrix,XMMatrixIdentity());
		std::fill_n(m_SkinMesh.ConstantBuffer_W().BlendMatrices,MAX_BLEND_MATRIX_COUNT,Identity);
		m_pAdaptor = nullptr;
		m_AnimatedVolume = m_RestVolume;
	}

	void Connect(const std::shared_ptr<IMotionAdaptor>& pAdaptor)
	{

	}


	DirectX::XMFLOAT2 DynamicMetaBallModel::UVMapping(DirectX::FXMVECTOR VertexPosition) const
	{
		XMVECTOR ptr = VertexPosition;
		if (m_RestVolume.size()>0)	
		{
			XMVECTOR SphereCenter = m_RestVolume[0].Position;
			ptr -= SphereCenter;
		}

		ptr = DirectX::XMVector3Normalize(ptr);
		XMFLOAT3A p;
		XMStoreFloat3A(&p,ptr);

		//float u = 0.5f + std::atan2f(p.z,p.x) * XM_1DIV2PI;
		float u = 0.5f + std::atan2f(p.x,-p.z) * XM_1DIV2PI;
		float v = 0.5f - std::asinf(p.y) * XM_1DIVPI;

		//float r = sqrtf(p.x*p.x+p.z*p.z);
		//float v = -atanf(p.y/r);
		//float u = acosf(p.x/r);
		//if (p.z<0.0f) u = -u;
		//u += XM_PI;
		//u *= XM_1DIV2PI;
		//v = v * XM_1DIVPI + 0.5f;
		return XMFLOAT2(u,v);
	}

	void MaippingTexture(std::vector<DirectX::SkinVertex> &vertrices);

	void DynamicMetaBallModel::Tessellate()
	{
		if (m_TessalationPrecise <= .0f) return;
		// If need to render the skin , then tessellate it
		//cout<<"Tessellating..."<<endl;
		m_RestVolume.Tessellate(m_SkinMesh.Vertices_W(),m_SkinMesh.Indices_W(),m_TessalationPrecise);
		WeightingSkin(m_SkinMesh.Vertices_W());
		MaippingTexture(m_SkinMesh.Vertices_W());
		//ResoloveCrossBoundryTriangle();
		//cout<<"Tessellation Finish..."<<endl;
	}

	void DynamicMetaBallModel::Remesh()
	{
		AnimationUpdate();
		m_RestVolume = m_AnimatedVolume;
		Update();
		this->Tessellate();
	}

	void DynamicMetaBallModel::AddMetaball(const Metaball &Ball)
	{
		m_DirtyFlag |= DirtyFlagEnum::VOLUME_DATA;
		if (!m_pAdaptor)
			m_RestVolume.Primitives.push_back(Ball);
		else
		{
			Metaball ball_MS(Ball);
			InverseDeformMetaball(ball_MS);
			m_RestVolume.Primitives.push_back(ball_MS);
		}
	}

	void DynamicMetaBallModel::AddSphere(const DirectX::BoundingSphere &Sphere)
	{
		m_DirtyFlag |= DirtyFlagEnum::VOLUME_DATA;
		float eratio = m_RestVolume.EffictiveRadiusRatio();
		m_RestVolume.Primitives.emplace_back(Sphere.Center,Sphere.Radius / eratio);
	};

	void DynamicMetaBallModel::AddCylinder(DirectX::FXMVECTOR p0, DirectX::FXMVECTOR p1 , float R0, float R1 , bool ExceptFront /*= false*/)
	{
		m_DirtyFlag |= DirtyFlagEnum::VOLUME_DATA;
		MetaBallModel Cylinder;
		float dis = Vector3::Length(p1-p0);
		float disthre = (R0+R1)*0.5f*Metaball_Spare_Factor;
		int m = (int)(dis/disthre);
		XMVECTOR vUnit= (p1-p0)/(float)(m+1);
		XMVECTOR vP = p0;
		float RadiusStep = (R1 - R0)/(m+1);
		float Radius = R0;
		int i = 0;
		if (ExceptFront)
		{
			vP += vUnit;
			Radius += RadiusStep;
			i = 1;
		}
		for(; i <= m+1; i++,Radius+=RadiusStep,vP += vUnit)
			Cylinder.Primitives.emplace_back(vP,Radius);
		AddVolume(Cylinder);
	}

	DirectX::XMMATRIX DynamicMetaBallModel::TransformMatrix(std::vector<float> WeightVector) const
	{
		const size_t M = m_VolumeWeights.cols();
		if (M<1) return XMMatrixIdentity();

		const auto& BlendMatrices = m_SkinMesh.ConstantBuffer().BlendMatrices;

		XMMATRIX Transform = XMLoadFloat3x4(&BlendMatrices[0]);
		Transform = XMMatrixTranspose(Transform);
		Transform *= WeightVector[0];

		for (size_t bone = 1; bone < M; bone++)
		{
			if (WeightVector[bone] > 0)
			{
				XMMATRIX BoneTransform = XMLoadFloat3x4(&BlendMatrices[bone]);
				BoneTransform = XMMatrixTranspose(BoneTransform);
				BoneTransform *= WeightVector[bone];
				Transform += BoneTransform;
			}
		}
		return Transform;
	}



	std::vector<float> DynamicMetaBallModel::InverseDeformMetaball(Metaball& Ball) const
	{
		const auto M = m_VolumeWeights.cols();
		Eigen::VectorXf Weights(M);

		Weights.setZero();
		float sum = .0f;
		for (size_t i = 0; i < m_AnimatedVolume.size(); i++)
		{
			if (m_AnimatedVolume.IsTwoMetaballIntersect(m_AnimatedVolume[i],Ball))
			{
				float w = Metaball::ConnectionStrength(m_AnimatedVolume[i],Ball,m_AnimatedVolume.ISO);
				sum += w;
				Weights += m_VolumeWeights.row(i);
			}
		}
		if (sum <= XM_EPSILON) return std::vector<float>();
		Weights /= sum;
		std::vector<float> weights(Weights.data(),Weights.data()+M);
		XMMATRIX Transform = TransformMatrix(weights);
		Transform = XMMatrixInverse(nullptr,Transform);
		Ball.Position.Transform(Transform);
		return weights;
	}

	unsigned int DynamicMetaBallModel::SetDirty(DirtyFlagEnum dirtyFalg)
	{
		m_DirtyFlag |= dirtyFalg;
		return m_DirtyFlag;
	}


	void DynamicMetaBallModel::AddVolume(const MetaBallModel & Volume, std::function<void(MetaBallModel &,Eigen::MatrixXf&)> RetesslateionFunc/* = nullptr*/)
	{
		if (Volume.ISO != m_RestVolume.ISO) throw std::exception("ISO Unmatched");

		float iso = m_RestVolume.ISO;
		const auto N = m_RestVolume.size();
		m_DirtyFlag |= DirtyFlagEnum::VOLUME_DATA;

		// Since the model is not animated yet , the World Space and Model Space should be the same , no need to caculate the volume weights
		if (!m_pAdaptor || N==0)
		{
			// Append the metaballs
			std::copy(Volume.Primitives.begin(),Volume.Primitives.end(),std::back_inserter(m_RestVolume.Primitives));
			return;
		}

		const auto M = m_pAdaptor->TranformMatricesCount();

		//vector<vector<float>> Weights;
		//Weights.resize(Volume.size());
		//for (auto& weights : Weights)
		//	weights.resize(M);
		Eigen::MatrixXf Weights(Volume.size(),M);

		graph_traits<ConnectionGraph>::edge_iterator eitr,eend;
		graph_traits<ConnectionGraph>::vertex_iterator vitr,vend;

		ConnectionGraph g_cross(N+Volume.size());

		const auto Vn = Volume.size();
		Eigen::SparseMatrix<float> Laplace(Vn,Vn);

		// Add edges with in Volume
		{
			ConnectionGraph g_volume;
			Volume.CreateConnectionGraph(g_volume);

			std::vector<Eigen::Triplet<float>> triplets;
			std::vector<float> weights_sum(Vn,0.0f);

			const auto& g_w = get(edge_weight, g_volume);
			for (std::tie(eitr,eend) = edges(g_volume); eitr != eend; eitr++)
			{
				int x = source(*eitr,g_volume) , y = target(*eitr,g_volume);
				float w = g_w[*eitr];

				triplets.emplace_back(x,y,-w);
				triplets.emplace_back(y,x,-w);
				weights_sum[x] += w;
				weights_sum[y] += w;
			}

			// Add edges cross Volume and m_AnimatedVolume
			for (size_t v = 0; v < N; v++)
			{
				for (size_t u = 0; u < Volume.size(); u++)
				{
					if (Metaball::Connected(m_AnimatedVolume[v],Volume[u],iso))
					{
						float w = Metaball::ConnectionStrength(m_AnimatedVolume[v],Volume[u],iso);
						add_edge(v,u+N,w,g_cross);


						weights_sum[u] += w;
					}
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

		// Build the weight matrix for new add volume
		//for (size_t bone = 0; bone < M; bone++)
		//{
		//	Eigen::VectorXf B(Vn);
		//	B.fill(.0f);
		//	const auto& g_w = get(edge_weight, g_cross);
		//	for (std::tie(eitr,eend) = edges(g_cross); eitr != eend; eitr++)
		//	{
		//		int x = source(*eitr,g_cross) , y = target(*eitr,g_cross);
		//		if (x>y) std::swap(x,y);
		//		float w = g_w[*eitr];
		//		y -= N; //assert(y>=0)
		//		B(y) += m_VolumeWeights(x,bone) * w;
		//	}

		//	bool zero = std::all_of(B.data(),B.data()+Vn,[](float value)->bool{
		//		return std::abs(value) < 1e-3f;
		//	});
		//	if (zero) continue; // The weights for this bone should be zero

		//	Eigen::VectorXf A = Cholesky.solve(B);
		//	for (size_t v = 0; v < Vn; v++)
		//	{
		//		Weights(v,bone) = A(v);
		//	}
		//}

		Eigen::MatrixXf B(Vn,M);
		B.fill(.0f);
		const auto& g_w = get(edge_weight, g_cross);
		for (std::tie(eitr,eend) = edges(g_cross); eitr != eend; eitr++)
		{
			int x = source(*eitr,g_cross) , y = target(*eitr,g_cross);
			if (x>y) std::swap(x,y);
			float w = g_w[*eitr];
			y -= N; //assert(y>=0)
			B.row(y) += m_VolumeWeights.row(x) * w;
		}
		Weights = Cholesky.solve(B);

		// Copy the metabaslls
		const auto & BlendMatrices = m_SkinMesh.ConstantBuffer().BlendMatrices;

		MetaBallModel RestVolume;
		for (size_t v = 0; v < Vn; v++)
		{
			XMMATRIX Transform = XMLoadFloat3x4(&BlendMatrices[0]);
			Transform = XMMatrixTranspose(Transform);
			Transform *= Weights(v,0);

			XMVECTOR vColor = XMVectorZero();
			for (size_t bone = 1; bone < M; bone++)
			{
				if (Weights(v,bone) > 0)
				{
					XMMATRIX BoneTransform = XMLoadFloat3x4(&BlendMatrices[bone]);
					BoneTransform = XMMatrixTranspose(BoneTransform);
					BoneTransform *= Weights(v,bone);
					Transform += BoneTransform;
					vColor += Weights(v,bone) * XMLoadFloat4A(&HumanSkeleton::HumanBoneColorSchedule[bone]);
				}
			}

			XMVECTOR vDeterminat;
			Transform = XMMatrixInverse(&vDeterminat,Transform);
			XMVECTOR vPos = Volume[v].Position;
			if (!XMVector4Equal(vDeterminat,g_XMZero))
				vPos = XMVector3TransformCoord(vPos,Transform);
			else
				cout<<"Can't Inverse!"<<endl;
			RestVolume.Primitives.emplace_back(vPos,Volume[v].Radius);
			m_VolumeColors.emplace_back(vColor);
		}

		// To-Do :
		// Since we did a non-rigid transform to all the metaball there , there is no constraint could assert the toponology is keeped
		// So we do need to do something to maintain this ... but , how ? = =|||
		// S1 : Undesired edge broke ==> Adaptive SUBDIVIDE the structure graph until it could keep the topology , 
		// S2 : Undesired edge emerge ==> No good sulution comes out yet
		if (RetesslateionFunc != nullptr)
			RetesslateionFunc(RestVolume,Weights);

		// Append the weight matrix
		{
			Eigen::MatrixXf temp(N+RestVolume.size(),M);
			temp.block(0,0,N,M) = m_VolumeWeights;
			temp.block(N,0,RestVolume.size(),M) = Weights;
			m_VolumeWeights = std::move(temp);
		}

		// Append the metaballs
		std::copy(RestVolume.begin(),RestVolume.end(),std::back_inserter(m_RestVolume.Primitives));

		// Update the animated volume / Connection graph / polygon mesh
		// Update();
	}

	bool DynamicMetaBallModel::RemoveMetaball_If(std::function<bool(unsigned int)> Pred)
	{
		const auto N = m_RestVolume.size();
		std::vector<bool> deleted_flags(N);
		for (size_t i = 0; i < N; i++)
		{
			deleted_flags[i] = Pred(i);
		}
		RemoveMetaball_If(deleted_flags);

		return std::any_of(deleted_flags.begin(),deleted_flags.end(),[](bool arg)->bool{return arg;});
	}

	bool DynamicMetaBallModel::RemoveMetaball_If(std::vector<bool> deleted_flags)
	{
		const auto N = m_RestVolume.size();
		assert(deleted_flags.size() == N);
		auto itr = deleted_flags.begin();
		for (; itr != deleted_flags.end(); ++itr)
		{
			if (!*itr) break;
		}
		if (itr == deleted_flags.end())
		{
				Clear();
				return true;
		}
		auto origin = itr - deleted_flags.begin();
		deleted_flags = m_RestVolume.flood_fill(origin,deleted_flags);
		unsigned int k = 0;
		auto& Primitives = m_RestVolume.Primitives;
		Primitives.erase(std::remove_if(Primitives.begin(),Primitives.end(),[&deleted_flags,&k](const Metaball& ball)->bool{
			return !deleted_flags[k++];
		}),Primitives.end());
		auto& Colors = m_VolumeColors;
		k = 0;
		Colors.erase(std::remove_if(Colors.begin(),Colors.end(),[&deleted_flags,&k](const Metaball& ball)->bool{
			return !deleted_flags[k++];
		}),Colors.end());

		const auto Nr = Primitives.size();
		Eigen::MatrixXf Weights(Nr,m_VolumeWeights.cols());

		k = 0;
		for (size_t i = 0; i < N; i++)
		{
			if (deleted_flags[i])
			{
				Weights.row(k) = m_VolumeWeights.row(i);
				++k;
			}
		}

		m_VolumeWeights = Weights;
		SetDirty(VOLUME_DATA);
		return std::any_of(deleted_flags.begin(),deleted_flags.end(),[](bool arg)->bool{return arg;});
	}

	//	void DynamicMetaBallModel::InterpolateSkeleton(Kinematics::Joint* pHypotheticParent , float UniformRadius/* = 0.0f*/){
	//		m_pMetaBalls->clear();
	//		auto pNode = Skeleton->Root;
	//		if (pHypotheticParent == nullptr)
	//		{
	////			m_pMetaBalls->push_back(Metaball(pNode->Entity[Default].Position,Skeleton->Root->Radius,pNode->ID));
	//			float r = Skeleton->Root->Radius;
	//			if (UniformRadius >= 0.03f) r = UniformRadius;
	//			m_pMetaBalls->push_back(Metaball(pNode->Entity[Default].Position,r,pNode->ID));
	//
	//		}	else
	//			InterpolateOnBone(Skeleton->Root,pHypotheticParent,UniformRadius,UniformRadius);
	//
	//		pNode->for_all_in_sub_skeleton([&](Joint* pJoint)
	//		{
	//			if (pJoint!=pNode)
	//				InterpolateOnBone(pJoint , nullptr , UniformRadius , UniformRadius);
	//		});
	//		//	InterpolateSubSkeleton(Skeleton->root());
	//	}
	//
	//	void DynamicMetaBallModel::InterpolateSubSkeleton(Kinematics::Joint* pRoot){
	//		for (Joint* pJoint : pRoot->Children)
	//		{
	//			InterpolateOnBone(pJoint);
	//			InterpolateSubSkeleton(pJoint);
	//		}
	//	}

	void DynamicMetaBallModel::Update()
	{
		if (m_DirtyFlag & VOLUME_DATA)
		{
			m_RestVolume.Update();

			m_RestVolume.CreateConnectionGraph(m_Connections);

			m_AnimatedVolume = m_RestVolume;

			if (m_VolumeWeights.rows() != m_RestVolume.size())
			{
				if (m_pAdaptor)
				{
					m_pAdaptor->Alter(m_VolumeWeights);
				}
				else
				{
					m_VolumeWeights.resize(m_RestVolume.size(),1);
					m_VolumeWeights.setConstant(1.0f);
				}
			}

			if (m_VolumeColors.size() != m_RestVolume.size())
			{
				const DirectX::SimpleMath::Color DefaultColor = DirectX::Colors::Azure;
				m_VolumeColors.resize(m_RestVolume.size(),DefaultColor);
			}


			this->Tessellate();

			AnimationUpdate();
			m_DirtyFlag = 0;
			return;
		}
		if (m_DirtyFlag & WEIGHT_DATA)
		{
			if (m_pAdaptor)
			{
				//if (m_VolumeWeights.rows() != m_RestVolume.size() || m_VolumeWeights.cols() != m_pAdaptor->TranformMatricesCount())
				m_pAdaptor->Alter(m_VolumeWeights);
			}
			else
			{
				//if (m_VolumeWeights.rows() != m_RestVolume.size())
				//{
				m_VolumeWeights.resize(m_RestVolume.size(),1);
				m_VolumeWeights.setConstant(1.0f);
				//}
			}
			this->WeightingSkin(m_SkinMesh.Vertices_W());
			m_DirtyFlag = 0;
			return;
		}

	}


	void DynamicMetaBallModel::AnimationUpdate()
	{
		assert(m_AnimatedVolume.size() == m_RestVolume.size());
		auto N = m_AnimatedVolume.Primitives.size();
		if (!N || !m_pAdaptor) return;

		{
			auto &BlendMatrics = m_SkinMesh.ConstantBuffer_W().BlendMatrices;
			if (m_pAdaptor)
				m_pAdaptor->UpdateTranformMatrices(BlendMatrics);
		}

		auto& Weights = m_VolumeWeights;
		const auto& Matrices = m_SkinMesh.ConstantBuffer().BlendMatrices;
		const size_t M = m_VolumeWeights.cols();
		auto& outBalls = m_AnimatedVolume.Primitives;
		auto& inBalls = m_RestVolume.Primitives;

		for (size_t i = 0; i < N; i++)
		{
			outBalls[i].Position.Set(0.0f,0.0f,0.0f);
		}

		for (size_t bone = 0; bone < M; bone++)
		{
			XMMATRIX Transform = XMLoadFloat3x4(&Matrices[bone]);
			Transform = XMMatrixTranspose(Transform);

			for (size_t v = 0; v < N; v++)
			{
				if (Weights(v,bone) > .0f) {
					XMVECTOR vPos = XMVector3TransformCoord(inBalls[v].Position,Transform);
					vPos *= Weights(v,bone);
					outBalls[v].Position += vPos;
				}
			}
		}
		//{
		//	auto Bones = m_pAdaptor->TransformBones();
		//	if (!M) return;
		//	for (size_t v = 0; v < N; v++)
		//	{
		//		XMDUALVECTOR Transform = Bones[0];
		//		Transform *= Weights(v,0);

		//		for (size_t bone = 1; bone < M; bone++)
		//		{
		//			float w = Weights(v,bone);
		//			if (w > .0f)
		//			{
		//				XMDUALVECTOR Bone = Bones[bone];
		//				Bone *= Weights(v,bone);
		//				Transform += Bone;
		//			}
		//		}

		//		Transform = XMDualQuaternionNormalize(Transform);
		//		XMVECTOR vPos = inBalls[v].Position;
		//		vPos = XMVector3Displacement(vPos,Transform);
		//		outBalls[v].Position = vPos;
		//	}

		//}
	}


	void DynamicMetaBallModel::WeightingSkin(std::vector<DirectX::SkinVertex> &Vertices)
	{

#ifndef PARALLEL_UPDATE
		for (unsigned int i = 0; i < Vertices.size(); i++)
#else
		Concurrency::parallel_for<unsigned int>(0 , (unsigned int)Vertices.size() , 1  , [&](unsigned int i)
#endif // !PARALLEL_UPDATE
		{
			DirectX::Vector3 Pos(Vertices[i].Position);
			DirectX::XMVECTOR PosVtr = Pos;

			//		float BoneWeights[MAX_BLEND_MATRIX_COUNT];
			float Weights[4];
			UINT32 Idx[4];
			Weighting(PosVtr,Weights,Idx);
			//			Weights[0]=0.5f;Weights[1]=0.5f;Weights[2]=0.0f;Weights[3]=0.0f;
			//			Idx[0]=0;Idx[1]=1;Idx[2]=2;Idx[3]=3;
			for (unsigned int j = 0; j < VERTEX_BLEND_BONES; j++) {
				memcpy(reinterpret_cast<UINT32*>(&(Vertices[i].Indices)),Idx,sizeof(UINT32)*4);
				memcpy(reinterpret_cast<float*>(&(Vertices[i].Weights)),Weights,sizeof(float)*4);
			}
		}
#ifdef PARALLEL_UPDATE
		);
#endif
	}

	void DynamicMetaBallModel::MaippingTexture(std::vector<DirectX::SkinVertex> &vertrices)
	{
		for (unsigned int i = 0; i < vertrices.size(); i++)
		{
			XMVECTOR Pos = XMLoadFloat4(&vertrices[i].Position);
			vertrices[i].TexCoord = this->UVMapping(Pos);
		}
	}


	/// <summary>
	/// find intersection point of a Planes and a edge.
	/// </summary>
	/// <param name="P">The Plane equation.</param>
	/// <param name="V0">The first end point of edge.</param>
	/// <param name="V1">The second end point of edge.</param>
	/// <returns> return g_XMQNaN if not intersected. otherwise return the intersection point. </returns>
	inline XMVECTOR PlaneIntersectionEdge(FXMVECTOR P , FXMVECTOR V0 , FXMVECTOR V1)
	{
		XMVECTOR S = XMPlaneDot(P,V0) * XMPlaneDot(P,V1);
		if (XMVector4LessOrEqual(S,g_XMZero))
		{
			return XMPlaneIntersectLine(P,V0,V1);
		} else
		{
			return g_XMQNaN;
		}
	}

	inline bool VailadIntersection(FXMVECTOR P)
	{
		return XMVector4NotEqual(P,g_XMQNaN) && (XMVectorGetX(P)<0.0f);
	}


	//void DynamicMetaBallModel::ResoloveCrossBoundryTriangle()
	//{
	//	auto& vertecies = m_SkinMesh.Vertices_W();
	//	auto& indices	= m_SkinMesh.Indices_W();
	//	typedef std::tuple<unsigned int,unsigned int,unsigned int> Triangle;
	//	auto N_Triangle = indices.size() / 3;
	//	XMVECTOR vOrigin = m_RestVolume[0].Position;
	//	const auto & CriticalPlane = XMPlaneFromPointNormal(vOrigin,g_XMIdentityR2);

	//	for (size_t i = 0; i < N_Triangle; i++)
	//	{
	//		auto& Vex0 = vertecies[indices[i*3+0]];
	//		auto& Vex1 = vertecies[indices[i*3+1]];
	//		auto& Vex2 = vertecies[indices[i*3+2]];
	//		XMVECTOR v0 = XMLoadFloat4(&vertecies[indices[i*3+0]].Position);
	//		XMVECTOR v1 = XMLoadFloat4(&vertecies[indices[i*3+1]].Position);
	//		XMVECTOR v2 = XMLoadFloat4(&vertecies[indices[i*3+2]].Position);
	//		
	//		// Not Intersected case
	//		if (TriangleTests::Intersects(v0,v1,v2,CriticalPlane) != PlaneIntersectionType::INTERSECTING)
	//			continue;
	//		// Intersection case
	//		//float upPole,downPole;
	//		//if (TriangleTests::Intersects(vOrigin,g_XMIdentityR1,v0,v1,v2,upPole))
	//		//{
	//		//	Vector3 vPole(vOrigin + upPole*g_XMIdentityR1);
	//		//	Vector3 vNormal(XMVectorBaryCentric );
	//		//	vertecies.emplace_back(SkinVertex(vPole,
	//		//}


	//		XMVECTOR s0 = PlaneIntersectionEdge(CriticalPlane,v0,v1);
	//		XMVECTOR s1 = PlaneIntersectionEdge(CriticalPlane,v1,v2);
	//		XMVECTOR s2 = PlaneIntersectionEdge(CriticalPlane,v2,v0);
	//		bool bs0 = VailadIntersection(s0);
	//		bool bs1 = VailadIntersection(s1);
	//		bool bs2 = VailadIntersection(s2);
	//		if (bs0 || bs1 || bs2)
	//		{
	//			indices[i*3+0]=0;
	//			indices[i*3+1]=0;
	//			indices[i*3+2]=0;
	//		}

	//		//{
	//		//	
	//		//} else
	//		//{
	//		//	continue;
	//		//}

	//	}
	//}



	//void DynamicMetaBallModel::AppendModel(const DynamicMetaBallModel* pSrc , Joint* pAppedTarget , bool IsRelative , float ConnectionSRadius /*= 0.0f*/, float ConnectionERadius/*= 0.0f*/)
	//{
	//	if (pSrc->Skeleton->Root==nullptr)
	//		return ;

	//	auto mapper = this->Skeleton->AppendSubSkeleton(pAppedTarget,pSrc->Skeleton->Root,IsRelative);

	//	int m = m_pMetaBalls->size();
	//	m_pMetaBalls->Primitives.insert(m_pMetaBalls->Primitives.end(),pSrc->Flesh->Primitives.begin(),pSrc->Flesh->Primitives.end());
	//	for (unsigned int i = m; i < m_pMetaBalls->size(); i++)
	//	{
	//		(*m_pMetaBalls)[i].BindingIndex = (*mapper)[(*m_pMetaBalls)[i].BindingIndex];
	//		if (IsRelative) {
	//			XMVECTOR Ptr = (*m_pMetaBalls)[i].Position;
	//			Ptr = Vector3::Rotate(Ptr,pAppedTarget->Entity[Default].Orientation) + pAppedTarget->Entity[Default].Position;
	//			(*m_pMetaBalls)[i].Position = Ptr;
	//		}
	//	}
	//	if (pAppedTarget == nullptr)
	//	{
	//		InterpolateOnBone((*this->Skeleton)[(*mapper)[pSrc->Skeleton->Root->ID]],nullptr,ConnectionSRadius,ConnectionERadius);
	//	}
	//}

	void Geometrics::DynamicMetaBallModel::Weighting( DirectX::FXMVECTOR PosVtr,_Out_writes_(4) float* BlendWeights ,_Out_writes_(4) UINT32 * BlendIndices ) const
	{
		const size_t M = m_VolumeWeights.cols();

		Eigen::VectorXf BoneWeights(M);
		BoneWeights.setZero();
		//memset(BoneWeights.data(),0,sizeof(float)*M);
		memset(BlendWeights,0,sizeof(float)*VERTEX_BLEND_BONES);
		memset(BlendIndices,0,sizeof(UINT32)*VERTEX_BLEND_BONES);

		for (unsigned int v = 0; v < m_RestVolume.size(); v++)
		{
			float Weight = m_RestVolume[v].eval(PosVtr);
			if (Weight > 0.0f){
				BoneWeights += Weight * m_VolumeWeights.row(v);
				//for (size_t i = 0; i < M; i++)
				//{
				//	BoneWeights(i) += Weight * m_VolumeWeights(j,i);
				//}
			}
		}

		for (UINT32 j = 0; j < M; j++)
		{
			float Weight = BoneWeights(j);
			if (BlendWeights[VERTEX_BLEND_BONES -1] < Weight)
			{
				int k=VERTEX_BLEND_BONES-2;
				while ((k >= 0)&&(BlendWeights[k] < Weight))
				{
					BlendWeights[k+1] = BlendWeights[k];
					BlendIndices[k+1] = BlendIndices[k];
					--k;
				}
				++k;
				BlendWeights[k] = Weight;
				BlendIndices[k] = j;
			}
		}

		float SumWeight = 0.0f;
		for (unsigned int j = 0; j < VERTEX_BLEND_BONES; j++) {
			SumWeight += BlendWeights[j];
		}
		//		DirectX::XMStoreFloat4(reinterpret_cast<DirectX::XMFLOAT4*>(BlendWeights),DirectX::XMLoadFloat4(reinterpret_cast<DirectX::XMFLOAT4*>(BlendWeights))/SumWeight);
		if (SumWeight < DirectX::XM_EPSILON)
		{
			BlendWeights[0] = 1.0f;
			BlendWeights[1] = 0.0f;
			BlendWeights[2] = 0.0f;
			BlendWeights[3] = 0.0f;
			return;
		}
		SumWeight = 1.0f/SumWeight;
		for (unsigned int j = 0; j < VERTEX_BLEND_BONES; j++)
		{
			BlendWeights[j]*=SumWeight;
		}

	}

	DirectX::XMVECTOR Geometrics::DynamicMetaBallModel::DeformVertex( DirectX::FXMVECTOR Vtr ) const
	{
		float Weights[VERTEX_BLEND_BONES];
		unsigned int Idx[VERTEX_BLEND_BONES];
		Weighting(Vtr,Weights,Idx);
		return DeformVertex(Vtr,Weights,Idx);
	}

	Geometrics::DynamicMetaBallModel::DynamicMetaBallModel(ID3D11Device *pDevice , float TessalationPreciese)
		: m_SkinMesh(pDevice , TextureSkinMesh::Default_Texture_Resolution)
		, m_TessalationPrecise(TessalationPreciese)
	{
	}

	Geometrics::DynamicMetaBallModel::DynamicMetaBallModel()
		: m_TessalationPrecise(.0f)
	{
	}

	//void Geometrics::DynamicMetaBallModel::Scale(float scale , DirectX::FXMVECTOR Origin)
	//{

	//	XMVECTOR vDisp = (Origin*(1-scale));
	//	Vector3 Disp(vDisp);

	//	for (auto& index : m_pSkeleton->Index)
	//	{
	//		auto& pJoint = index.second;
	//		if(!pJoint->is_root()) {
	//			pJoint->Entity[Current].Scale.y *= scale;
	//			pJoint->Entity[Default].Scale.y *= scale;
	//		}
	//	}
	//	m_pSkeleton->Root->Entity[Current].Position *= scale;
	//	m_pSkeleton->Root->Entity[Current].Position += Disp;

	//	m_pSkeleton->Root->Entity[Default].Position *= scale;
	//	m_pSkeleton->Root->Entity[Default].Position += Disp;

	//	m_pSkeleton->Update();

	//	m_pSkeleton->Root->ReBuildSubSkeleton_Global_from_Hierarchical(Default);
	//	//m_pSkeleton->Snap_Default_to_Current();

	//	for (auto& ball : m_pMetaBalls->Primitives)
	//	{
	//		ball.Position = scale*ball.Position + Disp;
	//		ball.Radius *= scale;
	//	}

	//	for (auto& vertex : m_pSkinMesh->Vertices_W())
	//	{
	//		XMVECTOR vPos = XMLoadFloat4(&vertex.Position);
	//		vPos *= scale;
	//		vPos += vDisp;
	//		XMStoreFloat4(&vertex.Position,vPos);
	//		vertex.Position.w = 1.0f;
	//	}

	//	//Tessellate();
	//}

	//void Geometrics::DynamicMetaBallModel::Rotate(DirectX::FXMVECTOR qRotation , DirectX::FXMVECTOR vRotationCenter)
	//{
	//	m_pSkeleton->Root->Entity[Current].Orientation = XMQuaternionMultiply(m_pSkeleton->Root->Entity[Current].Orientation,qRotation);
	//	auto UnRotated = m_pSkeleton->Root->Entity[Current].Position - vRotationCenter;
	//	auto Rotated = XMVector3Rotate(UnRotated,qRotation);
	//	m_pSkeleton->Root->Entity[Current].Position = m_pSkeleton->Root->Entity[Current].Position + Rotated - UnRotated;
	//	m_pSkeleton->Update();
	//	//AnimationUpdate();
	//}

	//void Geometrics::DynamicMetaBallModel::RotateTo(DirectX::FXMVECTOR qTargetOrientation , DirectX::FXMVECTOR vRotationCenter)
	//{
	//	auto qRotation = XMQuaternionMultiply(XMQuaternionConjugate(m_pSkeleton->Root->Entity[Current].Orientation),qTargetOrientation);
	//	m_pSkeleton->Root->Entity[Current].Orientation = qTargetOrientation;
	//	auto UnRotated = m_pSkeleton->Root->Entity[Current].Position - vRotationCenter;
	//	auto Rotated = XMVector3Rotate(UnRotated,qRotation);
	//	m_pSkeleton->Root->Entity[Current].Position = m_pSkeleton->Root->Entity[Current].Position + Rotated - UnRotated;
	//	m_pSkeleton->Update();
	//	//AnimationUpdate();
	//}

	//void Geometrics::DynamicMetaBallModel::Ground(float GroundHeight)
	//{
	//	this->AnimationUpdate();
	//	auto& Model = this->GetAnimatedMetaballModel();
	//	auto lowestVertex = std::min_element( Model.cbegin() ,  Model.cend(), 
	//		[](const Geometrics::Metaball& lhs , const Geometrics::Metaball& rhs ) -> bool 
	//	{
	//		return lhs.Position.y - lhs.Radius < rhs.Position.y - rhs.Radius;
	//	});

	//	auto displacement = GroundHeight - (lowestVertex->Position.y - lowestVertex->Radius);
	//	auto Origin = this->Position;
	//	Origin.y += displacement;
	//	this->Position = Origin;
	//	m_pSkeleton->Update();
	//}


	Geometrics::DynamicMetaBallModel::~DynamicMetaBallModel( void )
	{
	}

	DirectX::XMVECTOR DynamicMetaBallModel::DeformVertex(DirectX::FXMVECTOR Vtr,const float* BlendWeights ,const UINT32 * BlendIndices) const
	{
		XMVECTOR Ptr = XMVectorZero();
		for (size_t i = 0; i < VERTEX_BLEND_BONES; i++)
		{
			XMMATRIX transform = XMLoadFloat3x4(&m_SkinMesh.ConstantBuffer().BlendMatrices[BlendIndices[i]]);
			transform = XMMatrixTranspose(transform);
			XMVECTOR vPos = XMVector3TransformCoord(Vtr,transform);
			vPos *= BlendWeights[i];
			Ptr += vPos;
		}
		return Ptr;
	}

	//void DynamicMetaBallModel::ResetLocalTransform()
	//{
	//	auto RootInfo = Skeleton->Root->Entity[Current];
	//	Skeleton->Snap_Current_to_Default();
	//	Skeleton->Root->Entity[Current] = RootInfo;
	//	Skeleton->Update();
	//}



	void DynamicMetaBallModel::LoadPackage(Package&& package)
	{
		m_RestVolume.Primitives = std::move(package.Metaballs);
		m_VolumeWeights = std::move(package.VolumeWeights);
		m_SkinMesh.SetTexture(std::shared_ptr<Texture2D>(package.pTexture.release()));
		//m_SkinMesh.Texture() = std::move(package.pTexture);
		SetDirty(VOLUME_DATA);
		this->Update();
	}

	void DynamicMetaBallModel::LoadPackage(ID3D11DeviceContext *pDeviceContext, const Package& package)
	{
		m_RestVolume.Primitives = package.Metaballs;
		m_VolumeWeights = package.VolumeWeights;
		//*this->Skeleton = package.Skeleton;
		m_SkinMesh.Texture()->CopyFrom(pDeviceContext,package.pTexture.get());
		//this->Skin->Texture_W() = package.Texture;
		SetDirty(VOLUME_DATA);
		this->Update();
	}

	DynamicMetaBallModel::Package DynamicMetaBallModel::SavePackage(ID3D11DeviceContext *pDeviceContext)
	{
		Package package(pDeviceContext,m_RestVolume.Primitives,m_VolumeWeights,m_SkinMesh.Texture());
		return std::move(package);
	}

	DynamicMetaBallModel::Package::Package(Package && rhs)
		: Metaballs(std::move(rhs.Metaballs))
		, VolumeWeights(std::move(rhs.VolumeWeights))
		//, Skeleton(std::move(rhs.Skeleton))
		, pTexture(std::move(rhs.pTexture))
	{
	}

	DynamicMetaBallModel::Package& DynamicMetaBallModel::Package::operator=(DynamicMetaBallModel::Package&& rhs)
	{
		Metaballs = std::move(rhs.Metaballs);
		VolumeWeights = std::move(rhs.VolumeWeights);
		//Skeleton = std::move(rhs.Skeleton);
		pTexture = std::move(rhs.pTexture);
		return *this;
	}

	ISerializable::Blob DynamicMetaBallModel::Package::Serialize() const
	{
		Blob blob;
		//auto SkBlob = Skeleton.Serialize();
		blob.size = sizeof(size_t) + sizeof(Metaball) * Metaballs.size();// + SkBlob.size /*+ sizeof(DXGI_FORMAT) + sizeof(size_t) * 2 + Texture.SizeInBytes()*/;
		blob.pData = (unsigned char*)malloc(blob.size);
		if (blob.pData == nullptr) return blob;
		*(size_t*)blob.pData = Metaballs.size();
		memcpy(blob.pData + sizeof(size_t),&Metaballs[0],sizeof(Metaball) * Metaballs.size());
		//memcpy(blob.pData + sizeof(size_t) + sizeof(Metaball) * Metaballs.size(),SkBlob.pData,SkBlob.size);
		//auto pTexF = (DXGI_FORMAT*)(blob.pData + sizeof(size_t) + sizeof(Metaball) * Metaballs.size() + SkBlob.size);
		//*pTexF++ = Texture.Format();
		//auto pTex = reinterpret_cast<size_t*>(++pTexF);
		//*pTex = Texture.Width();
		//*(++pTex) = Texture.Height();
		//memcpy(++pTex,Texture.RawData(),Texture.SizeInBytes());
		return blob;
	}

	void DynamicMetaBallModel::Package::Deserialize(const Blob& blob){

		auto size_m = *(size_t*)blob.pData;
		Metaballs.assign((Metaball*)(blob.pData + sizeof(size_t)), (Metaball*)(blob.pData + sizeof(size_t)) + size_m);

		Blob SkBlob;
		SkBlob.pData = blob.pData + sizeof(size_t) + sizeof(Metaball) * Metaballs.size();
		unsigned int size_j = *(size_t*)SkBlob.pData;
		SkBlob.size = sizeof(size_t) + size_j * (sizeof(KinematicsData[2]) + sizeof(unsigned int));
		//Skeleton.Deserialize(SkBlob);

		//auto pTex = (size_t*)(blob.pData + sizeof(size_t) + sizeof(Metaball) * Metaballs.size() + SkBlob.size);
		//// Previous version save data
		//if (*pTex == 1024)
		//{
		//	Texture.Resize(*pTex,*(++pTex),DXGI_FORMAT::DXGI_FORMAT_R8G8B8A8_UNORM);
		//	Texture.FromVectorStream(reinterpret_cast<XMFLOAT4*>(++pTex));
		//}else
		//{
		//	auto pTexF = reinterpret_cast<DXGI_FORMAT*>(pTex);
		//	pTex = reinterpret_cast<size_t*>(pTexF + 1);
		//	Texture.Resize(*pTex,*(++pTex) , *pTexF);
		//	Texture.Fill(reinterpret_cast<XMFLOAT4*>(++pTex));
		//}
	}


}
