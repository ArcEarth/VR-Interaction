#pragma once
#include "MetaBallModel.h"
#include "KinematicsSkeleton.h"
#include "SkinMesh.h"
#include "BitMap.h"
#include "MotionAdaptor.h"

//#define REMESH_EVERY_FRAME
namespace Geometrics
{
	// 2.0f equals to diameter
	extern const float Metaball_Spare_Factor;

	extern const float Metaball_Radius_Superior_Limit;
	extern const float Metaball_Radius_Infieror_Limit;

	class DynamicMetaBallModel
		//: public DirectX::IRenderObject , public DirectX::ITextureObject , public DirectX::ISkinObject
	{
	public:
		enum RenderMode
		{
			SKELETON_DEFAULT,
			SKELETON_ANIMATED,
			METABALLS_DEFAULT,
			METABALLS_ANIMATED,
			SKIN_DEFAULT,
			SKIN_ANIMATED,
		};
		enum DirtyFlagEnum
		{
			VOLUME_DATA = 0x1,
			WEIGHT_DATA = 0x2,
			TEXTURE_DATA= 0x4,
		};
		//enum PrimitiveState
		//{
		//	FLAG_NORMAL,
		//	FLAG_DELETED,
		//};

	public:
		struct Package
			: ISerializable
		{
			std::vector<Metaball>					Metaballs;
			Eigen::MatrixXf							VolumeWeights;
			//Kinematics::IndexedSkeleton				Skeleton;
			//DirectX::Bitmap Texture;
			std::unique_ptr<DirectX::StagingTexture2D> pTexture;
			
			Package(){}
			~Package(){}
			Package(ID3D11DeviceContext *pDeviceContext, const std::vector<Metaball>& _Metaballs , const Eigen::MatrixXf& _VolumeWeights , const DirectX::Texture2D* _Texture)
				: Metaballs(_Metaballs) , VolumeWeights(_VolumeWeights) , pTexture(new DirectX::StagingTexture2D(pDeviceContext,_Texture))
			{}
			//Package(const std::vector<Metaball>& _Metaballs ,const Kinematics::IndexedSkeleton& _Skeleton , const DirectX::Bitmap& _Texture)
			//	: Metaballs(_Metaballs) , Skeleton(_Skeleton) , Texture(_Texture)
			//{}
			Package(Package&& rhs);
			Package& operator= (Package && rhs);

			// Format : 
			// Type				|	Data
			// size_t			|	MetaballCount
			// Metaball[]		|	Metaballs
			// size_t			|	JointsCount
			// KinematicsData[]	|	JointsState
			// uint[]			|	JointsParents
			// size_t			|	TextureWidth
			// size_t			|	TextureHeight
			// PixelType[]		|	TextureRawData
			virtual Blob Serialize() const;
			virtual void Deserialize(const Blob& Data); 
		private:
			Package(const Package&);
			void operator = (const Package& );
		};

	public:
		DynamicMetaBallModel();
		DynamicMetaBallModel(ID3D11Device*pDevice , float TessalationPreciese);
		DynamicMetaBallModel(ID3D11Device*pDevice , float TessalationPreciese , const Metaball& Model);
		DynamicMetaBallModel(ID3D11Device*pDevice , float TessalationPreciese , Metaball&& Model);

		~DynamicMetaBallModel(void);

		//DirectX::Quaternion getOrientation() const;
		//void setOrientation(const DirectX::Quaternion &NewOrientation) const;

		//DirectX::Vector3 getPosition() const;
		//void setPosition(const DirectX::Vector3& NewPosition);

		//__declspec(property(get = getOrientation , put = setOrientation))
		//	DirectX::Quaternion Orientation;

		//__declspec(property(get = getPosition , put = setPosition))
		//	DirectX::Vector3 Position;

		// IRenderObject Interface
		// Render the Avatar Model to screen
		//virtual void Render(ID3D11DeviceContext *pContext);
		
		// ITextureObject Interface
		DirectX::XMFLOAT2 UVMapping(DirectX::FXMVECTOR VertexPosition) const;

		// ISkinningObject Interface
		void Weighting(DirectX::FXMVECTOR PosVtr,_Out_writes_(4) float* BlendWeights ,_Out_writes_(4) uint32_t * BlendIndices) const;

		//// Reset the local state of this model to default state
		//void ResetLocalTransform();

		void Update();
		// Tessellate the metaball model into triangles , the data is auto buffered and rendered at next render call
		void Tessellate();
		// Snap the default skeleton into current skeleton , and drive the metaballs so , and then tessellate it.
		void Remesh();

		// Clear all the data
		void Clear();

		// Reset data
		void Reset(const Metaball& Model);
		void Reset(Metaball&& Model);

		unsigned int SetDirty(DirtyFlagEnum dirtyFalg);

		bool Empty() const {return m_RestVolume.empty();}
		
		inline std::shared_ptr<const Kinematics::IMotionAdaptor> Adaptor() const{return m_pAdaptor;}
		inline std::shared_ptr<Kinematics::IMotionAdaptor>& Adaptor(){return m_pAdaptor;}
		inline const MetaBallModel& Volume() const{	return m_AnimatedVolume;}
		inline MetaBallModel& RestVolume(){	return m_RestVolume;}
		inline std::vector<DirectX::SimpleMath::Color> const & Colors() const { return m_VolumeColors;}
		inline std::vector<DirectX::SimpleMath::Color>& Colors() { return m_VolumeColors;}
		inline const ConnectionGraph& Connections() const { return m_Connections; }
		inline const DirectX::TextureSkinMesh& Skin() const {return m_SkinMesh; }
		inline DirectX::TextureSkinMesh& Skin() {return m_SkinMesh; }
		inline Eigen::MatrixXf& VolumeWeights() {return m_VolumeWeights;}
		inline const Eigen::MatrixXf& VolumeWeights() const {return m_VolumeWeights;}


		// update the Animated metaball Model & Skeleton data
		void AnimationUpdate();

		void Disconnect();

		void Connect(const std::shared_ptr<Kinematics::IMotionAdaptor>& pAdaptor);

		//const MetaBallModel& GetAnimatedMetaballModel() const;

		void LoadPackage(Package&& package);
		void LoadPackage(ID3D11DeviceContext *pDeviceContext,const Package& package);
		Package SavePackage(ID3D11DeviceContext *pDeviceContext);

		///// Scaling with really update the vertices and metaballs and skeletons
		//void Scale(float scale , DirectX::FXMVECTOR Origin);

		///// <summary>
		///// Rotate the Model in World Space
		///// </summary>
		///// <param name="qRotation">The q rotation.</param>
		///// <param name="vRotationCenter">The v rotation center.</param>
		//void Rotate(DirectX::FXMVECTOR qRotation , DirectX::FXMVECTOR vRotationCenter);
		///// <summary>
		///// Rotate the Model to a specific orientation in World Space
		///// </summary>
		///// <param name="qTargetOrientation">The q target orientation.</param>
		///// <param name="vRotationCenter">The v rotation center.</param>
		//void RotateTo(DirectX::FXMVECTOR qTargetOrientation , DirectX::FXMVECTOR vRotationCenter);
		///// <summary>
		///// Ground the Model to a specific height in World Space
		///// </summary>
		///// <param name="GroundHeight">Height of the ground.</param>
		//void Ground(float GroundHeight);

		DirectX::XMMATRIX TransformMatrix(std::vector<float> WeightVector) const;
		// Deform the Volume from world space into modeling space
		//std::vector<std::vector<float>> InverseDeformModel(_Inout_ MetaBallModel &Volume);
		std::vector<float> InverseDeformMetaball(Metaball& Ball) const;

		// Methods for apply geometry change
		void AddSphere(const DirectX::BoundingSphere &Sphere);
		void AddMetaball(const Metaball &Ball);
		void AddCylinder(DirectX::FXMVECTOR p0, DirectX::FXMVECTOR p1 , float R0 , float R1 , bool ExceptFront = false);
		void AddStroke(std::vector<DirectX::Vector3> StrokePath , float Radius);
		void AddVolume(const MetaBallModel &Volume , std::function<void(MetaBallModel &,Eigen::MatrixXf&)> RetesslateionFunc = nullptr);
		//void AddSurface();
		void RemoveMetaball(size_t index);
		bool RemoveMetaball_If(std::function<bool(unsigned int)> Pred);
		bool RemoveMetaball_If(std::vector<bool> deleted_falgs);
		// Apply a transform to entire model
		void Transform(DirectX::CXMMATRIX M);

		int CheckConsist() const;
		//void InterpolateSubSkeleton(Kinematics::Joint* pRoot);
		//void InterpolateOnBone(Kinematics::Joint* pJoint , Kinematics::Joint* pHypotheticParent = nullptr , float SRadius = 0.0f, float ERadius = 0.0f);
		//void InterpolateSkeleton(Kinematics::Joint* pHypotheticParent = nullptr , float UniformRadius = 0.0f);

		// Obstacle old method
		//void AppendModel(const DynamicMetaBallModel* pSrc , Kinematics::Joint* pAppedTarget , bool IsRelative = false , float ConnectionSRadius = 0.0f, float ConnectionERadius = 0.0f);
		//void AppendRigidModel(const MetaBallModel * pSrc);

		DirectX::XMVECTOR DeformVertex(DirectX::FXMVECTOR Vtr) const;

	protected:
		DirectX::XMVECTOR DeformVertex(DirectX::FXMVECTOR Vtr,const float* BlendWeights ,const uint32_t * BlendIndices) const;
		// Using this method to get a weight distribute to your vertex
		// your Vertex must have the member "uint4 blendIndices" and "float4 blendWeights"
		void WeightingSkin(std::vector<DirectX::SkinVertex> &Vertices);
		void MaippingTexture(std::vector<DirectX::SkinVertex> &vertrices);
		//void ResoloveCrossBoundryTriangle();
	protected:	

		MetaBallModel					m_RestVolume;
		MetaBallModel					m_AnimatedVolume;
		ConnectionGraph					m_Connections;
		
		Eigen::MatrixXf					m_VolumeWeights;
		std::vector<DirectX::SimpleMath::Color> m_VolumeColors;
		//std::vector<std::vector<float>> m_VolumeWeights;

		std::shared_ptr<Kinematics::IMotionAdaptor>
										m_pAdaptor;
		DirectX::TextureSkinMesh		m_SkinMesh;
		float							m_TessalationPrecise;
		unsigned int					m_DirtyFlag;
	public:

		//__declspec(property(get = getAnimatedVolume)) 
		//	const MetaBallModel&  Volume;

		//// Return the volume in default space
		//__declspec(property(get = getRestVolume)) 
		//	MetaBallModel&  RestVolume;

		//__declspec(property(get = getSkinMesh))
		//	DirectX::TextureSkinMesh& Skin;
		////__declspec(property(get = getSkinMesh))
		////	const DirectX::TextureSkinMesh& Skin;

		//__declspec(property(get = getConnections))
		//	const ConnectionGraph& Connections;

		//const ConnectionGraph& getConnections() const { return m_Connections; }
		//const MetaBallModel& getAnimatedVolume() const { return m_AnimatedVolume; }
		//const DirectX::TextureSkinMesh& getSkinMesh() const {return m_SkinMesh; }
		//DirectX::TextureSkinMesh& getSkinMesh() {return m_SkinMesh; }
		//MetaBallModel& getRestVolume() { return m_RestVolume; }
	};
}
