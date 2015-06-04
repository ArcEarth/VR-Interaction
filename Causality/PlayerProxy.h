#pragma once
#include "SceneObject.h"
#include "Kinect.h"
#include <boost\circular_buffer.hpp>

namespace Causality
{
	using boost::circular_buffer;

	class PlayerProxy : public SceneObject, public IRenderable
	{
	public:
		typedef AffineFrame frame_type;

		class ControlState
		{
		public:

			const ArmatureTransform& Binding() const;
			ArmatureTransform& Binding();
			void SetBinding(ArmatureTransform* pBinding);

			const KinematicSceneObject& Target() const;
			KinematicSceneObject& Target();

			void SetSourceArmature(const IArmature& armature);

			void SetTargetObject(KinematicSceneObject& object);

			int						ID;
			AffineFrame				PotientialFrame;
			float					SpatialMotionScore;

		private:
			KinematicSceneObject*		  m_pSceneObject;
			unique_ptr<ArmatureTransform> m_pBinding;
		};

		struct StateChangedEventArgs
		{
			int OldStateIndex;
			int NewStateIndex;
			float Confidence;
			ControlState& OldState;
			ControlState& NewState;
		};

		PlayerProxy();
		virtual ~PlayerProxy() override;
		virtual void AddChild(SceneObject* pChild) override;

		void Initialize();

		// Enter the selecting phase
		void BeginSelectingPhase();

		// End the selecting phase and enter the manipulating phase
		void BeginManipulatingPhase();

		int SelectControlStateByLatestFrames();

		bool IsMapped() const { return CurrentIdx >= 0; }
		const ControlState& CurrentState() const { return States[CurrentIdx]; }
		ControlState& CurrentState() { return States[CurrentIdx]; }
		const ControlState&	GetState(int state) const { return States[state]; }

		Event<const StateChangedEventArgs&> StateChanged;

		void	Update(time_seconds const& time_delta) override;
		bool	UpdatePlayerFrame(const AffineFrame& frame);

		const IArmature& BodyArmature() const { return *pPlayerArmature; };

		// Inherited via IRenderable
		virtual bool IsVisible(const BoundingFrustum & viewFrustum) const override;
		virtual void Render(RenderContext & context) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

		std::pair<float, float> ExtractUserMotionPeriod();

		// Body Connection management
		bool IsConnected() const { return pBody != nullptr; }
		void Connect(TrackedBody* body) { 
			ResetConnection();
			pBody = body; 
			if (pBody)
				pBody->AddRef();
		}
		TrackedBody* ConnectedBody() const { return pBody; }
		void ResetConnection() { 
			if (pBody)
			{
				pBody->Release();
				pBody = nullptr;
			}
		}

	protected:
		bool								IsInitialized;

		const IArmature*					pPlayerArmature;
		int									Id;

		std::shared_ptr<Devices::Kinect>	pKinect;
		TrackedBody*						pBody;
		

		int									CurrentIdx;
		std::vector<ControlState>			States;

		static const size_t					FrameRate = ANIM_STANDARD::SAMPLE_RATE;
		static const size_t					ScaledMotionTime = ANIM_STANDARD::MAX_CLIP_DURATION; // second
		static const size_t					ScaledFramesCount = ANIM_STANDARD::CLIP_FRAME_COUNT;
		static const size_t					BufferTime = 10; // second
		static const size_t					BufferFramesCount = FrameRate * BufferTime;
		static const size_t					JointCount = JointType_Count;
		static const size_t					JointDemension = FeatureType::Dimension; // X-Y-Z Position
		static const size_t					MinHz = 3; // MinimumFrequency 3/BufferTime Hz
		static const size_t					MaxHz = 10; // 10 Hz
		static const size_t					HzWidth = MaxHz - MinHz + 1;

		circular_buffer<frame_type>			FrameBuffer;

		VectorX								StateProbality;
		VectorX								Likilihood;
		MatrixX								TransferMatrix;

	protected:
		void PrintFrameBuffer(int No);
	};

	struct SkeletonBlock
	{
		DirectX::XMVECTOR Scale;
		DirectX::XMVECTOR CenterFromParentCenter;
		JointType SkeletonJoint;
		int ParentBlockIndex;
	};

	const SkeletonBlock g_SkeletonBlocks [] = {
		{ { 0.75f, 1.0f, 0.5f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f }, JointType_SpineMid, -1 },      //  0 - lower torso
		{ { 0.75f, 1.0f, 0.5f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_SpineShoulder, 0 },  //  1 - upper torso

		{ { 0.25f, 0.25f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_Neck, 1 },         //  2 - neck
		{ { 0.5f, 0.5f, 0.5f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_Head, 2 },            //  3 - head

		{ { 0.35f, 0.4f, 0.35f, 0.0f },{ 0.5f, 1.0f, 0.0f, 0.0f }, JointType_ShoulderLeft, 1 },  //  4 - Left shoulderblade
		{ { 0.25f, 1.0f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_ElbowLeft, 4 },     //  5 - Left upper arm
		{ { 0.15f, 1.0f, 0.15f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_WristLeft, 5 },     //  6 - Left forearm
		{ { 0.10f, 0.4f, 0.30f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_HandLeft, 6 },      //  7 - Left hand

		{ { 0.35f, 0.4f, 0.35f, 0.0f },{ -0.5f, 1.0f, 0.0f, 0.0f }, JointType_ShoulderRight, 1 },//   8 - Right shoulderblade
		{ { 0.25f, 1.0f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_ElbowRight, 8 },    //   9 - Right upper arm
		{ { 0.15f, 1.0f, 0.15f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_WristRight, 9 },    //  10 - Right forearm
		{ { 0.10f, 0.4f, 0.30f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_HandRight, 10 },    //  11 - Right hand

		{ { 0.35f, 0.4f, 0.35f, 0.0f },{ 0.5f, 0.0f, 0.0f, 0.0f }, JointType_HipLeft, 0 },       //  12 - Left hipblade
		{ { .4f, 1.0f, .4f, 0.0f },{ 0.5f, 0.0f, 0.0f, 0.0f }, JointType_KneeLeft, 12 },         //  13 - Left thigh
		{ { .3f, 1.0f, .3f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_AnkleLeft, 13 },        //  14 - Left calf
		//{ { .35f, .6f, .20f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, JointType_FootLeft , 14 },     //     - Left foot

		{ { 0.35f, 0.4f, 0.35f, 0.0f },{ -0.5f, 0.0f, 0.0f, 0.0f }, JointType_HipRight, 0 },     //  15  - Right hipblade
		{ { .4f, 1.0f, .4f, 0.0f },{ -0.5f, 0.0f, 0.0f, 0.0f }, JointType_KneeRight, 15 },       //  16  - Right thigh
		{ { .3f, 1.0f, .3f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_AnkleRight, 16 },       //  17  - Right calf
		//{ { .35f, .6f, .20f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, JointType_FootRight, 18 },     //      - Right foot

	};

	const UINT BLOCK_COUNT = _countof(g_SkeletonBlocks);
	const UINT BODY_COUNT = 6;
	const UINT JOINT_COUNT = 25;

	class KinectVisualizer : public SceneObject, public IRenderable
	{
	public:
		KinectVisualizer();
		// Inherited via IRenderable
		virtual bool IsVisible(const BoundingFrustum & viewFrustum) const override;
		virtual void Render(RenderContext & context) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

	protected:
		std::shared_ptr<Devices::Kinect>	pKinect;
	};
}