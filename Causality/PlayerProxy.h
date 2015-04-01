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
		typedef BoneDisplacementFrame frame_type;


		class ControlState
		{
		public:

			const ArmatureTransform& Binding() const;
			ArmatureTransform& Binding();

			const KinematicSceneObject& Object() const;
			KinematicSceneObject& Object();

			void SetSourceArmature(const IArmature& armature);

			void SetTargetObject(KinematicSceneObject& object);


			int						ID;
			BoneDisplacementFrame	PotientialFrame;

		private:
			KinematicSceneObject*	m_pSceneObject;
			ArmatureTransform		m_Binding;
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

		void Initialize();

		// Enter the selecting phase
		void BeginSelectingPhase();

		// End the selecting phase and enter the manipulating phase
		void BeginManipulatingPhase();

		bool IsIdel() const { return CurrentIdx == 0; }
		const ControlState& CurrentState() const { return States[CurrentIdx]; }
		ControlState& CurrentState() { return States[CurrentIdx]; }
		const ControlState&	GetState(int state) const { return States[state]; }

		Event<const StateChangedEventArgs&> StateChanged;

		void	Update(time_seconds const& time_delta) override;
		bool	UpdatePlayerFrame(const BoneDisplacementFrame& frame);

		const IArmature&					PlayerArmature() const { return *pPlayerArmature; };

		// Inherited via IRenderable
		virtual bool IsVisible(const BoundingFrustum & viewFrustum) const override;
		virtual void Render(RenderContext & context) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

		std::pair<float, float> ExtractUserMotionPeriod();
	protected:

		const IArmature*					pPlayerArmature;
		int									PlayerID;
		std::shared_ptr<Devices::Kinect>	pKinect;
		int									CurrentIdx;
		std::vector<ControlState>			States;

		static const size_t					FrameRate = 30;
		static const size_t					BufferTime = 15; // second
		static const size_t					BufferFramesCount = FrameRate * BufferTime;
		static const size_t					JointCount = JointType_Count;
		static const size_t					JointDemension = 3; // X-Y-Z Position
		static const size_t					MinimumFrequency = 3; // 3/BufferTime Hz
		static const size_t					MaximumFrequency = BufferTime; // 1 Hz
		static const size_t					BandWidth = MaximumFrequency - MinimumFrequency + 1;

		circular_buffer<frame_type>			FrameBuffer;

		VectorX								StateProbality;
		VectorX								Likilihood;
		MatrixX								TransferMatrix;

	protected:
		void PrintFrameBuffer(int No);
	};

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