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

	protected:
		const IArmature*					pPlayerArmature;
		int									PlayerID;
		std::shared_ptr<Devices::Kinect>	pKinect;
		int									CurrentIdx;
		std::vector<ControlState>			States;

		circular_buffer<frame_type>			FrameBuffer;
		Eigen::Matrix<float, 450, JointType::JointType_Count * 3> 
											MotionBuffer;
		Eigen::Matrix<std::complex<float>, 450, JointType::JointType_Count * 3> 
											MotionSpecturm;
		VectorX								StateProbality;
		VectorX								Likilihood;
		MatrixX								TransferMatrix;
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