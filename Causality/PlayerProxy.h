#pragma once
#include "CharacterObject.h"
#include "Kinect.h"
#include <boost\circular_buffer.hpp>
#include "Animations.h"
#include "CharacterController.h"
#include <atomic>

namespace Causality
{
	using boost::circular_buffer;

	// Player : public Character
	class PlayerProxy : public SceneObject, public IRenderable, public IAppComponent, public IKeybordInteractive
	{
	public:
#pragma region Constants
		static const size_t					FrameRate = ANIM_STANDARD::SAMPLE_RATE;
		static const size_t					ScaledMotionTime = ANIM_STANDARD::MAX_CLIP_DURATION; // second

		static const size_t					BufferTime = 5; // second
		static const size_t					BufferFramesCount = FrameRate * BufferTime;

		static const size_t					JointCount = JointType_Count;

		static const size_t					JointDemension = InputFeature::Dimension; // X-Y-Z Position

		static const size_t					MinHz = 3; // MinimumFrequency 3/BufferTime Hz
		static const size_t					MaxHz = 10; // 10 Hz
		static const size_t					HzWidth = MaxHz - MinHz + 1;

		static const size_t					SampleRate = ANIM_STANDARD::SAMPLE_RATE; // Hz
		static const size_t					RecordFeatures = SampleRate * 10U;
		static const size_t					FeatureDimension = InputFeature::Dimension;
		static const size_t					FeatureBufferCaptcity = RecordFeatures * 5;
#pragma endregion

#pragma region Typedefs
		typedef BoneHiracheryFrame frame_type;
		typedef Eigen::Matrix<float, -1, FeatureDimension*JointType_Count, Eigen::RowMajor> FeatureMatrixType;
#pragma endregion

		// Character Map State
		bool							IsMapped() const;
		const CharacterController&		CurrentController() const;
		CharacterController&			CurrentController();
		const CharacterController&		GetController(int state) const;
		CharacterController&			GetController(int state) ;

		virtual void					OnKeyUp(const KeyboardEventArgs&e) override;
		virtual void					OnKeyDown(const KeyboardEventArgs&e) override;
		//struct StateChangedEventArgs
		//{
		//	int OldStateIndex;
		//	int NewStateIndex;
		//	float Confidence;
		//	CharacterController& OldState;
		//	CharacterController& NewState;
		//};
		//Event<const StateChangedEventArgs&> StateChanged;

		// SceneObject interface
		PlayerProxy();
		virtual ~PlayerProxy() override;
		virtual void AddChild(SceneObject* pChild) override;

		// Render / UI Thread 
		void	Update(time_seconds const& time_delta) override;

		// Inherited via IRenderable
		virtual bool IsVisible(const DirectX::BoundingGeometry & viewFrustum) const override;
		virtual void Render(RenderContext & context, DirectX::IEffect* pEffect = nullptr) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

		// PlayerSelector Interface
		const TrackedBodySelector&	GetPlayer() const { return m_playerSelector; }
		TrackedBodySelector&		GetPlayer() { return m_playerSelector; }
		const IArmature&			PlayerArmature() const { return *m_pPlayerArmature; };

	protected:
		void	UpdatePrimaryCameraForTrack();
		// Helper methods
		bool	UpdateByFrame(const BoneHiracheryFrame& frame);

		Eigen::Map<FeatureMatrixType> 
			GetPlayerFeatureMatrix(time_seconds duration);

		void	SetActiveController(int idx);
		int		MapCharacterByLatestMotion();

		friend TrackedBodySelector;
		// Kinect streaming thread
		void StreamPlayerFrame(const TrackedBody& body, const TrackedBody::FrameType& frame);
		void ResetPlayer(TrackedBody* pOld, TrackedBody* pNew);
		void ClearPlayerFeatureBuffer();

	protected:
		bool								m_EnableOverShoulderCam;
		bool								m_IsInitialized;

		std::mutex							m_controlMutex;
		std::atomic_bool					m_mapTaskOnGoing;
		concurrency::task<void>				m_mapTask;

		const IArmature*					m_pPlayerArmature;
		int									m_Id;

		Devices::KinectSensor::Refptr		m_pKinect;
		TrackedBodySelector					m_playerSelector;
		BoneHiracheryFrame							m_CurrentPlayerFrame;
		BoneHiracheryFrame							m_LastPlayerFrame;

		uptr<IArmaturePartFeature>			m_pPlayerFeatureExtrator; // All joints block-Localized gbl-position 

		int									m_CurrentIdx;
		std::list<CharacterController>		m_Controllers;

		std::mutex							m_BufferMutex;

		// This buffer stores the time-re-sampled frame data as feature matrix
		// This buffer is allocated as 30x30 frames size
		// thus , it would be linearized every 30 seconds
		boost::circular_buffer<
			std::array<float,FeatureDimension*JointType_Count>> 
			FeatureBuffer;


	




		// deperacte
		//circular_buffer<frame_type>			FrameBuffer;

		Eigen::VectorXf						StateProbality;
		Eigen::VectorXf						Likilihood;
		Eigen::MatrixXf						TransferMatrix;

		time_seconds						current_time;

	protected:
		// Enter the selecting phase
		void BeginSelectingPhase();
		// End the selecting phase and enter the manipulating phase
		void BeginManipulatingPhase();

		std::pair<float, float> ExtractUserMotionPeriod();

		// Inherited via IRenderable
		virtual RenderFlags GetRenderFlags() const override;
		//void PrintFrameBuffer(int No);
	};

	class KinectVisualizer : public SceneObject, public IRenderable
	{
	public:
		KinectVisualizer();
		// Inherited via IRenderable
		virtual RenderFlags GetRenderFlags() const override;
		virtual bool IsVisible(const DirectX::BoundingGeometry & viewFrustum) const override;
		virtual void Render(RenderContext & context, DirectX::IEffect* pEffect = nullptr) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

	protected:
		std::shared_ptr<Devices::KinectSensor>	pKinect;

	};
}