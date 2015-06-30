#pragma once
#include "CharacterObject.h"
#include "Kinect.h"
#include <boost\circular_buffer.hpp>
#include "Animations.h"

namespace Causality
{
	using boost::circular_buffer;

	class AnimationAnalyzer;

	class CharacterController
	{
	public:
		~CharacterController();
		void Initialize(const IArmature& player, CharacterObject& character);

		const ArmatureTransform& Binding() const;
		ArmatureTransform& Binding();
		void SetBinding(ArmatureTransform* pBinding);

		const CharacterObject& Character() const;
		CharacterObject& Character();

		void UpdateTargetCharacter(const AffineFrame& sourceFrame) const;

		int						ID;
		AffineFrame				PotientialFrame;
		float					SpatialMotionScore;
		Vector3					MapRefPos;
		Vector3					CMapRefPos;

		AnimationAnalyzer& GetAnimationInfo(const string& name);

	public:
		CharacterObject*							m_pCharacter;
		map<string, AnimationAnalyzer*>				m_Analyzers;
		uptr<ArmatureTransform>						m_pBinding;

		void SetSourceArmature(const IArmature& armature);

		void SetTargetCharacter(CharacterObject& object);

	};

	class PlayerProxy : public SceneObject, public IRenderable, public IAppComponent, public IKeybordInteractive
	{
	public:
#pragma region Constants
		static const size_t					FrameRate = ANIM_STANDARD::SAMPLE_RATE;
		static const size_t					ScaledMotionTime = ANIM_STANDARD::MAX_CLIP_DURATION; // second
		static const size_t					StretchedSampleCount = ANIM_STANDARD::CLIP_FRAME_COUNT;

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
		typedef AffineFrame frame_type;
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
		virtual bool IsVisible(const BoundingFrustum & viewFrustum) const override;
		virtual void Render(RenderContext & context) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

		// PlayerSelector Interface
		const TrackedBodySelector&	GetPlayer() const { return playerSelector; }
		TrackedBodySelector&		GetPlayer() { return playerSelector; }
		const IArmature&			PlayerArmature() const { return *pPlayerArmature; };

	protected:
		// Helper methods
		bool	UpdateByFrame(const AffineFrame& frame);

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
		bool								IsInitialized;

		const IArmature*					pPlayerArmature;
		int									Id;

		Devices::KinectSensor::Refptr		pKinect;
		TrackedBodySelector					playerSelector;
		

		int									CurrentIdx;
		std::list<CharacterController>		Controllers;

		std::mutex							BufferMutex;

		// This buffer stores the time-re-sampled frame data as feature matrix
		// This buffer is allocated as 30x30 frames size
		// thus , it would be linearized every 30 seconds
		boost::circular_buffer<
			std::array<float,FeatureDimension*JointType_Count>> 
			FeatureBuffer;


	




		// deperacte
		//circular_buffer<frame_type>			FrameBuffer;

		VectorX								StateProbality;
		VectorX								Likilihood;
		MatrixX								TransferMatrix;

	protected:
		// Enter the selecting phase
		void BeginSelectingPhase();
		// End the selecting phase and enter the manipulating phase
		void BeginManipulatingPhase();

		std::pair<float, float> ExtractUserMotionPeriod();
		//void PrintFrameBuffer(int No);
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
		std::shared_ptr<Devices::KinectSensor>	pKinect;
	};
}