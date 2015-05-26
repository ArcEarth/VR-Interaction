#ifndef	KINECT_2_USER
#define KINECT_2_USER

#ifdef _MSC_VER
#pragma once
#endif  // _MSC_VER
#include "BCL.h"
#include <wrl\client.h>
#include <memory>
#include "Common\DirectXMathExtend.h"
#include "Armature.h"
#include "Common\Filter.h"
#include <boost\circular_buffer.hpp>
namespace Causality
{
#ifndef _HandType_
#define _HandType_
	typedef enum _HandType HandType;


	enum _HandType
	{
		HandType_NONE = 0,
		HandType_LEFT = (HandType_NONE + 1),
		HandType_RIGHT = (HandType_LEFT + 1)
	};
#endif // _HandType_
#ifndef _HandState_
#define _HandState_
	typedef enum _HandState HandState;


	enum _HandState
	{
		HandState_Unknown = 0,
		HandState_NotTracked = 1,
		HandState_Open = 2,
		HandState_Closed = 3,
		HandState_Lasso = 4
	};
#endif // _HandState_

#ifndef _JointType_
#define _JointType_
	typedef enum _JointType JointType;


	enum _JointType
	{
		JointType_SpineBase = 0,
		JointType_SpineMid = 1,
		JointType_Neck = 2,
		JointType_Head = 3,
		JointType_ShoulderLeft = 4,
		JointType_ElbowLeft = 5,
		JointType_WristLeft = 6,
		JointType_HandLeft = 7,
		JointType_ShoulderRight = 8,
		JointType_ElbowRight = 9,
		JointType_WristRight = 10,
		JointType_HandRight = 11,
		JointType_HipLeft = 12,
		JointType_KneeLeft = 13,
		JointType_AnkleLeft = 14,
		JointType_FootLeft = 15,
		JointType_HipRight = 16,
		JointType_KneeRight = 17,
		JointType_AnkleRight = 18,
		JointType_FootRight = 19,
		JointType_SpineShoulder = 20,
		JointType_HandTipLeft = 21,
		JointType_ThumbLeft = 22,
		JointType_HandTipRight = 23,
		JointType_ThumbRight = 24,
		JointType_Count = (JointType_ThumbRight + 1)
	};
#endif // _JointType_

	template <class _Ty, class _Alloc = std::allocator<_Ty>>
	class BufferedStreamViewer
	{
	public:
		typedef const _Ty* const_pointer;
		typedef _Ty* pointer;
		typedef const _Ty& const_reference;
		typedef _Ty &  reference;
		typedef _Ty && rvalue_reference;

		//enum StreamViewMode
		//{
		//	Latest = 0,
		//	SequenceCaching = 1,
		//}

		BufferedStreamViewer(size_t BufferSize = 65536U)
			: m_paused(false) , m_Capicity(BufferSize)
		{
		}

		BufferedStreamViewer(const BufferedStreamViewer& rhs)
		{
			m_paused = rhs.m_paused;
			std::lock_guard<std::mutex> guard(rhs.m_BufferMutex);
			m_StreamingBuffer = rhs.m_StreamingBuffer;
		}

		BufferedStreamViewer(BufferedStreamViewer&& rhs) = default;

		~BufferedStreamViewer(void)
		{
			m_paused = true;
		}

		void Push(const_reference frame)
		{
			if (m_paused) return;

			std::lock_guard<std::mutex> guard(m_BufferMutex);
			if (m_StreamingBuffer.size() >= m_StreamingBuffer.max_size())
				m_StreamingBuffer.pop_front();
			m_StreamingBuffer.emplace_back(frame);
		}

		void Push(rvalue_reference frame)
		{
			if (m_paused) return;

			std::lock_guard<std::mutex> guard(m_BufferMutex);
			if (m_StreamingBuffer.size() >= m_Capicity)
				m_StreamingBuffer.pop_front();
			m_StreamingBuffer.emplace_back(std::move(frame));
		}

		pointer GetNext() const
		{
			if (m_StreamingBuffer.empty()) return nullptr;

			std::lock_guard<std::mutex> guard(m_BufferMutex);

			m_ReadingBuffer = m_StreamingBuffer.front();
			m_StreamingBuffer.pop_front();

			return &m_ReadingBuffer;
		}

		pointer GetCurrent() const
		{
			return &m_ReadingBuffer;
		}

		pointer GetLatest() const 
		{
			if (m_StreamingBuffer.empty()) return &m_ReadingBuffer;

			std::lock_guard<std::mutex> guard(m_BufferMutex);

			m_ReadingBuffer = m_StreamingBuffer.back();
			m_StreamingBuffer.clear();

			return &m_ReadingBuffer;
		}

		std::deque<_Ty>& LockBuffer()
		{
			m_BufferMutex.lock();
			return m_StreamingBuffer;
		}

		void UnlockBuffer()
		{
			m_BufferMutex.unlock();
		}

		bool empty() const
		{
			return m_StreamingBuffer.empty();
		}

		void Pause(bool pause)
		{
			m_paused = pause;
		}

	private:
		bool						m_paused;
		size_t						m_Capicity;
		mutable _Ty					m_ReadingBuffer;
		mutable std::deque<_Ty, _Alloc >	m_StreamingBuffer;
		mutable std::mutex			m_BufferMutex;
	};


	struct TrackedBody
	{
	public:
		TrackedBody();
		TrackedBody(const TrackedBody &) = default;
		TrackedBody(TrackedBody &&) = default;

		typedef Causality::BoneDisplacementFrame FrameType;

		void PushFrame(FrameType && frame);

		Causality::BoneDisplacementFrame& GetPoseFrame() const
		{
			return *PoseBuffer.GetLatest();
		}


	public:
		// Skeleton Structure and basic body parameter
		// Shared through all players since they have same structure
		static std::unique_ptr<StaticArmature> BodyArmature;

		void AddRef() {
			++RefCount;
		}

		void Release() {
			--RefCount;
		}

		// Default pose data, should we use this ?
		// Causality::BoneDisplacementFrame RestFrame;

		// Current pose data
		typedef Causality::BoneDisplacementFrame FrameType;
		BufferedStreamViewer<FrameType>	 PoseBuffer;

		static const size_t SampleRate = 30U; // Hz

		static const size_t FeatureDimension = 6U;
		Eigen::Map<Eigen::Matrix<float, FeatureDimension*JointType_Count, -1>> GetTrajectoryMatrix(time_seconds duration);

		static const size_t RecordFeatures = SampleRate * 10U;
		static const size_t FeatureBufferCaptcity = RecordFeatures * 3;
		// This buffer stores the time-re-sampled frame data as feature matrix
		// This buffer is allocated as 30x30 frames size
		// thus , it would be linearized every 30 seconds
		boost::circular_buffer<std::array<float, FeatureDimension*JointType_Count>> FeatureBuffer;

		//Causality::BoneDisplacementFrame PoseFrame;

		// Hand States
		std::array<HandState,2>	HandStates;

		int			RefCount;

		uint64_t	Id;
		bool		IsCurrentTracked;

		time_t		LastTrackedTime;
		int			LostFrameCount;

		typedef LowPassDynamicFilter<Vector3, float> Vector3DynamicFilter;

		//std::array<Vector3DynamicFilter, JointType_Count> JointFilters;

		Event<const TrackedBody&, HandType, HandState>	OnHandStateChanged;
		Event<const TrackedBody&>						OnPoseChanged;
	};


	namespace Devices
	{

		// An aggregate of Kinect Resources
		class Kinect
		{
		public:
			static std::weak_ptr<Kinect> wpCurrentDevice;
			static std::shared_ptr<Kinect> GetForCurrentView();

			const StaticArmature& Armature() const { return *TrackedBody::BodyArmature; }

			~Kinect();

			bool IsConnected() const;

			// Player Event event interface!
			Event<TrackedBody&> OnPlayerTracked;
			Event<TrackedBody&> OnPlayerLost;
			//Platform::Fundation::Event<const TrackedBody&> OnPlayerPoseChanged;
			//Platform::Fundation::Event<const TrackedBody&, HandEnum, HandState> OnPlayerHandStateChanged;

			// DeviceCoordinate Matrix will be use to transform all input data every frame
			// If Kinect is moving, this function should be called every frame
			// Should be valiad before the call to Process Frame
			void SetDeviceCoordinate(const DirectX::Matrix4x4& mat);
			const DirectX::Matrix4x4& GetDeviceCoordinate();

			// Process frame and trigger the events
			void ProcessFrame();

			bool Start();
			void Stop();
			bool Pause();
			bool Resume();

			// Return the list of CURRENT Tracked bodies
			const std::list<TrackedBody> &GetTrackedBodies();

			// Static Constructors!!!
			static std::shared_ptr<Kinect> CreateDefault();
		protected:

			Kinect();

		public:
			static JointType JointsParent[JointType_Count];

		private:

			class Impl;
			std::unique_ptr<Impl> pImpl;

		};
		
	}
}


#endif


