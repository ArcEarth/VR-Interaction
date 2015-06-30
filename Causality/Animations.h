#pragma once
#include "Armature.h"

namespace Causality
{
	typedef time_seconds TimeScalarType;

	enum ANIM_STANDARD
	{
		SAMPLE_RATE = 30, // 30 Hz 
		CLIP_FRAME_COUNT = 90, // 90 Frames for one clip
		MAX_CLIP_DURATION = 3, // 3 second for one cyclic clip
	};

	template <typename Ty>
	struct KeyFrame // Meta data for an animation frame
	{
	public:
		std::string		Name;
		TimeScalarType	Time;
		Ty				Frame;
		//concept static Lerp(self&out, const self& lhs, const self& rhs, float t);

		//concept frame_type& operator[]
	};

	class AffineFrame : public std::vector<Bone, DirectX::AlignedAllocator<Bone>>
	{
	public:
		typedef AffineFrame self_type;

		typedef public std::vector<Bone, DirectX::AlignedAllocator<Bone>> BaseType;
		using BaseType::operator[];
		//using BaseType::operator=;

		AffineFrame() = default;
		explicit AffineFrame(size_t size);
		// copy from default frame
		explicit AffineFrame(const IArmature& armature);
		AffineFrame(const AffineFrame&) = default;
		AffineFrame(AffineFrame&& rhs) { *this = std::move(rhs); }
		AffineFrame& operator=(const AffineFrame&) = default;
		AffineFrame& operator=(AffineFrame&& rhs)
		{
			BaseType::_Assign_rv(std::move(rhs));
			return *this;
		}

		//const IArmature& Armature() const;
		//// Which skeleton this state fram apply for
		//IArmature* pArmature;

		void RebuildGlobal(const IArmature& armature);
		void RebuildLocal(const IArmature& armature);

		// Lerp the local-rotation and scaling, "interpolate in Time"
		static void Lerp(AffineFrame& out, const AffineFrame &lhs, const AffineFrame &rhs, float t, const IArmature& armature);

		// Blend Two Animation Frame, "Blend different parts in Space"
		static void Blend(AffineFrame& out, const AffineFrame &lhs, const AffineFrame &rhs, float* blend_weights, const IArmature& armature);

		static void TransformMatrix(DirectX::XMFLOAT3X4* pOut, const self_type &from, const self_type& to, size_t numOut = 0);
		static void TransformMatrix(DirectX::XMFLOAT4X4* pOut, const self_type &from, const self_type& to, size_t numOut = 0);
		//void BlendMatrixFrom(DirectX::XMFLOAT3X4* pOut, const StateFrame &from)
		//{

		//}

		// number of float elements per bone
		static const auto BoneWidth = sizeof(Bone) / sizeof(float);
		//! not the eigen vector in math !!!
		typedef Eigen::Map<VectorX, Eigen::Aligned> EigenVectorType;

		// return a 20N x 1 column vector of this frame, where 20 = BoneWidth
		EigenVectorType AsEigenVector()
		{
			return EigenVectorType(&(*this)[0].LclRotation.x, size() * BoneWidth);
		}

		typedef Eigen::Map<Eigen::Matrix<float, BoneWidth, Eigen::Dynamic>, Eigen::Aligned> EigenMatrixType;
		// return a 20 x N matrix of this frame
		EigenMatrixType AsEigenMatrix()
		{
			return EigenMatrixType(&(*this)[0].LclRotation.x, BoneWidth, size());
		}
	};

	class BoneVelocityFrame : public std::vector<BoneVelocity>
	{

	};

	class LinearFrame
	{};
	class SpineFrame
	{};
	class DicredFrame
	{};

	class AnimationManager
	{

	};

	template <typename FrameType>
	class IFrameAnimation
	{
	public:
		virtual ~IFrameAnimation()
		{}

		virtual bool GetFrameAt(FrameType& outFrame, TimeScalarType time) const = 0;
	};

	class LinearWarp
	{
	public:
		TimeScalarType operator()(TimeScalarType t)
		{
			return t;
		}
	};


	// The underline resources of an animation, not for "Play" Control
	// Handles Interpolations and warps in Time
	template <typename FrameType>
	class KeyframeAnimation : public IFrameAnimation<FrameType>
	{
	public:
		string								Name;
		std::list<KeyFrame<FrameType>>		KeyFrames;
		TimeScalarType						Duration;
		TimeScalarType						FrameInterval;
		// A function map : (BeginTime,EndTime) -> (BeginTime,EndTime),  that handels the easing effect between frames
		// Restriction : TimeWarp(KeyFrameTime(i)) must equals to it self
		std::function<TimeScalarType(TimeScalarType)> TimeWarpFunction;
	protected:
		const FrameType * m_pDefaultFrame; // Use to generate
	public:
		typedef KeyframeAnimation self_type;

		explicit KeyframeAnimation(const string& name)
			: Name(name)
		{}
		KeyframeAnimation() = default;
		KeyframeAnimation(const self_type& rhs) = default;
		//self_type& operator=(const self_type& rhs) = default;

		KeyframeAnimation(self_type&& rhs) = default;
		//{
		//	*this = std::move(rhs);
		//}

		//self_type& operator=(const self_type&& rhs) = default;
		//{
		//	Name = std::move(rhs.Name);
		//	KeyFrames = std::move(rhs.KeyFrames);
		//	m_pDefaultFrame = rhs.m_pDefaultFrame;
		//	Duration = rhs.Duration;
		//	FrameInterval = rhs.FrameInterval;
		//	TimeWarpFunction = std::move(rhs.TimeWarpFunction);
		//	return *this;
		//}

		const FrameType& DefaultFrame() const { return *m_pDefaultFrame; }
		void SetDefaultFrame(const FrameType &default_frame)
		{
			m_pDefaultFrame = &default_frame;
		}
		// Duration of this clip
		TimeScalarType	Length() const;
		// Is this clip a loopable animation : last frame == first frame
		bool			Loopable()  const;

		// Frame operations
	public:
		bool InsertKeyFrame(TimeScalarType time, const FrameType& frame);
		//bool InsertKeyFrame(TimeScalarType time, FrameType&& frame);
		bool InsertKeyFrame(int idx, const FrameType& frame);
		//bool InsertKeyFrame(int idx, FrameType&& frame);
		bool RemoveKeyFrame(int idx);
		const FrameType& GetKeyFrame(int idx) const;
		bool ReplaceKeyFrame(int idx, const FrameType& frame);
		bool Clear();

	public:
		// Frame Retrival
		virtual bool GetFrameAt(FrameType& outFrame, TimeScalarType time) const;

	};

	template<typename FrameType>
	inline bool KeyframeAnimation<FrameType>::GetFrameAt(FrameType & outFrame, TimeScalarType time) const
	{
		return false;
	}


	class ArmatureFrameAnimation : public KeyframeAnimation<AffineFrame>
	{
	public:
		typedef ArmatureFrameAnimation self_type;
		typedef KeyframeAnimation<AffineFrame> base_type;
		typedef AffineFrame frame_type;

		self_type() = default;
		self_type(const self_type& rhs) = default;
		//self_type& operator=(const self_type& rhs) = default;

		self_type(self_type&& rhs) = default;
		//{
		//	*this = std::move(rhs);
		//}

		explicit self_type(const std::string& name);
		explicit self_type(std::istream& file);

		//self_type& operator=(const self_type&& rhs);

		const IArmature& Armature() const { return *pArmature; }
		void SetArmature(IArmature& armature) { pArmature = &armature; }
		// get the pre-computed frame buffer which contains interpolated frame
		const std::vector<frame_type>& GetFrameBuffer() const { return frames; }
		std::vector<frame_type>& GetFrameBuffer() { return frames; }

		bool InterpolateFrames(double frameRate);
		virtual bool GetFrameAt(AffineFrame& outFrame, TimeScalarType time) const override;

		enum DataType
		{
			LocalRotation = 0,
			GlobalRotation = 2,
			OriginPosition = 3,
			EndPositon = 4,
		};

		// Eigen interface
		//Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::Stride<sizeof(float), sizeof(Bone)>> DataMatrix(DataType data) const
		//{
		//	return Eigen::Map<Eigen::Matrix3Xf, Eigen::Aligned, Eigen::Stride<sizeof(float), sizeof(Bone)>>(&frames[0][0]);
		//}

		void Serialize(std::ostream& binary) const;
		void Deserialize(std::istream& binary);

	private:
		IArmature*			pArmature;
		size_t				bonesCount;
		vector<frame_type>	frames;
	//public:
	//	Eigen::MatrixXf		animMatrix; // 20N x F matrix
	//	Eigen::RowVectorXf  Ecj;	// Jointwise Energy
	//	Eigen::RowVectorXf	Ecb;	// Blockwise Energy
	//	Eigen::Matrix<float,3,-1> Ysp;	// Spatial traits
	//	std::vector<Eigen::MeanThinQr<Eigen::MatrixXf>> QrYs;
	//	std::vector<Eigen::Pca<Eigen::MatrixXf>> PcaYs;
	//	std::vector<Eigen::MatrixXf> Ys;
	};

	class ArmatureTransform
	{
	public:
		virtual ~ArmatureTransform()
		{
		}

		typedef AffineFrame frame_type;

		const IArmature& SourceArmature() const { return *pSource; }
		const IArmature& TargetArmature() const { return *pTarget; }
		void SetSourceArmature(const IArmature& armature) { pSource = &armature; }
		void SetTargetArmature(const IArmature& armature) { pTarget = &armature; }

		int GetBindIndex(int sourceIdx) const;
		int GetInverseBindIndex(int tragetIdx) const;

		virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const = 0;

		virtual void TransformBack(_Out_ frame_type& source_frame, _In_ const frame_type& target_frame) const;

	protected:
		const IArmature	*pSource, *pTarget;
	};

	template <typename FrameType>
	class RealTimeAnimation
	{
		FrameType *m_pDefaultFrame; // Use to generate
		const FrameType& DefaultFrame() const
		{
			return *m_pDefaultFrame;
		}
		void SetDefaultFrame(FrameType *default_frame)
		{
			m_pDefaultFrame = default_frame;
		}
		FrameType& CurrentFrame() { return m_CurrentFrame; }
		const FrameType& CurrentFrame() const { return m_CurrentFrame; }
		FrameType m_CurrentFrame;
	};

	class ArmatureRealTimeAnimation : public RealTimeAnimation<AffineFrame>
	{
	public:
		typedef AffineFrame frame_type;

		const IArmature& Armature() const;

	private:
		IArmature* pArmature;
	};

	enum AnimationPlayState
	{
		Stopped = 0,
		Active = 1,
		Paused = 2,
	};

	enum AnimationRepeatBehavior
	{
		NotRepeat = 0,
		AlwaysRepeat = 1,
	};


	class StoryBoard
	{
	public:
		// Play Control
		void Begin();
		void Stop();
		void Pause();
		void Resume();

		// Play properties
		AnimationRepeatBehavior GetRepeatBehavior() const;
		void SetRepeatBehavior(AnimationRepeatBehavior behavior);

		double GetSpeedRatio() const;
		void SetSpeedRatio(double ratio) const;

		// Timeline basic
		TimeScalarType GetBeginTime() const;
		void SetBeginTime(TimeScalarType time);
		TimeScalarType GetEndTime() const;
		void SetEndTime(TimeScalarType time);
		TimeScalarType GetDuration();

		// Time seeking
		AnimationPlayState GetCurrentState() const;
		TimeScalarType GetCurrentTime() const; // Shit! Name Collision with Windows Header!!! Dame te outdated C-style header!!!
		void AdvanceTimeBy(TimeScalarType time);
		void Seek(TimeScalarType time);

		TypedEvent<StoryBoard> Completed;

		//std::vector<IFrameAnimation*> ChildrenAnimation;
	protected:
		AnimationPlayState CurrentState;
	};
}