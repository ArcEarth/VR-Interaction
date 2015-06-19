#include "pch_bcl.h"
#include "Animations.h"
#include "BoneFeatures.h"

using namespace Causality;
using namespace DirectX;
using namespace Eigen;
using namespace std;


AffineFrame::AffineFrame(size_t size)
	: BaseType(size)
{
}

AffineFrame::AffineFrame(const IArmature & armature)
	: BaseType(armature.default_frame())
{
}

void AffineFrame::RebuildGlobal(const IArmature & armature)
{
	for (auto& joint : armature.joints())
	{
		auto& bone = at(joint.ID());
		if (joint.is_root())
		{
			bone.GblRotation = bone.LclRotation;
			bone.GblScaling = bone.LclScaling;
			bone.EndPostion = bone.LclTranslation;
			bone.LclLength = bone.GblLength = 1.0f; // Length of root doesnot have any meaning
		}
		else
		{
			//bone.OriginPosition = at(joint.ParentID()).EndPostion;

			bone.UpdateGlobalData(at(joint.ParentID()));
		}
	}
}

void Causality::AffineFrame::RebuildLocal(const IArmature & armature)
{
	for (auto& joint : armature.joints())
	{
		auto& bone = at(joint.ID());
		if (joint.is_root())
		{
			bone.LclRotation = bone.GblRotation;
			bone.LclScaling = bone.GblScaling;
			bone.LclTranslation = bone.EndPostion;
			bone.LclLength = bone.GblLength = 1.0f; // Length of root doesnot have any meaning
		}
		else
		{
			bone.UpdateLocalData(at(joint.ParentID()));
		}
	}
}

//Eigen::VectorXf AffineFrame::LocalRotationVector() const
//{
//	Eigen::VectorXf fvector(size() * 3);
//	Eigen::Vector3f v{ 0,1,0 };
//
//	for (size_t i = 0; i < size(); i++)
//	{
//		const auto& bone = (*this)[i];
//		XMVECTOR q = XMLoadFloat4A(reinterpret_cast<const XMFLOAT4A*>(&bone.LclRotation));
//		DirectX::Quaternion lq = XMQuaternionLn(bone.LclRotation);
//		Eigen::Map<const Eigen::Vector3f> elq(&lq.x);
//		fvector.block<3, 1>(i * 3, 0) = elq;
//	}
//
//	return fvector;
//}
//
//void AffineFrame::UpdateFromLocalRotationVector(const IArmature& armature, const Eigen::VectorXf fv)
//{
//	auto& This = *this;
//	auto& sarmature = static_cast<const StaticArmature&>(armature);
//	for (auto i : sarmature.joint_indices())
//	{
//		auto data = fv.block<3, 1>(i * 3, 0).data();
//		This[i].LclRotation = XMQuaternionExp(XMLoadFloat3(reinterpret_cast<const XMFLOAT3*>(data)));
//		if (!armature[i]->is_root())
//		{
//			This[i].UpdateGlobalData(This[armature[i]->ParentID()]);
//		}
//	}
//}

void AffineFrame::Lerp(AffineFrame& out, const AffineFrame & lhs, const AffineFrame & rhs, float t, const IArmature& armature)
{
	//assert((Armature == lhs.pArmature) && (lhs.pArmature == rhs.pArmature));
	for (size_t i = 0; i < lhs.size(); i++)
	{
		out[i].LclRotation = DirectX::Quaternion::Slerp(lhs[i].LclRotation, rhs[i].LclRotation, t);
		out[i].LclScaling = DirectX::XMVectorLerp(lhs[i].LclScaling, rhs[i].LclScaling, t);
		if (!armature[i]->is_root())
		{
			out[i].UpdateGlobalData(out[armature[i]->ParentID()]);
		}
	}
}

void AffineFrame::Blend(AffineFrame& out, const AffineFrame & lhs, const AffineFrame & rhs, float * blend_weights, const IArmature& armature)
{

}

void Causality::AffineFrame::TransformMatrix(DirectX::XMFLOAT3X4 * pOut, const self_type & from, const self_type & to)
{
	using namespace std;
	auto n = min(from.size(), to.size());
	for (int i = 0; i <= n; ++i)
	{
		XMMATRIX mat = Bone::TransformMatrix(from[i], to[i]);
		mat = XMMatrixTranspose(mat);
		XMStoreFloat3x4(pOut + i, mat);
	}
}

void Causality::AffineFrame::TransformMatrix(DirectX::XMFLOAT4X4 * pOut, const self_type & from, const self_type & to)
{
	using namespace std;
	auto n = min(from.size(), to.size());
	for (int i = 0; i <= n; ++i)
	{
		XMMATRIX mat = Bone::TransformMatrix(from[i], to[i]);
		mat = XMMatrixTranspose(mat);
		XMStoreFloat4x4(pOut + i, mat);
	}
}


ArmatureFrameAnimation::ArmatureFrameAnimation(std::istream & file)
{
	using namespace std;
	using namespace DirectX;
	self_type animation;
	int keyframs, joints;
	file >> joints >> keyframs;
	animation.KeyFrames.resize(keyframs);
	for (int i = 0; i < keyframs; i++)
	{
		auto& frame = animation.KeyFrames[i];
		frame.resize(joints);
		double seconds;
		file >> frame.Name;
		file >> seconds;
		frame.Time = (time_seconds)seconds;
		for (auto& bone : frame)
		{
			Vector3 vec;
			auto& scl = bone.LclScaling;
			//char ch;
			file >> vec.x >> vec.y >> vec.z >> scl.y;
			bone.LclRotation = XMQuaternionRotationRollPitchYawFromVector(vec);
		}
	}
}

Causality::ArmatureFrameAnimation::ArmatureFrameAnimation(const std::string & name)
	: base_type(name)
{
}
//
//ArmatureFrameAnimation::self_type& ArmatureFrameAnimation::operator=(const self_type&& rhs)
//{
//	pArmature = rhs.pArmature;
//	frames = std::move(rhs.frames);
//	QrYs = std::move(rhs.QrYs);
//	Ecj = std::move(rhs.Ecj);
//	base_type::operator=(std::move(rhs));
//	return *this;
//}

bool ArmatureFrameAnimation::InterpolateFrames(double frameRate)
{
	float delta = (float)(1.0 / frameRate);
	auto& armature = *pArmature;
	float t = 0;
	for (size_t i = 1; i < KeyFrames.size(); i++)
	{
		const frame_type& lhs = KeyFrames[i - 1];
		const frame_type& rhs = KeyFrames[i];
		for (t = (float)lhs.Time.count(); t < (float)rhs.Time.count(); t += delta)
		{
			frames.emplace_back(armature.size());
			frame_type::Lerp(frames.back(), lhs, rhs, (float)t, armature);
		}
	}
	frames.shrink_to_fit();
	return true;
}

bool Causality::ArmatureFrameAnimation::GetFrameAt(AffineFrame & outFrame, TimeScalarType time) const
{
	double t = fmod(time.count(), Duration.count());
	int frameIdx = round(t / FrameInterval.count());
	frameIdx = frameIdx % frames.size(); // ensure the index is none negative
	outFrame = frames[frameIdx];
	return true;
}

void Causality::ArmatureFrameAnimation::Serialize(std::ostream & binary) const
{
	binary << (uint32_t)bonesCount << (uint32_t)frames.size() << (uint32_t)sizeof(Bone);
	binary << (double)Duration.count() << (double)FrameInterval.count();
	for (auto& frame : frames)
	{
		binary.write(reinterpret_cast<const char*>(frame.data()),
			sizeof(Bone)*bonesCount);
	}
}

void Causality::ArmatureFrameAnimation::Deserialize(std::istream & binary)
{
	uint32_t bC, fC, sB;
	double dur, itv;
	binary >> bC >> fC >> sB >> dur >> itv;

	Duration = time_seconds(dur);
	FrameInterval = time_seconds(itv);

	assert((int)(dur / itv) == fC);
	assert(sB == sizeof(Bone));

	time_seconds time(0);
	frames.resize(fC);
	for (auto& frame : frames)
	{
		frame.Time = time;
		time += FrameInterval;

		frame.resize(bC);
		binary.read(reinterpret_cast<char*>(frame.data()),
			sizeof(Bone)*bonesCount);
	}
}

void ArmatureTransform::TransformBack(frame_type & source_frame, const frame_type & target_frame) const
{
}
