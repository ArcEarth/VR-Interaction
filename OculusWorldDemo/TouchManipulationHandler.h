#pragma once
#include "Math.hpp"
#include "Common.hpp"

struct ManipulationPose
{
	DirectX::Vector3	Translation;
	DirectX::Quaternion Rotation;
	float				Scale;
};


class TouchManipulationHandler
{
public:
	enum Modes
	{
		None = 0,
		Selective = 0x1,
		Rotation = 0x2,
		Scale = 0x4,
		Translation = 0x8,
		All = 0xf,
	};

	TouchManipulationHandler();
	~TouchManipulationHandler();

	void Start();
	void Finish();
	void Abort();

	void Apply(DirectX::Vector3* TouchPoints);
	const ManipulationPose& Pose() const;
	const ManipulationPose& Delta() const;

};

